/* Copyright (c) <2018-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// dTimeTracker.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "dTimeTracker.h"
#include "dTimeTarckerRecord.h"

#if _MSC_VER < 1900
#define thread_local __declspec(thread)
#endif

#define DG_TIME_TRACKER_PAGE_ENTRIES (1<<DG_TIME_TRACKER_ENTRIES_POWER)

class dTrackeString
{
	public:
	dTrackeString(const char* const string)
	{
		strcpy (m_string, string);
	}

	dTrackeString(const dTrackeString& src)
	{
		strcpy (m_string, src.m_string);
	}
	char m_string[128];
};

class dTimeTrack
{
	public:
	dTimeTrack(const char* const name)
		:m_count(0)
	{
		m_banks[0] = DG_TIME_TRACKER_PAGE_ENTRIES;
		m_banks[1] = DG_TIME_TRACKER_PAGE_ENTRIES;
		strncpy (m_threadName, name, sizeof (m_threadName) - 1);
	}

	~dTimeTrack()
	{
	}

	void Clear()
	{
		m_count = 0;
		m_banks[0] = DG_TIME_TRACKER_PAGE_ENTRIES;
		m_banks[1] = DG_TIME_TRACKER_PAGE_ENTRIES;
		memset (m_buffer, 0, sizeof (m_buffer));
		m_nameMap.RemoveAll();
	}

	int AddEntry(const char* const name);
	void CloseEntry(int index);

	void SetName(const char* const name)
	{
		strncpy(m_threadName, name, sizeof (m_threadName) - 1);
	}

	const dTree<dTrackeString, dCRCTYPE>& GetStringMap() const
	{
		return m_nameMap;
	}

	const dTimeTarckerRecord* GetBuffer() const
	{
		return m_buffer;
	}

	private:
	void SaveBuffer(int bank);

	int m_count;
	int m_banks[2];
	dTimeTarckerRecord m_buffer[DG_TIME_TRACKER_PAGE_ENTRIES * 2];
	dTree<dTrackeString, dCRCTYPE> m_nameMap;
	char m_threadName[64];
};


class dTimeTrackerServer
{
	public:
	dTimeTrackerServer()
		:m_initialized(false)
		,m_socket(0)
		,m_trackEnum(0)
		,m_tracks()
		,m_baseCount(__rdtsc())
		,m_currentFile(NULL)
	{
		memset(&m_client, 0, sizeof(m_client));
		memset(&m_server, 0, sizeof(m_server));
	}

	~dTimeTrackerServer()
	{
		while (m_tracks.GetCount()) {
			dTree<dTimeTrack*, DWORD>::dTreeNode* const node = m_tracks.GetRoot();
			delete node->GetInfo();
			m_tracks.Remove(node);
		}

		if (m_initialized) {
			closesocket(m_socket);
			WSACleanup();
		}
	}

	static dTimeTrackerServer& GetServer()
	{
		static dTimeTrackerServer server;
		return server;
	}

	unsigned GetTime() const
	{
		return unsigned ((__rdtsc() - m_baseCount) >> 10);
	}

	bool StartServer()
	{
		if (!m_initialized) {
			// initialized win socket
			WORD version = MAKEWORD(2, 2);
			int state = WSAStartup(version, &m_wsaData);
			if (!state) {
				// bind socket to ip address
				m_socket = socket(AF_INET, SOCK_DGRAM, 0);
				
				m_server.sin_addr.S_un.S_addr = ADDR_ANY;
				m_server.sin_family = AF_INET;
				if (!bind(m_socket, (SOCKADDR*) &m_server, sizeof(m_server))) {
					m_initialized = true;
				}
			}
		}
		return m_initialized;
	}

	bool IsValid() const
	{
		return m_initialized;
	}

	dTimeTrack& GetFrame()
	{
		// I have to do this because VS 2013 do not fully supports thread_local initialization 
		static thread_local dTimeTrack* frame = NULL;
		if (!frame) {
			char name[64];
			sprintf(name, "thread_%2d", m_trackEnum);
			frame = new dTimeTrack (name);
			DWORD threadID = GetThreadId(GetCurrentThread());
			m_tracks.Insert(frame, threadID);
			m_trackEnum ++;
		}
		return *frame;
	}

	void DeleteTrack()
	{
		DWORD threadID = GetThreadId(GetCurrentThread());
		dTree<dTimeTrack*, DWORD>::dTreeNode* const node = m_tracks.Find(threadID);
		if (node) {
			delete node->GetInfo();
			m_tracks.Remove(node);
		}
	}

	void StartRecording(const char* const fileName)
	{
		dTree<dTimeTrack*, DWORD>::Iterator iter(m_tracks);
		for (iter.Begin(); iter; iter++) {
			dTimeTrack* const track = iter.GetNode()->GetInfo();
			track->Clear();
		}

		if (m_currentFile) {
			StopRecording();
		}

		m_currentFile = fopen (fileName, "wb");
		dAssert(m_currentFile);
		m_baseCount = __rdtsc();
	}

	void StopRecording()
	{
		dAssert(m_currentFile);
		dTree<int, dCRCTYPE> filter;
		dTree<dTimeTrack*, DWORD>::Iterator iter(m_tracks);
		for (iter.Begin(); iter; iter++) {
			dTimeTrack* const track = iter.GetNode()->GetInfo();
			dTree<dTrackeString, dCRCTYPE>::Iterator nameIter (track->GetStringMap());
			for (nameIter.Begin(); nameIter; nameIter++) {
				dCRCTYPE key = nameIter.GetKey();
				if (!filter.Find(key)) {
					const dTrackeString& name = nameIter.GetNode()->GetInfo();
					int size = strlen(name.m_string);
					fwrite(&key, sizeof(dCRCTYPE), 1, m_currentFile);
					fwrite(&size, sizeof(int), 1, m_currentFile);
					fwrite(name.m_string, size,1,  m_currentFile);
					filter.Insert(0, key);
				}
			}
		}
		
		fclose(m_currentFile);
		m_currentFile = NULL;
	}

	void SaveTrack(dTimeTrack& track, int bank)
	{
		int sizeInByte = sizeof(dTimeTarckerRecord) * DG_TIME_TRACKER_PAGE_ENTRIES;
		Bytef* const buffer = dAlloca(Bytef, sizeInByte);

		uLongf destLen;
		const dTimeTarckerRecord* const trackBuffer = track.GetBuffer();
		int compressError = compress(buffer, &destLen, (Bytef*)&trackBuffer[bank * DG_TIME_TRACKER_PAGE_ENTRIES], sizeInByte);
		dAssert(compressError == Z_OK);

		fwrite(&destLen, sizeof(uLongf), 1, m_currentFile);
		fwrite(buffer, destLen, 1, m_currentFile);
	}

	bool m_initialized;
	SOCKET m_socket;
	SOCKADDR_IN m_client;
	SOCKADDR_IN m_server;
	WSADATA m_wsaData;

	int m_trackEnum;
	dTree<dTimeTrack*, DWORD> m_tracks;

	DWORD64 m_baseCount;
	FILE* m_currentFile;
};

bool StartServer()
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	return server.StartServer();
}

void ttStartRecording(const char* const fileName)
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	server.StartRecording(fileName);
}

void ttStopRecording()
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	server.StopRecording();
}


void ttSetTrackName(const char* const threadName)
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	dTimeTrack& frame = server.GetFrame();
	frame.SetName(threadName);
}

void ttDeleteTrack()
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	server.DeleteTrack();
}

int ttOpenRecord(const char* const name)
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	dTimeTrack& frame = server.GetFrame();
	return frame.AddEntry(name);
}

void ttCloseRecord(int recordIndex)
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	dTimeTrack& frame = server.GetFrame();
	frame.CloseEntry(recordIndex);
}

int dTimeTrack::AddEntry(const char* const name)
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	int index = m_count;
	dTimeTarckerRecord& record = m_buffer[index];
	record.m_start = server.GetTime();

	dCRCTYPE nameHash = dCRC64(name);
	m_nameMap.Insert(name, nameHash);
	record.m_nameHash = nameHash;

	m_count = (m_count + 1) & (DG_TIME_TRACKER_PAGE_ENTRIES * 2 - 1);
	return index;
}

void dTimeTrack::CloseEntry(int index)
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	dTimeTarckerRecord& record = m_buffer[index];
	record.m_duration = server.GetTime() - record.m_start;

	int bank = index >> DG_TIME_TRACKER_ENTRIES_POWER;
	m_banks[bank]--;
	dAssert(m_banks[bank] >= 0);
	if (!m_banks[bank]) {
		SaveBuffer(bank);
	}
}

void dTimeTrack::SaveBuffer(int bank)
{
	dAssert((bank && (m_count < DG_TIME_TRACKER_PAGE_ENTRIES)) || (!bank && (m_count >= DG_TIME_TRACKER_PAGE_ENTRIES)));
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	if (server.m_currentFile) {
		server.SaveTrack(*this, bank);
	}
	m_banks[bank] = DG_TIME_TRACKER_PAGE_ENTRIES;
}
