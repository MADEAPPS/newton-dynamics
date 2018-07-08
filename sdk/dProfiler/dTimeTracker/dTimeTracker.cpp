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
#include "dTimeTrackerRecord.h"
#include "dTimeTrackerMap.h"

#if _MSC_VER < 1900
#define thread_local __declspec(thread)
#endif

#define DG_TIME_TRACKER_PAGE_ENTRIES (1<<DG_TIME_TRACKER_ENTRIES_POWER)

class dTimeTrack
{
	public:
	class dTrackerString
	{
		public:
		dTrackerString(const char* const string)
		{
			Copy(string);
		}

		dTrackerString(const dTrackerString& src)
		{
			Copy(src.m_string);
		}

		void Copy(const char* const src)
		{
			int i = 0;
			do {
				m_string[i] = src[i];
			} while (src[i++]);
		}
		char m_string[128];
	};

	class dTimeTrackerRecord
	{
		public:
		unsigned m_start;
		unsigned m_duration;
		unsigned m_nameHash;
	};

	dTimeTrack(const char* const name)
		:m_count(0)
		,m_threadName (name)
	{
		m_banks[0] = DG_TIME_TRACKER_PAGE_ENTRIES;
		m_banks[1] = DG_TIME_TRACKER_PAGE_ENTRIES;
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
		//m_nameMap.clear();
	}

	int AddEntry(const char* const name);
	void CloseEntry(int index);

	void SetName(const char* const name)
	{
		m_threadName = name;
	}

	const dTrackerString& GetName() const
	{
		return m_threadName;
	}

	dTimeTrackerMap<dTrackerString, unsigned>& GetStringMap()
	{
		return m_nameMap;
	}

	const dTimeTrackerRecord* GetBuffer() const
	{
		return m_buffer;
	}

	private:
	void SaveBuffer(int bank);

	int m_count;
	int m_banks[2];
	dTimeTrackerRecord m_buffer[DG_TIME_TRACKER_PAGE_ENTRIES * 2];
	dTimeTrackerMap<dTrackerString, unsigned> m_nameMap;
	dTrackerString m_threadName;
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
			dTimeTrackerMap<dTimeTrack*, DWORD>::dTreeNode* const node = m_tracks.GetRoot();
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
		dTimeTrackerMap<dTimeTrack*, DWORD>::dTreeNode* const node = m_tracks.Find(threadID);
		if (node) {
			delete node->GetInfo();
			m_tracks.Remove(node);
		}
	}

	void StartRecording(const char* const fileName)
	{
		dTimeTrackerMap<dTimeTrack*, DWORD>::Iterator iter(m_tracks);
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
		dTimeTrackerMap<unsigned, unsigned> filter;
		dTimeTrackerMap<dTimeTrack*, DWORD>::Iterator iter(m_tracks);
		int chunkType = m_traceLabel;
		fwrite(&chunkType, sizeof(unsigned), 1, m_currentFile);
		for (iter.Begin(); iter; iter++) {
			dTimeTrack* const track = iter.GetNode()->GetInfo();
			dTimeTrackerMap<dTimeTrack::dTrackerString, unsigned>& nameMap = track->GetStringMap();

			const dTimeTrack::dTrackerString& threadName = track->GetName();
			unsigned threadNameCrc = unsigned(dCRC64(threadName.m_string));
			nameMap.Insert(threadName, threadNameCrc);

			dTimeTrackerMap<dTimeTrack::dTrackerString, unsigned>::Iterator nameIter (nameMap);
			for (nameIter.Begin(); nameIter; nameIter++) {
				unsigned key = nameIter.GetKey();
				if (!filter.Find(key)) {
					const dTimeTrack::dTrackerString& name = nameIter.GetNode()->GetInfo();
					int size = strlen(name.m_string);
					fwrite(&chunkType, sizeof(unsigned), 1, m_currentFile);
					fwrite(&key, sizeof(unsigned), 1, m_currentFile);
					fwrite(&size, sizeof(unsigned), 1, m_currentFile);
					fwrite(name.m_string, size,1,  m_currentFile);
					filter.Insert(key, key);
				}
			}
		}

		chunkType = 0;
		fwrite(&chunkType, sizeof(int), 1, m_currentFile);

		chunkType = m_traceEnd;
		fwrite(&chunkType, sizeof(int), 1, m_currentFile);
		fclose(m_currentFile);

		m_currentFile = NULL;
	}

	void SaveTrack(dTimeTrack& track, int bank)
	{
		int sizeInByte = sizeof(dTimeTrack::dTimeTrackerRecord) * DG_TIME_TRACKER_PAGE_ENTRIES;
		Bytef* const buffer = dAlloca(Bytef, sizeInByte);

		uLongf destLen;
		const dTimeTrack::dTimeTrackerRecord* const trackBuffer = track.GetBuffer();
		int compressError = compress(buffer, &destLen, (Bytef*)&trackBuffer[bank * DG_TIME_TRACKER_PAGE_ENTRIES], sizeInByte);
		dAssert(compressError == Z_OK);

		int chunkType = m_traceSamples;
		int size = unsigned (destLen);
		const dTimeTrack::dTrackerString& threadName = track.GetName();
		unsigned threadNameCrc = unsigned(dCRC64(threadName.m_string));

		fwrite(&chunkType, sizeof(unsigned), 1, m_currentFile);
		fwrite(&threadNameCrc, sizeof(unsigned), 1, m_currentFile);
		fwrite(&size, sizeof(unsigned), 1, m_currentFile);
		fwrite(buffer, size, 1, m_currentFile);
	}

	bool m_initialized;
	SOCKET m_socket;
	SOCKADDR_IN m_client;
	SOCKADDR_IN m_server;
	WSADATA m_wsaData;

	int m_trackEnum;
	dTimeTrackerMap<dTimeTrack*, DWORD> m_tracks;

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
	dTimeTrackerRecord& record = m_buffer[index];
	record.m_start = server.GetTime();

	unsigned  nameHash = unsigned  (dCRC64(name));
	m_nameMap.Insert(name, nameHash);
	record.m_nameHash = nameHash;

	m_count = (m_count + 1) & (DG_TIME_TRACKER_PAGE_ENTRIES * 2 - 1);
	return index;
}

void dTimeTrack::CloseEntry(int index)
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	dTimeTrackerRecord& record = m_buffer[index];
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
