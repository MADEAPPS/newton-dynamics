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

#if _MSC_VER < 1900
#define thread_local __declspec(thread)
#endif


#define DG_MAX_ENTRIES (1<<11)

class dTimeTrackeFrame
{
	public:
	dTimeTrackeFrame()
		:m_count(0)
		,m_baseCount(0)
	{
		m_buffer = new dTimeTarckerRecord[DG_MAX_ENTRIES * 2];
	}

	~dTimeTrackeFrame()
	{
		delete[] m_buffer;
	}

	int AddEntry(const char* const name)
	{
		int index = m_count - m_baseCount;
		if ((index + 1) == DG_MAX_ENTRIES * 2) {
			SaveBuffer();
			index = m_count - m_baseCount;
		}
		dTimeTarckerRecord& record = m_buffer[index];
		record.m_start = __rdtsc();

		dCRCTYPE nameHash = dCRC64(name);
		m_nameMap.Insert (name, nameHash);
		record.m_nameHash = nameHash;

		m_count++;
		return m_count - 1;
	}

	void CloseEntry(int index)
	{
		dTimeTarckerRecord& record = m_buffer[m_count - m_baseCount];
		record.m_end = __rdtsc();
	}

	private:
	void SaveBuffer()
	{
		memcpy(m_buffer, m_buffer + DG_MAX_ENTRIES, DG_MAX_ENTRIES * sizeof(dTimeTarckerRecord));
		m_baseCount += DG_MAX_ENTRIES;
	}

	int m_count;
	int m_baseCount;
	dTimeTarckerRecord* m_buffer;
	dTree<dString, dCRCTYPE> m_nameMap;
};


static dTimeTrackeFrame& GetFrame()
{
//	thread_local static dTimeTrackeFrame frame;
	static dTimeTrackeFrame frame;
	return frame;
}


class dTimeTrackerServer
{
	public:
	dTimeTrackerServer()
		:m_initialized(false)
		,m_socket(-1)
	{
		memset(&m_client, 0, sizeof(m_client));
		memset(&m_server, 0, sizeof(m_server));
	}

	~dTimeTrackerServer()
	{
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

	bool m_initialized;
	SOCKET m_socket;
	SOCKADDR_IN m_client;
	SOCKADDR_IN m_server;
	WSADATA m_wsaData;
};

bool StartServer()
{
	dTimeTrackerServer& server = dTimeTrackerServer::GetServer();
	return server.StartServer();
}

int ttOpenRecord(const char* const name)
{
	dTimeTrackeFrame& frame = GetFrame();
	return frame.AddEntry(name);
}

void ttCloseRecord(int recordIndex)
{
	dTimeTrackeFrame& frame = GetFrame();
	frame.CloseEntry(recordIndex);
}
