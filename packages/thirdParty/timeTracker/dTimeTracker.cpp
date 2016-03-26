
/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include "stdafx.h"
// need to include stdafx.h first so that visual studio do no issues a precompiled header error 
#if defined (D_TIME_TRACKER) && defined(_MSC_VER)

#include "dTimeTracker.h"

dTimeTracker* dTimeTracker::GetInstance()
{
	static dTimeTracker instance;
	return &instance;
}


dTimeTracker::dTimeTracker ()
{
	m_file = NULL;
	m_firstRecord = 0;
	m_numberOfFrames = 0;
	m_processId = GetCurrentProcessId();
	QueryPerformanceFrequency(&m_frequency);
	QueryPerformanceCounter (&m_baseCount);

	InitializeCriticalSectionAndSpinCount(&m_criticalSection, 0x1000);

	m_bufferIndex = 0;
	m_bufferSize = 1024;
	m_buffer = ((dTrackEntry*) dContainersAlloc::Alloc (m_bufferSize * sizeof (dTrackEntry)));

	m_outQueueBufferindex = 0;
	m_outQueueBufferSize = 1024 * 512;
	m_outQueueBuffer = ((dTrackEntry*) dContainersAlloc::Alloc (m_outQueueBufferSize * sizeof (dTrackEntry)));
}

dTimeTracker::~dTimeTracker ()
{
	dContainersAlloc::Free (m_buffer);
	dContainersAlloc::Free (m_outQueueBuffer);
	DeleteCriticalSection(&m_criticalSection);
}

void dTimeTracker::StartSection (int numberOfFrames)
{
	if (m_file) {
		fclose (m_file);
	}

	time_t rawtime;
	char fileName [80];

	time (&rawtime);
	struct tm * timeinfo = localtime (&rawtime);
	strftime (fileName, sizeof(fileName), "profile_%H%M%S%m%d%Y.json", timeinfo);
	
	m_file = fopen (fileName, "wb");

	fprintf (m_file, "{\n");
	fprintf (m_file, "\t\"traceEvents\": [");

	QueryPerformanceFrequency(&m_frequency);
	QueryPerformanceCounter (&m_baseCount);
	m_numberOfFrames = numberOfFrames;	
}

void dTimeTracker::EndSection ()
{
	EnterCriticalSection(&m_criticalSection); 
	FlushQueue ();

	fprintf (m_file, "\n");
	fprintf (m_file, "\t],\n");

	fprintf (m_file, "\t\"displayTimeUnit\": \"ns\",\n");
	fprintf (m_file, "\t\"systemTraceEvents\": \"SystemTraceData\",\n");
	fprintf (m_file, "\t\"otherData\": {\n");
	fprintf (m_file, "\t\t\"version\": \"My Application v1.0\"\n");
	fprintf (m_file, "\t}\n");
	fprintf (m_file, "}\n");

	fclose (m_file);
	m_file = NULL;
	m_firstRecord = 0;
	LeaveCriticalSection(&m_criticalSection);
}

void dTimeTracker::Update ()
{
	if (m_file) {
		QueueEvents ();
		m_numberOfFrames --;
		if (m_numberOfFrames == 0) {
			EndSection ();
		}
	}
}


long long dTimeTracker::GetTimeInNanosenconds()
{
	LARGE_INTEGER count;
	QueryPerformanceCounter (&count);
	count.QuadPart -= m_baseCount.QuadPart;
	LONGLONG ticks = LONGLONG (count.QuadPart * LONGLONG (1000000000) / m_frequency.QuadPart);
	return ticks;
}

dCRCTYPE dTimeTracker::RegisterName (const char* const name)
{
	dCRCTYPE crc = dCRC64 (name);

	EnterCriticalSection(&m_criticalSection); 
	dTree<dLabels, dCRCTYPE>::dTreeNode* const node = m_dictionary.Insert (crc);
	LeaveCriticalSection(&m_criticalSection);
	if (node) {
		strncpy (&node->GetInfo().m_name[0], name, sizeof (node->GetInfo().m_name));  
	}
	return crc;
}

TIMETRACKER_API void dTimeTracker::RegisterThreadName (const char* const threadName)
{
	dCRCTYPE crc = GetCurrentThreadId();
	EnterCriticalSection(&m_criticalSection); 
	dTree<dLabels, dCRCTYPE>::dTreeNode* const node = m_dictionary.Insert (crc);
	LeaveCriticalSection(&m_criticalSection);
	if (node) {
		strncpy (&node->GetInfo().m_name[0], threadName, sizeof (node->GetInfo().m_name));  
	}
}	

void dTimeTracker::QueueEvents ()
{
	EnterCriticalSection(&m_criticalSection); 
	if ((m_outQueueBufferindex + m_bufferIndex) > m_outQueueBufferSize) {
		FlushQueue ();
		m_outQueueBufferindex = 0;
	}
	memcpy (&m_outQueueBuffer[m_outQueueBufferindex], m_buffer, m_bufferIndex * sizeof (dTrackEntry));
	m_outQueueBufferindex += m_bufferIndex;
	m_bufferIndex = 0;

	LeaveCriticalSection(&m_criticalSection);
}


void dTimeTracker::FlushQueue ()
{
	for (int i = 0; i < m_outQueueBufferindex; i ++) {
		const dTrackEntry& event = m_outQueueBuffer[i];
		if (!m_firstRecord) {
			fprintf (m_file, "\n");
			m_firstRecord = true;
		} else {
			fprintf (m_file, ",\n");
		}
		
		dTree<dLabels, dCRCTYPE>::dTreeNode* threadNodeName = m_dictionary.Find (event.m_threadId);
		if (!threadNodeName) {
			threadNodeName = m_dictionary.Insert(event.m_threadId);
			sprintf (threadNodeName->GetInfo().m_name, "thread_%d", event.m_threadId);
		}

		fprintf (m_file, "\t\t {");
		fprintf (m_file, "\"name\": \"%s\"", m_dictionary.Find (event.m_nameCRC)->GetInfo().m_name);
		fprintf (m_file, ", \"cat\": \"%s\"", threadNodeName->GetInfo().m_name);
		fprintf (m_file, ", \"ph\": \"X\"");
		fprintf (m_file, ", \"pid\": \"%d\"", m_processId);
		fprintf (m_file, ", \"tid\": \"%d\"", event.m_threadId);
		fprintf (m_file, ", \"ts\": %lld", event.m_startTime);
		fprintf (m_file, ", \"dur\": %lld", event.m_endTime - event.m_startTime);
		fprintf (m_file, "}");
/*
		fprintf (m_file, "\t\t {");
		fprintf (m_file, "\"name\": \"%s\", ", m_dictionary.Find (event.m_nameCRC)->GetInfo().m_name);
		fprintf (m_file, "\"cat\": \"%s\", ", threadNodeName->GetInfo().m_name);
		fprintf (m_file, "\"ph\": \"E\", ");
		fprintf (m_file, "\"pid\": \"%d\", ", m_processId);
		fprintf (m_file, "\"tid\": \"%d\", ", event.m_threadId);
		fprintf (m_file, "\"ts\": %lld", event.m_endTime);
		fprintf (m_file, "}");
*/
	}

	m_outQueueBufferindex = 0;
}



dTimeTracker::dTrackEntry::dTrackEntry(dCRCTYPE nameCRC)
{
	dTimeTracker* const instance = dTimeTracker::GetInstance();
	if (instance->m_file) {
		long long startTime = instance->GetTimeInNanosenconds ();
		m_nameCRC = nameCRC;
		m_startTime = startTime;
		m_endTime = startTime;
		m_threadId = GetCurrentThreadId();
	}

}

dTimeTracker::dTrackEntry::~dTrackEntry()
{
	dTimeTracker* const instance = dTimeTracker::GetInstance();
	m_endTime = instance->GetTimeInNanosenconds ();
	if (instance->m_file) {
		EnterCriticalSection(&instance->m_criticalSection); 
		if (instance->m_bufferIndex >= instance->m_bufferSize) {
			dTrackEntry* const buffer = ((dTrackEntry*) dContainersAlloc::Alloc (2 * instance->m_bufferSize * sizeof (dTrackEntry)));
			memcpy (buffer, instance->m_buffer, instance->m_bufferIndex * sizeof (dTrackEntry));
			dContainersAlloc::Free (instance->m_buffer);
			instance->m_bufferSize = instance->m_bufferSize * 2;
			instance->m_buffer = buffer;
		}
		instance->m_buffer[instance->m_bufferIndex] = *this;
		instance->m_bufferIndex ++;
		LeaveCriticalSection(&instance->m_criticalSection);
	}
}


#endif