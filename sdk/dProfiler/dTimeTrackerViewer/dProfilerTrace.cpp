#include "stdafx.h"
#include "dProfilerTrace.h"
#include <zlib.h>

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

class dThreadTrace
{
	public:
	dThreadTrace ()
		:m_count(0)
		,m_maxCount(1<<DG_TIME_TRACKER_ENTRIES_POWER)
		,m_buffer (new dTimeTrackerRecord[m_maxCount])
	{
	}

	~dThreadTrace ()
	{
		delete[] m_buffer;
	}

	int GetCount() const
	{
		return m_count; 
	}

	void AddTrace(Bytef* const compressedData, int compressesDataSize)
	{
		dThreadTrace& me = *this;
		dTimeTrackerRecord* const buffer = &me[m_count];

		uLongf destLen;
		int compressError = uncompress((Bytef*)buffer, &destLen, compressedData, compressesDataSize);
		
		m_count += 1<<DG_TIME_TRACKER_ENTRIES_POWER;
	}

	dTimeTrackerRecord& operator[] (int i)
	{
		while (i >= m_maxCount) {
			dTimeTrackerRecord* const newbuffer = new dTimeTrackerRecord[m_maxCount * 2];
			for (int i = 0; i < m_maxCount; i++) {
				newbuffer[i] = m_buffer[i];
			}
			delete[] m_buffer;
			m_buffer = newbuffer;
			m_maxCount = m_maxCount * 2;
		}
		return m_buffer[i];
	}

	int m_count;
	int m_maxCount;
	dTimeTrackerRecord* m_buffer;
};

ref class dProfilerTrace::dDataBase
{
	public: dDataBase(System::IO::Stream^ file)
		:m_data(gcnew array<unsigned char>(int(file->Length)))
		,m_trace(new dTimeTrackerMap<dThreadTrace, unsigned>())
		,m_dictionary(new dTimeTrackerMap<dTrackerString, unsigned>())
		,m_index(0)
	{
		file->Read(m_data, 0, m_data->Length);
	}

	public: ~dDataBase()
	{
		delete m_trace;
		delete m_dictionary;
	}

	int ReadInt()
	{
		char code[4];
		code[0] = m_data[m_index++];
		code[1] = m_data[m_index++];
		code[2] = m_data[m_index++];
		code[3] = m_data[m_index++];
		return *(int*)&code[0];
	}

	dTrackerChunkType ReadChunkType()
	{
		return dTrackerChunkType(ReadInt());
	}

	int ReadName(char* const name)
	{
		unsigned size = ReadInt();
		for (unsigned i = 0; i < size; i++) {
			name[i] = m_data[m_index++];
		}
		name[size] = 0;
		return size;
	}

	void ReadCompressedTrack(char* const buffer, int size)
	{
		for (int i = 0; i < size; i++) {
			buffer[i] = m_data[m_index++];
		}
	}


	array<unsigned char>^ m_data;
	dTimeTrackerMap<dThreadTrace, unsigned>* m_trace;	
	dTimeTrackerMap<dTrackerString, unsigned>* m_dictionary;
	int m_index;
};


dProfilerTrace::dProfilerTrace(System::IO::Stream^ file)
	:m_rootNode (gcnew dTraceCapture())
	,m_nameList (gcnew array<System::String^>(0))
{
	dDataBase^ database = gcnew dDataBase(file); 
	for (dTrackerChunkType chunkType = database->ReadChunkType(); chunkType != m_traceEnd; chunkType = database->ReadChunkType()) {
		switch (chunkType) 
		{
			case m_traceSamples:
			{
				ReadTrack(database);
				break;
			}

			case m_traceLabel:
			{
				ReadLabels(database);
				break;
			}

			default:
				assert(0);
				break;
		}
	}

	int index = 0;
	dTimeTrackerMap<int, unsigned> nameMap;
	dTimeTrackerMap<dTrackerString, unsigned>::Iterator iter (*database->m_dictionary);
	array<System::String^>::Resize(m_nameList, database->m_dictionary->GetCount());
	for (iter.Begin(); iter; iter ++) {
		const dTrackerString& name = iter.GetNode()->GetInfo();
		m_nameList[index] = gcnew System::String(name.m_string);
		nameMap.Insert(index, iter.GetKey());
		index ++;
	}

	dTimeTrackerMap<dThreadTrace, unsigned>::Iterator traceIter (*database->m_trace);
	for (traceIter.Begin(); traceIter; traceIter ++) {
		dThreadTrace& track = traceIter.GetNode()->GetInfo();
		for (int i = 0; i < track.m_count; i ++) {
			track[i].m_nameHash = nameMap.Find(track[i].m_nameHash)->GetInfo();
		}

		for (int i = 0; i < track.m_count; i++) {
//			track[i].m_nameHash = nameMap.Find(track[i].m_nameHash)->GetInfo();
		}

	}

}

dProfilerTrace::~dProfilerTrace()
{
}



void dProfilerTrace::ReadLabels(dDataBase^ database)
{
	for (dTrackerChunkType chunkType = database->ReadChunkType(); chunkType == m_traceLabel; chunkType = database->ReadChunkType()) {
		char name[1024];
		unsigned key = database->ReadInt();
		unsigned size = database->ReadName(name);
		database->m_dictionary->Insert(name, key);
	}
}

void dProfilerTrace::ReadTrack(dDataBase^ database)
{
	unsigned nameCRC = database->ReadInt();
	unsigned compressesDataSize = database->ReadInt();
	char* const compressedData = new char[compressesDataSize + 1024];
	database->ReadCompressedTrack(compressedData, compressesDataSize);

	dTimeTrackerMap<dThreadTrace, unsigned>::dTreeNode* threadNode = database->m_trace->Find(nameCRC);
	if (!threadNode) {
		threadNode = database->m_trace->Insert(nameCRC);
	}
	dThreadTrace& track = threadNode->GetInfo();
	track.AddTrace((Bytef*)compressedData, compressesDataSize);
	delete[] compressedData;
}
