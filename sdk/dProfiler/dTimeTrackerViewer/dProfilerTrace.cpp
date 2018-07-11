#include "stdafx.h"
#include "dProfilerTrace.h"


/*
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
*/

class dThreadTrace: public dArray<dTimeTrackerRecord>
{
	public:
	dThreadTrace ()
		:dArray<dTimeTrackerRecord>()
//		:m_count(0)
//		,m_maxCount(1<<DG_TIME_TRACKER_ENTRIES_POWER)
//		,m_buffer (new dTimeTrackerRecord[m_maxCount])
	{
		static int xxxx;
		xxxxx = xxxx;
		xxxx ++;
	}

	~dThreadTrace ()
	{
	}
/*
	int GetCount() const
	{
		return m_count; 
	}
*/
	void AddTrace(Bytef* const compressedData, int compressesDataSize)
	{
		dThreadTrace& me = *this;

		me[me.GetSize() + (1<<DG_TIME_TRACKER_ENTRIES_POWER) - 1].m_start = 0;
		dTimeTrackerRecord* const buffer = &me[me.GetSize()];

		uLongf destLen;
		int compressError = uncompress((Bytef*)buffer, &destLen, compressedData, compressesDataSize);
		dAssert (compressError == Z_OK);
	}

/*
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
*/
	int xxxxx;
//	int m_count;
//	int m_maxCount;
//	dTimeTrackerRecord* m_buffer;
//	dArray<dTimeTrackerRecord> m_buffer;
};

class dProfilerTrace::dDataBase
{
	public: 
	dDataBase(FILE* const file)
		:m_file(file)
		,m_trace()
		,m_dictionary()
//		,m_index(0)
	{
	}

	public: ~dDataBase()
	{
	}

	int ReadInt()
	{
		int code;
		fread (&code, sizeof (int), 1, m_file);
		return code;
	}

	dTrackerChunkType ReadChunkType()
	{
		return dTrackerChunkType(ReadInt());
	}

	int ReadName(char* const name)
	{
		dAssert (0);
		return 0;
/*
		unsigned size = ReadInt();
		for (unsigned i = 0; i < size; i++) {
			name[i] = m_data[m_index++];
		}
		name[size] = 0;
		return size;
*/
	}

	void ReadCompressedTrack(char* const buffer, int size)
	{
		fread (buffer, size, 1, m_file);
	}

	FILE* m_file;
	dTimeTrackerMap<dThreadTrace, unsigned> m_trace;	
	dTimeTrackerMap<dString, unsigned> m_dictionary;
//	int m_index;
};


dProfilerTrace::dProfilerTrace(FILE* const file)
//	:m_rootNode (gcnew dTraceCapture())
//	,m_nameList (gcnew array<System::String^>(0))
{
	dDataBase database(file); 

	for (dTrackerChunkType chunkType = database.ReadChunkType(); chunkType != m_traceEnd; chunkType = database.ReadChunkType()) {

		switch (chunkType) 
		{
			case m_traceSamples:
			{
				ReadTrack(database);
				break;
			}

			case m_traceLabel:
			{
				dAssert(0);
				//ReadLabels(database);
				break;
			}

			default:
				dAssert(0);
				break;
		}
	}
	dAssert(0);
/*
	int index = 0;
	dTimeTrackerMap<int, unsigned> nameMap;
	dTimeTrackerMap<dTrackerString, unsigned>::Iterator iter (*database->m_dictionary);
	array<System::String^>::Resize(m_nameList, database->m_dictionary->GetCount());
	for (iter.Begin(); iter; iter ++) {
		const dTrackerString& name = iter.GetNode()->GetInfo();
		m_nameList[index] = gcnew System::String(name.m_string);
		unsigned key = iter.GetKey();
		nameMap.Insert(index, key);
		index ++;
	}

	index = 0;
	dTimeTrackerMap<dThreadTrace, unsigned>::Iterator traceIter (*database->m_trace);
	for (traceIter.Begin(); traceIter; traceIter ++) {
		dThreadTrace& track = traceIter.GetNode()->GetInfo();
		for (int i = 0; i < track.m_count; i ++) {
			track[i].m_nameHash = nameMap.Find(track[i].m_nameHash)->GetInfo();
		}
		m_rootNode->m_treads->Push(gcnew dThread(nameMap.Find(traceIter.GetKey())->GetInfo(), track, m_nameList));
		index ++;
	}
*/
}

dProfilerTrace::~dProfilerTrace()
{
}


/*
void dProfilerTrace::ReadLabels(dDataBase^ database)
{
	for (dTrackerChunkType chunkType = database->ReadChunkType(); chunkType == m_traceLabel; chunkType = database->ReadChunkType()) {
		char name[1024];
		unsigned key = database->ReadInt();
		unsigned size = database->ReadName(name);
		database->m_dictionary->Insert(name, key);
	}
}



dProfilerTrace::dThread::dThread(unsigned threadName, dThreadTrace& track, array<System::String^>^ xxxx)
	:m_frames(gcnew dArray<dTrace^>)
	,m_name(threadName)
{
System::String^ xxxxxxxxxxx = xxxx[threadName];

	int index = 0;
	const int maxSize = track.m_count;
	do {
		const dTimeTrackerRecord& record = track[index];
		dTrace^ trace = gcnew dTrace(record.m_nameHash, record.m_start, record.m_duration);

		bool isRootTrace = true;
		const int framesCount = m_frames->GetSize(); 

System::String^ xxxxxxxxx= xxxx[record.m_nameHash];
		if (framesCount) {
			int x0 = record.m_start;
			//int x1 = x0 + record.m_duration;
			int y0 = m_frames[framesCount - 1]->m_start;
			int y1 = y0 + m_frames[framesCount - 1]->m_duration;
			if (x0 >= y1) {
				m_frames->Push(trace);
			} else {
				index*=1;
				//assert (0);
			}
		} else {
			m_frames->Push(trace);
		}
		index ++;
	} while (index < maxSize);

}
*/


void dProfilerTrace::ReadTrack(dDataBase& database)
{
	unsigned nameCRC = database.ReadInt();
	unsigned compressesDataSize = database.ReadInt();

	char* compressedData = dAlloca (char, compressesDataSize + 1024);
	database.ReadCompressedTrack(compressedData, compressesDataSize);

	dTimeTrackerMap<dThreadTrace, unsigned>::dTreeNode* threadNode = database.m_trace.Find(nameCRC);
	if (!threadNode) {
		threadNode = database.m_trace.Insert(nameCRC);
	}
	dThreadTrace& track = threadNode->GetInfo();
	track.AddTrace((Bytef*)compressedData, compressesDataSize);
}
