#include "stdafx.h"
#include "dProfilerTrace.h"


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
		m_count += 1 << DG_TIME_TRACKER_ENTRIES_POWER;
	}

	int xxxxx;
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
		unsigned size = ReadInt();
		fread(name, size, 1, m_file);
		name[size] = 0;
		return size;
	}

	void ReadCompressedTrack(char* const buffer, int size)
	{
		fread (buffer, size, 1, m_file);
	}

	FILE* m_file;
	dTimeTrackerMap<dThreadTrace, unsigned> m_trace;	
	dTimeTrackerMap<dTrackerString, unsigned> m_dictionary;
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
				ReadLabels(database);
				break;
			}

			default:
				dAssert(0);
				break;
		}
	}

	dTimeTrackerMap<int, unsigned> nameMap;
	dTimeTrackerMap<dTrackerString, unsigned>::Iterator iter (database.m_dictionary);
	for (iter.Begin(); iter; iter ++) {
		const dTrackerString& name = iter.GetNode()->GetInfo();
		unsigned key = iter.GetKey();
		nameMap.Insert(m_nameList.GetSize(), key);
		m_nameList.Push (name);
	}

	dTimeTrackerMap<dThreadTrace, unsigned>::Iterator traceIter (database.m_trace);
	for (traceIter.Begin(); traceIter; traceIter ++) {
		dThreadTrace& track = traceIter.GetNode()->GetInfo();
		const int threadsCount = track.GetSize();
		for (int i = 0; i < threadsCount; i ++) {
			int remapHashIndex = nameMap.Find(track[i].m_nameHash)->GetInfo();
			track[i].m_nameHash = remapHashIndex;
		}
		int treadHashIndex = nameMap.Find(traceIter.GetKey())->GetInfo();
		m_rootNode.m_treads.Push(new dThread(treadHashIndex, track, m_nameList));
	}
}

dProfilerTrace::~dProfilerTrace()
{
}

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


void dProfilerTrace::ReadLabels(dDataBase& database)
{
	for (dTrackerChunkType chunkType = database.ReadChunkType(); chunkType == m_traceLabel; chunkType = database.ReadChunkType()) {
		char name[1024];
		unsigned key = database.ReadInt();
		unsigned size = database.ReadName(name);
		database.m_dictionary.Insert(name, key);
	}
}


dProfilerTrace::dThread::dThread(unsigned threadName, dThreadTrace& track, const dArray<dTrackerString>& xxxxx)
	:m_frames()
	,m_name(threadName)
{
	const dTrackerString& xxxxxxxxxxx = xxxxx[threadName];

	int index = 0;
	const int maxSize = track.GetSize();
	do {
		const dTimeTrackerRecord& record = track[index];
		dSample* const trace = new dSample(record.m_nameHash, record.m_start, record.m_duration);

const dTrackerString& xxxxxxxxx = xxxxx[record.m_nameHash];

		bool isRootTrace = true;
		const int framesCount = m_frames.GetSize();

		if (framesCount) {
			int x0 = record.m_start;
			//int x1 = x0 + record.m_duration;
			int y0 = m_frames[framesCount - 1]->m_start;
			int y1 = y0 + m_frames[framesCount - 1]->m_duration;
			if (x0 >= y1) {
				m_frames.Push(trace);
			} else {
				index *= 1;
				//assert (0);
			}
		} else {
			m_frames.Push(trace);
		}

		index++;
	} while (index < maxSize);
}
