#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"

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


ref class dProfilerTrace
{
	ref class dDataBase;
	ref class dTrackNode 
	{
		public: dTrackNode ()
			:m_children (gcnew array<dTrackNode^>(0))
		{
		}
		virtual ~dTrackNode()
		{
		}

		array<dTrackNode^>^ m_children;
	};

	ref class dThreadTrace: public dTrackNode 
	{
		public: dThreadTrace(unsigned threadName)
			:dTrackNode()
			,m_threadName(threadName)
			,m_name(gcnew array<unsigned>(0))
			,m_start(gcnew array<unsigned>(0))
			,m_duration(gcnew array<unsigned>(0))
		{
		}

		unsigned m_threadName;
		array<unsigned>^ m_name; 
		array<unsigned>^ m_start; 
		array<unsigned>^ m_duration; 
	};

	ref class dTraceCapture: public dTrackNode 
	{
		public: dTraceCapture ()
			:dTrackNode ()
		{
		}
	};

	public: dProfilerTrace(System::IO::Stream^ file);
	public: ~dProfilerTrace();

	private: void ReadTrack(dDataBase^ database);
	private: void ReadLabels(dDataBase^ database);

	dTraceCapture^ m_rootNode;
	dTimeTrackerMap<dTrackerString, unsigned>* m_dictionary;	
};

