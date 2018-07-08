#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"



ref class dProfilerTrace
{
	ref class dDataBase;

	ref class dTrace
	{
		public: dTrace()
			:m_children (gcnew array<dTrace^>(0))
		{
		}

		unsigned m_name;
		unsigned m_start;
		unsigned m_duration;
		
		array<dTrace^>^ m_children; 
	};

	ref class dThread
	{
		public: dThread(unsigned threadName)
			:m_frames (gcnew array<dTrace^>(0))
		{
		}

		array<dTrace^>^ m_frames; 
	};

	ref class dTraceCapture
	{
		public: dTraceCapture ()
			:m_treads (gcnew array<dThread^>(0))
		{
		}

		array<dThread^>^ m_treads;
	};

	public: dProfilerTrace(System::IO::Stream^ file);
	public: ~dProfilerTrace();

	private: void ReadTrack(dDataBase^ database);
	private: void ReadLabels(dDataBase^ database);

	dTraceCapture^ m_rootNode;
	array<System::String^>^ m_nameList;
};

