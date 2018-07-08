#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"


ref class dProfilerTrace
{
	ref class dDataBase;
	public: dProfilerTrace(System::IO::Stream^ file);
	public: ~dProfilerTrace();

	private: void ReadTrack(dDataBase^ database);
	private: void ReadLabels(dDataBase^ database);

	dTimeTrackerMap<dTrackeString, unsigned>* m_dictionary;
};

