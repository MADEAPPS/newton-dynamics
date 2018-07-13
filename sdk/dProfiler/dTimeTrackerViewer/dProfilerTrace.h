#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"

class dTimeTrackerViewer;

class dProfilerTrace
{
	class dDataBase;
	class dThreadTrace;
	class dTrackerSample;
	class dTrackerThread;

	class dTrackerString
	{
		public:
		dTrackerString(){m_string[0] = 0;}
		dTrackerString(const char* const string){ strcpy(m_string, string);	}
		dTrackerString(const dTrackerString& src){strcpy(m_string, src.m_string);}

		char m_string[128];
	};

	class dTraceCapture
	{
		public: 
		dTraceCapture ();
		~dTraceCapture();
		void Render(dTimeTrackerViewer* const viewer);

		dArray<dTrackerThread*> m_treads;
		unsigned m_minTime;
		unsigned m_maxTime;
	};

	public: 
	dProfilerTrace(FILE* const file);
	~dProfilerTrace();

	void Render (dTimeTrackerViewer* const viewer);

	private: 
	void ReadTrack(dDataBase& database);
	void ReadLabels(dDataBase& database);

	dTraceCapture m_rootNode;
	dArray<dTrackerString> m_nameList;
};

