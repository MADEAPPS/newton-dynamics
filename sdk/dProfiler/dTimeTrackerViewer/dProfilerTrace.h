#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"

class dThreadTrace;

class dProfilerTrace
{
	class dDataBase;

	template<class T>
	class dArray
	{
		public: 
		dArray()
			:m_data (new T[1]) 
			,m_count(0)
			,m_maxCount(1)
		{
		}

		T operator[] (int i)
		{
			assert (0);
			return m_data[i];
		}

		int GetSize()
		{
			return m_count;
		}

		void Push(T val)
		{
			while (m_count >= m_data->Length) {
				array<T>::Resize(m_data, m_data->Length * 2);
			}
			m_data[m_count] = val;
			m_count ++;
		}

		T* m_data;
		int m_count;
		int m_maxCount;
	};

	class dSample
	{
		public: 

		dSample(unsigned name, unsigned start, unsigned duration)
			:m_name(name)
			,m_start(start)
			,m_duration(duration)
			,m_children (dArray<dSample*>())
		{
		}

		unsigned m_name;
		unsigned m_start;
		unsigned m_duration;
		dArray<dSample*> m_children; 
	};

	class dThread
	{
		public: 
		dThread(unsigned threadName, dThreadTrace& track, dArray<dString>& xxxxx);

		dArray<dSample*> m_frames; 
		int m_name;
	};

	class dTraceCapture
	{
		public: dTraceCapture ()
			:m_treads (dArray<dThread*>())
		{
		}
		dArray<dThread*> m_treads;
	};

	public: dProfilerTrace(FILE* const file);
	public: ~dProfilerTrace();

	private: void ReadTrack(dDataBase database);
	private: void ReadLabels(dDataBase database);

	dTraceCapture m_rootNode;
	dArray<dString> m_nameList;
};

