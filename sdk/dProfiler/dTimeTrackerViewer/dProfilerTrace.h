#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"

class dThreadTrace;

ref class dProfilerTrace
{
	ref class dDataBase;

	template<class T>
	ref class dArray
	{
		public: dArray()
			:m_data (gcnew array<T>(1)) 
			,m_count(0)
		{
		}

		T operator[] (int i)
		{
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

		array<T>^ m_data;
		int m_count;
	};

	ref class dTrace
	{
		public: dTrace(unsigned name, unsigned start, unsigned duration)
			:m_name(name)
			,m_start(start)
			,m_duration(duration)
			,m_children (gcnew dArray<dTrace^>)
		{
		}

		unsigned m_name;
		unsigned m_start;
		unsigned m_duration;
		dArray<dTrace^>^ m_children; 
	};

	ref class dThread
	{
		public: dThread(unsigned threadName, dThreadTrace& track, array<System::String^>^ xxxxx);

		dArray<dTrace^>^ m_frames; 
		int m_name;
	};

	ref class dTraceCapture
	{
		public: dTraceCapture ()
			:m_treads (gcnew dArray<dThread^>)
		{
		}
		dArray<dThread^>^ m_treads;
	};

	public: dProfilerTrace(System::IO::Stream^ file);
	public: ~dProfilerTrace();

	private: void ReadTrack(dDataBase^ database);
	private: void ReadLabels(dDataBase^ database);

	dTraceCapture^ m_rootNode;
	array<System::String^>^ m_nameList;
};

