#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"

class dThreadTrace;

template<class T>
class dArray
{
	public:
	dArray()
		:m_data(new T[1])
		, m_count(0)
		, m_maxCount(1)
	{
	}

	T& operator[] (int i)
	{
		Resize(i);
		return m_data[i];
	}

	int GetSize()
	{
		return m_count;
	}

	void Push(T val)
	{
		Resize(m_count + 1);
		m_data[m_count] = val;
		m_count++;
	}

	private:
	void Resize(int index)
	{
		int currentMaxCount = m_maxCount;
		while (index >= m_maxCount) {
			T* const data = new T[m_maxCount * 2];
			memcpy (data, m_data, currentMaxCount * sizeof (T));
			delete[] m_data;
			m_data = data;
			m_maxCount = m_maxCount * 2;
		}
	}

	T* m_data;
	int m_count;
	int m_maxCount;
};


class dProfilerTrace
{
	class dDataBase;
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

	public: 
	dProfilerTrace(FILE* const file);
	~dProfilerTrace();

	private: 
	void ReadTrack(dDataBase& database);
	void ReadLabels(dDataBase& database);

	dTraceCapture m_rootNode;
	dArray<dString> m_nameList;
};

