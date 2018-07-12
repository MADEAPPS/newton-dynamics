#pragma once

#include "dTimeTrackerMap.h"
#include "dTimeTrackerRecord.h"

class dThreadTrace;
class dTimeTrackerViewer;

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

	const T& operator[] (int i) const
	{
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

	protected:
	T* m_data;
	int m_count;
	int m_maxCount;
};

class dProfilerTrace
{
	class dDataBase;

	class dTrackerString
	{
		public:
		dTrackerString()
		{
			m_string[0] = 0;
		}

		dTrackerString(const char* const string)
		{
			strcpy(m_string, string);
		}

		dTrackerString(const dTrackerString& src)
		{
			strcpy(m_string, src.m_string);
		}

		char m_string[128];
	};

	class dTrackerSample
	{
		public: 
		dTrackerSample(unsigned name, unsigned start, unsigned duration)
			:m_name(name)
			,m_start(start)
			,m_duration(duration)
			,m_children (dArray<dTrackerSample*>())
		{
		}

		~dTrackerSample()
		{
			for (int i = 0; i < m_children.GetSize(); i++) {
				delete m_children[i];
			}
		}

		void Render(dTimeTrackerViewer* const viewer, const dArray<dTrackerString>& nameList);

		unsigned m_name;
		unsigned m_start;
		unsigned m_duration;
		dArray<dTrackerSample*> m_children; 
	};

	class dTrackerThread
	{
		public: 
		dTrackerThread(unsigned threadName, dThreadTrace& track, const dArray<dTrackerString>& xxxxx);

		~dTrackerThread()
		{
			for (int i = 0; i < m_frames.GetSize(); i++) {
				delete m_frames[i];
			}
		}

		void Render(dTimeTrackerViewer* const viewer, const dArray<dTrackerString>& nameList);

		dArray<dTrackerSample*> m_frames; 
		int m_name;
		bool m_isOpen;
	};

	class dTraceCapture
	{
		public: dTraceCapture ()
			:m_treads (dArray<dTrackerThread*>())
		{
		}

		~dTraceCapture()
		{
			for (int i = 0; i < m_treads.GetSize(); i++) {
				delete m_treads[i];
			}
		}

		void Render(dTimeTrackerViewer* const viewer, const dArray<dTrackerString>& nameList);

		dArray<dTrackerThread*> m_treads;
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

