#pragma once

#include "dTimeTrackerRecord.h"

ref class dProfilerTrace
{
	ref class dDataBase
	{
		public: dDataBase(System::IO::Stream^ file)
			:m_data(gcnew array<unsigned char>(int(file->Length)))
			,m_index(0)
		{
			file->Read(m_data, 0, m_data->Length);
		}

		int ReadInt()
		{
			char code[4];
			code[0] = m_data[m_index++];
			code[1] = m_data[m_index++];
			code[2] = m_data[m_index++];
			code[3] = m_data[m_index++];
			return *(int*)&code[0];
		}

		dTrackerChunkType ReadChunkType()
		{
			return dTrackerChunkType(ReadInt());
		}

		array<unsigned char>^ ReadCompressedTrack(int size)
		{
			array<unsigned char>^ compressesData = gcnew array<unsigned char>(size);
			for (int i = 0; i < size; i ++) {
				compressesData[i] = m_data[m_index++];
			}
			return compressesData;
		}

		array<unsigned char>^ m_data;
		int m_index;
	};


	public: dProfilerTrace(System::IO::Stream^ file);
	private: void ReadTrack(dDataBase^ database);
	private: void ReadLabels(dDataBase^ database);
};

