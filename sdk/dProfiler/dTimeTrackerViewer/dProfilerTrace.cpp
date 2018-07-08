#include "stdafx.h"
#include "dProfilerTrace.h"



ref class dProfilerTrace::dDataBase
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

	int ReadName(char* const name)
	{
		unsigned size = ReadInt();
		for (unsigned i = 0; i < size; i++) {
			name[i] = m_data[m_index++];
		}
		name[size] = 0;
		return size;
	}

	array<unsigned char>^ ReadCompressedTrack(int size)
	{
		array<unsigned char>^ compressesData = gcnew array<unsigned char>(size);
		for (int i = 0; i < size; i++) {
			compressesData[i] = m_data[m_index++];
		}
		return compressesData;
	}

	array<unsigned char>^ m_data;
	int m_index;
};


dProfilerTrace::dProfilerTrace(System::IO::Stream^ file)
	:m_dictionary(nullptr)
{
	m_rootNode = gcnew dTraceCapture;

	dDataBase^ database = gcnew dDataBase(file); 
	for (dTrackerChunkType chunkType = database->ReadChunkType(); chunkType != m_traceEnd; chunkType = database->ReadChunkType()) {
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
				assert(0);
				break;
		}
	}
}

dProfilerTrace::~dProfilerTrace()
{
	if (m_dictionary) {
		delete m_dictionary;
	}
}


void dProfilerTrace::ReadTrack(dDataBase^ database)
{
	unsigned nameCRC = database->ReadInt();
	unsigned compressesDataSize = database->ReadInt();
	array<unsigned char>^ compressedData = database->ReadCompressedTrack(compressesDataSize);
}

void dProfilerTrace::ReadLabels(dDataBase^ database)
{
	m_dictionary = new dTimeTrackerMap<dTrackerString, unsigned>;
	for (dTrackerChunkType chunkType = database->ReadChunkType(); chunkType == m_traceLabel; chunkType = database->ReadChunkType()) {
		char name[1024];
		unsigned key = database->ReadInt();
		unsigned size = database->ReadName(name);
		m_dictionary->Insert(name, key);
	}
}