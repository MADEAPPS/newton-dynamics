#include "stdafx.h"
#include "dProfilerTrace.h"



dProfilerTrace::dProfilerTrace(System::IO::Stream^ file)
{
	dDataBase^ database = gcnew dDataBase(file); 

	while (1) 
	{
		dTrackerChunkType chunkType (database->ReadChunkType());
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


void dProfilerTrace::ReadTrack(dDataBase^ database)
{
	unsigned nameCRC = database->ReadInt();
	unsigned compressesDataSize = database->ReadInt();
	array<unsigned char>^ compressedData = database->ReadCompressedTrack(compressesDataSize);
}

void dProfilerTrace::ReadLabels(dDataBase^ database)
{
}