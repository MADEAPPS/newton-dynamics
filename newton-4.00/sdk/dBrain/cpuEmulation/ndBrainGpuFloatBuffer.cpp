/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainGpuFloatBuffer.h"

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt64 size, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, size * ndInt32(sizeof(ndReal)), ndStorageData, deviceType)
	,m_buffer()
{
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData, deviceType)
{
	LoadData(input.GetCount() * sizeof (ndBrainFloat),  & input[0]);
}

ndBrainGpuFloatBuffer* ndBrainGpuFloatBuffer::GetAsFloatBuffer()
{
	return this;
}

void ndBrainGpuFloatBuffer::LoadData(size_t sizeInByte, const void* const sourceData)
{
	ndInt64 size = ndInt64(sizeInByte / sizeof(ndBrainFloat));
	m_buffer.SetCount(size);
	const ndBrainMemVector src((ndBrainFloat*)sourceData, size);
	m_buffer.Set(src);
}

void ndBrainGpuFloatBuffer::UnloadData(size_t sizeInByte, void* const dstData) const
{
	ndInt64 size = ndInt64(sizeInByte / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)dstData, size);
	dst.Set(m_buffer);
}
