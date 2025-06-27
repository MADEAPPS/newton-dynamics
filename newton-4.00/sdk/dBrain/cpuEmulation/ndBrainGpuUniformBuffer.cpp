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
#include "ndBrainGpuContext.h"
#include "ndBrainGpuUniformBuffer.h"

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes)
	:ndBrainBuffer(context, sizeInBytes)
{
}

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainBuffer(context, sizeInBytes)
{
	LoadData(0, size_t(sizeInBytes), data);
}

void* ndBrainGpuUniformBuffer::GetData()
{
	return &m_data[0];
}

//void ndBrainGpuUniformBuffer::BrainVectorToDevice(const ndBrainVector& vector)
void ndBrainGpuUniformBuffer::BrainVectorToDevice(const ndBrainVector&)
{
	ndAssert(0);
}

//void ndBrainGpuUniformBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainGpuUniformBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainGpuUniformBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory)
{
	LoadData(offsetInBytes, sizeInBytes, inputMemory);
}

void ndBrainGpuUniformBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

//void ndBrainGpuUniformBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
void ndBrainGpuUniformBuffer::CopyBuffer(const ndBrainBuffer&, ndInt32, const ndBrainBuffer&)
{
	ndAssert(0);
}

//void ndBrainGpuUniformBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
void ndBrainGpuUniformBuffer::CopyBufferIndirect(const ndBrainBuffer&, const ndBrainBuffer&, const ndBrainBuffer&)
{
	ndAssert(0);
}

void ndBrainGpuUniformBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	const char* const ptr = &((char*)inputData)[offsetInBytes];
	m_data.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32(sizeInBytes); ++i)
	{
		m_data.PushBack(ptr[i]);
	}
}

void ndBrainGpuUniformBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
{
	char* const ptr = &((char*)outputData)[offsetInBytes];
	ndInt32 bytes = ndMin(ndInt32(sizeInBytes), ndInt32(m_data.GetCount()));
	for (ndInt32 i = 0; i < bytes; ++i)
	{
		ptr[i] = m_data[i];
	}
}
