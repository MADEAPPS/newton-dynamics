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
	:ndBrainBuffer(context, sizeInBytes, ndUniformData)
{
}

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainBuffer(context, sizeInBytes, ndUniformData)
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

//void ndBrainGpuUniformBuffer::BrainMatrixToDevice(const ndBrainMatrix* const matrix)
void ndBrainGpuUniformBuffer::BrainMatrixToDevice(const ndBrainMatrix* const)
{
	ndAssert(0);
}

void ndBrainGpuUniformBuffer::MemoryToDevive(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory)
{
	const char* const ptr = &((char*)inputMemory)[offsetInBytes];
	m_data.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32 (sizeInBytes); ++i)
	{
		m_data.PushBack(ptr[i]);
	}
}

void ndBrainGpuUniformBuffer::MemoryFromDevive(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	char* const ptr = &((char*)outputMemory)[offsetInBytes];
	ndInt32 bytes = ndMin(ndInt32 (sizeInBytes), ndInt32(m_data.GetCount()));
	for (ndInt32 i = 0; i < bytes; ++i)
	{
		ptr[i] = m_data[i];
	}
}

//void ndBrainGpuUniformBuffer::CopyBuffer(const ndBrainBuffer& srcBuffer, size_t sourceOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
void ndBrainGpuUniformBuffer::CopyBuffer(const ndBrainBuffer&, size_t, size_t, size_t)
{
	ndAssert(0);
}

//void ndBrainGpuUniformBuffer::CopyBufferIndirectSource(const ndBrainBuffer& indexBuffer, size_t dstOffsetInBytes, size_t dstStrideInBytes, const ndBrainBuffer& srcData, size_t srcOffsetInBytes, size_t srcStrideInBytes)
void ndBrainGpuUniformBuffer::CopyBufferIndirectSource(const ndBrainBuffer&, size_t, size_t, const ndBrainBuffer&, size_t, size_t)
{
	ndAssert(0);
}

//void ndBrainGpuUniformBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
void ndBrainGpuUniformBuffer::LoadData(size_t, size_t, const void* const)
{
	ndAssert(0);
}

//void ndBrainGpuUniformBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
void ndBrainGpuUniformBuffer::UnloadData(size_t, size_t, void* const) const
{
	ndAssert(0);
}
