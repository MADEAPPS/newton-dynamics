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

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes)
	:ndBrainBuffer(context, sizeInBytes, ndUniformData)
{
}

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainBuffer(context, sizeInBytes, ndUniformData)
{
	ndAssert(0);
	LoadData(size_t(sizeInBytes), data);
}

#if 0
ndBrainGpuUniformBuffer* ndBrainGpuUniformBuffer::GetAsUniformBuffer()
{
	return this;
}

void ndBrainGpuUniformBuffer::UnloadData(size_t, void* const) const
{
}

void ndBrainGpuUniformBuffer::LoadData(size_t sizeIntBytes, const void* const sourceData)
{
	//const UniformBufferObject* const xxx = (UniformBufferObject*)sourceData;
	m_data.SetCount(0);
	const char* const src = (const char*) sourceData;
	for (ndInt32 i = 0; i < ndInt32 (sizeIntBytes); ++i)
	{
		m_data.PushBack(src[i]);
	}
	//const UniformBufferObject* const xxx1 = (UniformBufferObject*)&m_data[0];
	//const UniformBufferObject* const xxx0 = (UniformBufferObject*)sourceData;
}
#endif

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

void ndBrainGpuUniformBuffer::MemoryToDevive(size_t sizeInBytes, const void* const inputMemory)
{
	const char* const ptr = (char*)inputMemory;
	m_data.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32 (sizeInBytes); ++i)
	{
		m_data.PushBack(ptr[i]);
	}
}

void ndBrainGpuUniformBuffer::MemoryFromDevive(size_t sizeInBytes, void* const outputMemory) const
{
	char* const ptr = (char*)outputMemory;
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

//void ndBrainGpuUniformBuffer::CopyBufferIndirectSource(const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcDataBuffer, ndInt32 srcStrideIntBytes)
void ndBrainGpuUniformBuffer::CopyBufferIndirectSource(const ndBrainBuffer&, const ndBrainBuffer&, ndInt32)
{
	ndAssert(0);
}

//void ndBrainGpuUniformBuffer::LoadData(size_t sizeInBytes, const void* const inputData)
void ndBrainGpuUniformBuffer::LoadData(size_t, const void* const)
{
	ndAssert(0);
}

//void ndBrainGpuUniformBuffer::UnloadData(size_t sizeInBytes, void* const outputData) const
void ndBrainGpuUniformBuffer::UnloadData(size_t, void* const) const
{
	ndAssert(0);
}
