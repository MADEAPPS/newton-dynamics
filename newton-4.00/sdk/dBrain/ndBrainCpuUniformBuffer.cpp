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
#include "ndBrainCpuContext.h"
#include "ndBrainCpuUniformBuffer.h"

ndBrainCpuUniformBuffer::ndBrainCpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes)
	:ndBrainBuffer(context, sizeInBytes, ndUniformData)
{
}

ndBrainCpuUniformBuffer::ndBrainCpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainBuffer(context, sizeInBytes, ndUniformData)
{
	LoadData(0, size_t(sizeInBytes), data);
}

void ndBrainCpuUniformBuffer::BrainVectorToDevice(const ndBrainVector&)
{
	ndAssert(0);
	//ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	//LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainCpuUniformBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainCpuUniformBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainCpuUniformBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	LoadData(offsetInBytes, sizeInBytes, inputData);
}

void ndBrainCpuUniformBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainCpuUniformBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	m_data.SetCount(ndInt32 (size));
	ndInt32 offset = ndInt32(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy(&m_data[offset], (ndUnsigned32*)sourceData, ndInt64(size));
}

void ndBrainCpuUniformBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
{
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	ndInt32 offset = ndInt32(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy((ndUnsigned32*)outputData, &m_data[offset], ndInt64(size));
}

//void ndBrainCpuUniformBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
void ndBrainCpuUniformBuffer::CopyBuffer(const ndBrainBuffer&, ndInt32, const ndBrainBuffer&)
{
	ndAssert(0);
	//ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	//ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));
	//
	//ndBrainCpuUniformBuffer& source = *((ndBrainCpuUniformBuffer*)&sourceData);
	//ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);
}

void ndBrainCpuUniformBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
{
	ndAssert(0);
}