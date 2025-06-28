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
#include "ndBrainUniformBuffer.h"

ndBrainUniformBuffer::ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes)
	:ndBrainBuffer(context, sizeInBytes)
{
}

ndBrainUniformBuffer::ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainBuffer(context, sizeInBytes)
{
	LoadData(0, size_t(sizeInBytes), data);
}

void ndBrainUniformBuffer::BrainVectorToDevice(const ndBrainVector&)
{
	ndAssert(0);
	//ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	//LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainUniformBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainUniformBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainUniformBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	LoadData(offsetInBytes, sizeInBytes, inputData);
}

void ndBrainUniformBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainUniformBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	m_data.SetCount(ndInt32 (size));
	ndInt32 offset = ndInt32(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy(&m_data[offset], (ndUnsigned32*)sourceData, ndInt64(size));
}

void ndBrainUniformBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
{
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	ndInt32 offset = ndInt32(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy((ndUnsigned32*)outputData, &m_data[offset], ndInt64(size));
}

//void ndBrainUniformBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
void ndBrainUniformBuffer::CopyBuffer(const ndBrainBuffer&, ndInt32, const ndBrainBuffer&)
{
	ndAssert(0);
	//ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	//ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));
	//
	//ndBrainUniformBuffer& source = *((ndBrainUniformBuffer*)&sourceData);
	//ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);
}

//void ndBrainUniformBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
void ndBrainUniformBuffer::CopyBufferIndirect(const ndBrainBuffer&, const ndBrainBuffer&, const ndBrainBuffer&)
{
	ndAssert(0);
}