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
#include "ndBrainMatrix.h"
#include "ndBrainCpuContext.h"
#include "ndBrainCpuIntegerBuffer.h"

ndBrainCpuIntegerBuffer::ndBrainCpuIntegerBuffer(ndBrainContext* const context, ndInt64 sizeInElements)
	:ndBrainBuffer(context, sizeInElements* ndInt64(sizeof(ndUnsigned32)), ndStorageData)
{
	m_indexArray.SetCount(sizeInElements);
}

ndBrainCpuIntegerBuffer::ndBrainCpuIntegerBuffer(ndBrainContext* const context, ndInt64 numberOfElements, const ndUnsigned32* const indexArray)
	:ndBrainBuffer(context, numberOfElements * ndInt64(sizeof(ndUnsigned32)), ndStorageData)
{
	ndAssert(m_context->GetAsCpuContext());
	m_indexArray.SetCount(numberOfElements);
	MemoryToDevive(0, m_sizeInBytes, indexArray);
}

void ndBrainCpuIntegerBuffer::BrainVectorToDevice(const ndBrainVector&)
{
	ndAssert(0);
	//ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	//LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainCpuIntegerBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainCpuIntegerBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainCpuIntegerBuffer::MemoryToDevive(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof (ndUnsigned32);
	ndInt64 offset = ndInt64(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy(&m_indexArray[offset], (ndUnsigned32*)inputData, ndInt64(size));
}

//void ndBrainCpuIntegerBuffer::MemoryFromDevive(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
void ndBrainCpuIntegerBuffer::MemoryFromDevive(size_t, size_t, void* const) const
{
	ndAssert(0);
}

//void ndBrainCpuIntegerBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
void ndBrainCpuIntegerBuffer::LoadData(size_t, size_t, const void* const)
{
	ndAssert(0);
	//ndBrainMemVector src((ndBrainFloat*)sourceData, ndInt32 (sizeInBytes / sizeof(ndReal)));
	//m_buffer.Set(src);
}

//void ndBrainCpuIntegerBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
void ndBrainCpuIntegerBuffer::UnloadData(size_t, size_t, void* const) const
{
	ndAssert(0);
}

//void ndBrainCpuIntegerBuffer::CopyBuffer(const ndBrainBuffer& sourceData, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
void ndBrainCpuIntegerBuffer::CopyBuffer(const ndBrainBuffer&, size_t, size_t, size_t)
{
	ndAssert(0);
	//ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	//ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));
	//
	//ndBrainCpuIntegerBuffer& source = *((ndBrainCpuIntegerBuffer*)&sourceData);
	//ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);
}

void ndBrainCpuIntegerBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
{
	ndAssert(0);
}