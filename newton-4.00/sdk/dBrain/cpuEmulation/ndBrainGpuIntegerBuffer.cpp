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
#include "ndBrainGpuContext.h"
#include "ndBrainGpuIntegerBuffer.h"

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainContext* const context, ndInt64 sizeInElements)
	:ndBrainBuffer(context, sizeInElements * ndInt64(sizeof(ndUnsigned32)), ndStorageData)
{
	m_indexArray.SetCount(sizeInElements);
}

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainContext* const context, ndInt64 numberOfElements, const ndUnsigned32* const indexArray)
	:ndBrainBuffer(context, numberOfElements * ndInt64(sizeof(ndUnsigned32)), ndStorageData)
{
	ndAssert(m_context->GetAsGpuContext());
	m_indexArray.SetCount(numberOfElements);
	MemoryToDevive(0, m_sizeInBytes, indexArray);
}

void ndBrainGpuIntegerBuffer::BrainVectorToDevice(const ndBrainVector&)
{
	ndAssert(0);
	//ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	//LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainGpuIntegerBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainGpuIntegerBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainGpuIntegerBuffer::BrainMatrixToDevice(const ndBrainMatrix* const matrix)
{
	ndBrainVector flatArray;
	for (ndInt32 i = 0; i < matrix->GetRows(); i++)
	{
		for (ndInt32 j = 0; j < matrix->GetColumns(); j++)
		{
			flatArray.PushBack((*matrix)[i][j]);
		}
	}
	BrainVectorToDevice(flatArray);
}

void ndBrainGpuIntegerBuffer::MemoryToDevive(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	//ndAssert(sizeInBytes <= m_sizeInBytes);
	//size_t offset = offsetInBytes / sizeof(ndUnsigned32);
	//size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	//ndMemCpy(&m_indexArray[ndInt64(offset)], (ndUnsigned32*)inputData, ndInt64(size));
	LoadData(offsetInBytes, sizeInBytes, inputData);
}

void ndBrainGpuIntegerBuffer::MemoryFromDevive(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainGpuIntegerBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	size_t offset = offsetInBytes / sizeof(ndUnsigned32);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	ndMemCpy(&m_indexArray[ndInt64(offset)], (ndUnsigned32*)sourceData, ndInt64(size));
}

void ndBrainGpuIntegerBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
{
	size_t offset = offsetInBytes / sizeof(ndUnsigned32);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	ndMemCpy((ndUnsigned32*)outputData, &m_indexArray[ndInt64(offset)], ndInt64(size));
}

void ndBrainGpuIntegerBuffer::CopyBuffer(const ndBrainBuffer& sourceData, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
{
	ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	
	ndBrainGpuIntegerBuffer& source = *((ndBrainGpuIntegerBuffer*)&sourceData);
	//ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);
	ndMemCpy(&source.m_indexArray[dstOffset], &m_indexArray[srcOffset], ndInt64(size));
}

//void ndBrainGpuIntegerBuffer::CopyBufferIndirectSource(const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcDataBuffer, ndInt32 srcStrideInBytes)
void ndBrainGpuIntegerBuffer::CopyBufferIndirectSource(const ndBrainBuffer&, const ndBrainBuffer&, ndInt32)
{
	ndAssert(0);
}