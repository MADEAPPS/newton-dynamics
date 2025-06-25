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
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuIntegerBuffer.h"

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, ndInt64 size)
	:ndBrainBuffer(context, size * ndInt32(sizeof(ndReal)), ndStorageData)
	,m_buffer()
{
	m_buffer.SetCount(size);
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsGpuContext());
	m_buffer.SetCount(input.GetCount());
	BrainVectorToDevice(input);
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainBuffer(context, matrix.GetColumns()* matrix.GetRows()* ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsGpuContext());
	m_buffer.SetCount(matrix.GetColumns() * matrix.GetRows());
	BrainMatrixToDevice(&matrix);
}

#if 0
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

#endif

ndBrainFloat* ndBrainGpuFloatBuffer::GetData()
{
	return &m_buffer[0];
}

void ndBrainGpuFloatBuffer::BrainVectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	LoadData(0, vector.GetCount() * sizeof(ndReal), &vector[0]);
}

void ndBrainGpuFloatBuffer::BrainVectorFromDevice(ndBrainVector& ouput) const
{
	ouput.SetCount(m_buffer.GetCount());
	UnloadData(0, ouput.GetCount() * sizeof(ndReal), &ouput[0]);
}

void ndBrainGpuFloatBuffer::BrainMatrixToDevice(const ndBrainMatrix* const matrix)
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

void ndBrainGpuFloatBuffer::MemoryToDevive(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory)
{
	//ndAssert(sizeInBytes <= m_sizeInBytes);
	//size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndReal);
	//size_t offset = offsetInBytes / sizeof(ndReal);
	//ndMemCpy(&m_buffer[ndInt64(offset)], (ndBrainFloat*)inputMemory, ndInt64(size));
	LoadData(offsetInBytes, sizeInBytes, inputMemory);
}

void ndBrainGpuFloatBuffer::MemoryFromDevive(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	//ndAssert(sizeInBytes <= m_sizeInBytes);
	//size_t offset = offsetInBytes / sizeof(ndReal);
	//size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndReal);
	//ndMemCpy((ndBrainFloat*)outputMemory, &m_buffer[ndInt64(offset)], ndInt64(size));
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainGpuFloatBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	size_t offset = offsetInBytes / sizeof(ndReal);
	ndBrainMemVector dst(&m_buffer[ndInt64(offset)], ndInt32(sizeInBytes / sizeof(ndReal)));
	const ndBrainMemVector src((ndBrainFloat*)sourceData, ndInt32(sizeInBytes / sizeof(ndReal)));
	dst.Set(src);
}

void ndBrainGpuFloatBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
{
	size_t offset = offsetInBytes / sizeof(ndReal);
	ndBrainMemVector dst((ndBrainFloat*)outputData, ndInt32(sizeInBytes / sizeof(ndReal)));
	const ndBrainMemVector src(&m_buffer[ndInt64(offset)], ndInt32(sizeInBytes / sizeof(ndReal)));
	dst.Set(src);
}

void ndBrainGpuFloatBuffer::CopyBuffer(const ndBrainBuffer& srcBuffer, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
{
	ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));

	ndBrainGpuFloatBuffer& source = *((ndBrainGpuFloatBuffer*)&srcBuffer);
	ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	dst.Set(src);
}

void ndBrainGpuFloatBuffer::CopyBufferIndirectSource(const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcDataBuffer, size_t srcstrideInBytes)
{
	const ndBrainGpuFloatBuffer& srcBuffer = *((ndBrainGpuFloatBuffer*)&srcDataBuffer);
	const ndBrainGpuIntegerBuffer& indirectArray = *((ndBrainGpuIntegerBuffer*)&indexBuffer);
	ndInt32 count = ndInt32(indirectArray.SizeInBytes() / sizeof(ndUnsigned32));

	ndInt32 stride = ndInt32 (srcstrideInBytes / sizeof(ndReal));
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndUnsigned32 index = indirectArray.m_indexArray[i];
		ndBrainMemVector src(&srcBuffer.m_buffer[index * stride], stride);
		ndBrainMemVector dst(&m_buffer[i * stride], stride);
		dst.Set(src);
	}
}