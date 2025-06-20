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
#include "ndBrainCpuFloatBuffer.h"
#include "ndBrainCpuIntegerBuffer.h"

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, ndInt64 sizeInFloat)
	:ndBrainBuffer(context, sizeInFloat * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(0);
	//m_buffer.SetCount(sizeInFloat);
	//m_buffer.Set(ndBrainFloat(0.0f));
}

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsCpuContext());
	m_buffer.SetCount(input.GetCount());
	BrainVectorToDevice(input);
}

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainBuffer(context, matrix.GetColumns() * matrix.GetRows() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsCpuContext());
	m_buffer.SetCount(matrix.GetColumns() * matrix.GetRows());
	BrainMatrixToDevice(&matrix);
}

////void ndBrainCpuFloatBuffer::LoadData(size_t sizeInBytes, const void* const sourceData)
//void ndBrainCpuFloatBuffer::LoadData(size_t, const void* const)
//{
//	ndAssert(0);
//}
//
////void ndBrainCpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const destinationData) const
//void ndBrainCpuFloatBuffer::UnloadData(size_t, void* const) const
//{
//	ndAssert(0);
//}
//
//void ndBrainCpuFloatBuffer::LoadBuffer(const ndBrainMatrix* const matrix)
//{
//	for (ndInt32 i = 0; i < matrix->GetRows(); ++i)
//	{
//		ndBrainMemVector dst (&m_buffer[i * matrix->GetColumns()], matrix->GetColumns());
//		dst.Set((*matrix)[i]);
//	}
//}
//
//void ndBrainCpuFloatBuffer::CopyBuffer(const ndBrainBuffer& source, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
//{
//	size_t size = sizeInBytes / sizeof(ndReal);
//	size_t srcOffset = srcOffsetInBytes / sizeof(ndReal);
//	size_t dstOffset = dstOffsetInBytes / sizeof(ndReal);
//	ndAssert(size <= size_t(m_buffer.GetCount()));
//	ndAssert((dstOffset + size) * sizeof(ndReal) <= SizeInBytes());
//	ndAssert((srcOffset + size) * sizeof (ndReal) <= source.SizeInBytes());
//
//	ndBrainMemVector dst(&m_buffer[ndInt64(dstOffset)], ndInt64(size));
//	const ndBrainCpuFloatBuffer& srcBuffer = *((ndBrainCpuFloatBuffer*)&source);
//	const ndBrainMemVector src(&srcBuffer.m_buffer[ndInt64(srcOffset)], ndInt64(size));
//	dst.Set(src);
//}


void ndBrainCpuFloatBuffer::BrainVectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainCpuFloatBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainCpuFloatBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainCpuFloatBuffer::BrainMatrixToDevice(const ndBrainMatrix* const matrix)
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

//void ndBrainCpuFloatBuffer::MemoryToDevive(size_t sizeInBytes, const void* const inputData)
void ndBrainCpuFloatBuffer::MemoryToDevive(size_t, const void* const)
{
	ndAssert(0);
}

//void ndBrainCpuFloatBuffer::MemoryFromDevive(size_t sizeInBytes, void* const outputMemory) const
void ndBrainCpuFloatBuffer::MemoryFromDevive(size_t, void* const) const
{
	ndAssert(0);
}

void ndBrainCpuFloatBuffer::LoadData(size_t sizeInBytes, const void* const sourceData)
{
	ndBrainMemVector src((ndBrainFloat*)sourceData, ndInt32 (sizeInBytes / sizeof(ndReal)));
	m_buffer.Set(src);
}

//void ndBrainCpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const outputData) const
void ndBrainCpuFloatBuffer::UnloadData(size_t, void* const) const
{
	ndAssert(0);
}


void ndBrainCpuFloatBuffer::CopyBuffer(const ndBrainBuffer& sourceData, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
{
	ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));

	ndBrainCpuFloatBuffer& source = *((ndBrainCpuFloatBuffer*)&sourceData);
	ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	dst.Set(src);
}

void ndBrainCpuFloatBuffer::CopyBufferIndirectSource(const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcDataBuffer, ndInt32 srcStrideIntBytes)
{
	const ndBrainCpuFloatBuffer& srcBuffer = *((ndBrainCpuFloatBuffer*)&srcDataBuffer);
	const ndBrainCpuIntegerBuffer& indirectArray = *((ndBrainCpuIntegerBuffer*)&indexBuffer);
	ndInt32 count = ndInt32 (indirectArray.m_sizeInBytes / sizeof(ndUnsigned32));

	ndInt32 stride = ndInt32 (srcStrideIntBytes / sizeof(ndReal));
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndUnsigned32 index = indirectArray.m_indexArray[i];
		const ndBrainMemVector src(&srcBuffer.m_buffer[index * stride], stride);
		ndBrainMemVector dst(&m_buffer[i * stride], stride);
		dst.Set(src);
	}
}