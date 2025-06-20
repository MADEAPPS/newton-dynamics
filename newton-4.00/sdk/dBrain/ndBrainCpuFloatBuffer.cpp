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

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, ndInt64 sizeInFloat)
	:ndBrainBuffer(context, sizeInFloat * ndInt32(sizeof(ndReal)), ndStorageData)
{
	m_buffer.SetCount(sizeInFloat);
	m_buffer.Set(ndBrainFloat(0.0f));
}

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsCpuContext());
	m_buffer.SetCount(ndInt64(m_sizeInBytes / sizeof(ndReal)));
	LoadData(input.GetCount() * sizeof (ndReal), &input[0]);
}

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainBuffer(context, matrix.GetColumns() * matrix.GetRows() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsCpuContext());
	m_buffer.SetCount(ndInt64(m_sizeInBytes / sizeof(ndReal)));
	LoadBuffer(&matrix);
}

//void ndBrainCpuFloatBuffer::LoadData(size_t sizeInBytes, const void* const sourceData)
void ndBrainCpuFloatBuffer::LoadData(size_t, const void* const)
{
	ndAssert(0);
}

//void ndBrainCpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const destinationData) const
void ndBrainCpuFloatBuffer::UnloadData(size_t, void* const) const
{
	ndAssert(0);
}

void ndBrainCpuFloatBuffer::LoadBuffer(const ndBrainMatrix* const matrix)
{
	for (ndInt32 i = 0; i < matrix->GetRows(); ++i)
	{
		ndBrainMemVector dst (&m_buffer[i * matrix->GetColumns()], matrix->GetColumns());
		dst.Set((*matrix)[i]);
	}
}

void ndBrainCpuFloatBuffer::CopyBuffer(const ndBrainBuffer& source, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
{
	size_t size = sizeInBytes / sizeof(ndReal);
	size_t srcOffset = srcOffsetInBytes / sizeof(ndReal);
	size_t dstOffset = dstOffsetInBytes / sizeof(ndReal);
	ndAssert(size <= size_t(m_buffer.GetCount()));
	ndAssert((dstOffset + size) * sizeof(ndReal) <= SizeInBytes());
	ndAssert((srcOffset + size) * sizeof (ndReal) <= source.SizeInBytes());

	ndBrainMemVector dst(&m_buffer[ndInt64(dstOffset)], ndInt64(size));
	const ndBrainCpuFloatBuffer& srcBuffer = *((ndBrainCpuFloatBuffer*)&source);
	const ndBrainMemVector src(&srcBuffer.m_buffer[ndInt64(srcOffset)], ndInt64(size));
	dst.Set(src);
}