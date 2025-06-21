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

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, ndInt64 size)
	:ndBrainBuffer(context, size * ndInt32(sizeof(ndReal)), ndStorageData)
	,m_buffer()
{
	ndAssert(0);
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(0);
	//LoadData(input.GetCount() * sizeof (ndBrainFloat),  & input[0]);
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainBuffer(context, matrix.GetColumns()* matrix.GetRows()* ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(0);
	//ndAssert(m_context->GetAsGpuContext());
	//BrainMatrixToDevice(&matrix);
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
	ndAssert(0);
}

void ndBrainGpuFloatBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
{
	ndAssert(0);
}

void ndBrainGpuFloatBuffer::BrainMatrixToDevice(const ndBrainMatrix* const matrix)
{
	ndAssert(0);
}

void ndBrainGpuFloatBuffer::MemoryToDevive(size_t sizeInBytes, const void* const inputMemory)
{
	ndAssert(0);
}

void ndBrainGpuFloatBuffer::MemoryFromDevive(size_t sizeInBytes, void* const outputMemory) const
{
	ndAssert(0);
}

void ndBrainGpuFloatBuffer::CopyBuffer(const ndBrainBuffer& srcBuffer, size_t sourceOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
{
	ndAssert(0);
}

void ndBrainGpuFloatBuffer::LoadData(size_t sizeInBytes, const void* const inputData)
{
	ndAssert(0);
}

void ndBrainGpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const outputData) const
{
	ndAssert(0);
}
