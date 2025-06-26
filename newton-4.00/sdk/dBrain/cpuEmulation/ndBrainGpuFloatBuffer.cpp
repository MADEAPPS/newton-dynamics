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
#include "ndBrainGpuUniformBuffer.h"
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
	ndBrainVector flatArray;
	ndAssert(m_context->GetAsGpuContext());
	m_buffer.SetCount(matrix.GetColumns() * matrix.GetRows());
	for (ndInt32 i = 0; i < matrix.GetRows(); i++)
	{
		for (ndInt32 j = 0; j < matrix.GetColumns(); j++)
		{
			flatArray.PushBack(matrix[i][j]);
		}
	}
	BrainVectorToDevice(flatArray);
}

ndBrainFloat* ndBrainGpuFloatBuffer::GetData()
{
	return &m_buffer[0];
}

void ndBrainGpuFloatBuffer::BrainVectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	LoadData(0, vector.GetCount() * sizeof(ndReal), &vector[0]);
}

void ndBrainGpuFloatBuffer::BrainVectorFromDevice(ndBrainVector& output) const
{
	output.SetCount(m_buffer.GetCount());
	UnloadData(0, output.GetCount() * sizeof(ndReal), &output[0]);
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

void ndBrainGpuFloatBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcData)
{
	const ndBrainGpuFloatBuffer& srcBuffer = *((ndBrainGpuFloatBuffer*)&srcData);
	const ndBrainGpuIntegerBuffer& indirectArray = *((ndBrainGpuIntegerBuffer*)&indexBuffer);
	const ndBrainGpuUniformBuffer& uniforms = *(ndBrainGpuUniformBuffer*)&parameterBuffer;
	const ndCopyBufferCommandInfo& data = *((ndCopyBufferCommandInfo*)&uniforms.m_data[0]);

	ndInt32 stride = ndInt32(data.m_strideInByte / sizeof(ndReal));
	ndInt32 srcStride = ndInt32(data.m_srcStrideInByte / sizeof(ndReal));
	ndInt32 srcOffset = ndInt32(data.m_srcOffsetInByte / sizeof(ndReal));
	ndInt32 dstStride = ndInt32(data.m_dstStrideInByte / sizeof(ndReal));
	ndInt32 dstOffset = ndInt32(data.m_dstOffsetInByte / sizeof(ndReal));
	ndInt32 count = ndInt32(indirectArray.SizeInBytes() / sizeof(ndUnsigned32));
	
	ndAssert(stride <= srcStride);
	ndAssert(stride <= dstStride);
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndUnsigned32 index = indirectArray.m_indexArray[i];
		const ndBrainMemVector src(&srcBuffer.m_buffer[index * srcStride + srcOffset], stride);
		ndBrainMemVector dst(&m_buffer[i * dstStride + dstOffset], stride);
		dst.Set(src);
	}
}