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
#include "ndBrainContext.h"
#include "ndBrainCpuContext.h"
#include "ndBrainFloatBuffer.h"

ndBrainFloatBuffer::ndBrainFloatBuffer(ndBrainContext* const context, ndInt64 sizeInFloat, bool memoryMapped)
	:ndBrainBuffer(context, sizeInFloat * ndInt32(sizeof(ndReal)), memoryMapped)
{
	ndAssert(0);
	//m_buffer.SetCount(sizeInFloat);
	//m_buffer.Set(ndBrainFloat(0.0f));
}

ndBrainFloatBuffer::ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainVector& input, bool memoryMapped)
	:ndBrainBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), memoryMapped)
{
	if (m_context->GetAsCpuContext())
	{
		m_buffer = ndSharedPtr<ndBrainVector>(new ndBrainVector());
		m_buffer->SetCount(input.GetCount());
	}
	else
	{
		ndAssert(0);
	}
	m_context->BrainVectorToDevice(*this, input);
}

ndBrainFloatBuffer::ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix, bool memoryMapped)
	:ndBrainBuffer(context, matrix.GetColumns() * matrix.GetRows() * ndInt32(sizeof(ndReal)), memoryMapped)
{
	ndBrainVector flatArray;
	for (ndInt32 i = 0; i < matrix.GetRows(); i++)
	{
		for (ndInt32 j = 0; j < matrix.GetColumns(); j++)
		{
			flatArray.PushBack(matrix[i][j]);
		}
	}

	if (m_context->GetAsCpuContext())
	{
		m_buffer = ndSharedPtr<ndBrainVector>(new ndBrainVector());
		m_buffer->SetCount(matrix.GetColumns() * matrix.GetRows());
	}
	else
	{
		ndAssert(0);
		//BrainVectorToDevice(flatArray);
	}
	m_context->BrainVectorToDevice(*this, flatArray);
}

size_t ndBrainFloatBuffer::GetCount() const
{
	return m_sizeInBytes / sizeof(ndReal);
}

#if 0
void ndBrainFloatBuffer::BrainVectorToDevice(const ndBrainVector& vector)
{
	ndAssert(0);
	//ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	//LoadData(0, vector.GetCount() * sizeof(ndReal), &vector[0]);
}

void ndBrainFloatBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
{
	UnloadData(0, vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainFloatBuffer::MemoryToDevice(size_t sizeInBytes, const void* const inputData)
void ndBrainFloatBuffer::MemoryToDevice(size_t, size_t, const void* const)
{
	ndAssert(0);
}

void ndBrainFloatBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(0);
	//const ndBrainMemVector src((ndBrainFloat*)sourceData, ndInt32 (sizeInBytes / sizeof(ndReal)));
	//ndBrainMemVector dst(&m_buffer[ndInt64(offsetInBytes / sizeof(ndReal))], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);
}

//void ndBrainFloatBuffer::UnloadData(size_t sizeInBytes, void* const outputData) const
void ndBrainFloatBuffer::UnloadData(size_t, size_t, void* const) const
{
	ndAssert(0);
}

void ndBrainFloatBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcData)
{
	ndAssert(0);
	//const ndBrainFloatBuffer& srcBuffer = *((ndBrainFloatBuffer*)&srcData);
	//const ndBrainCpuUniformBuffer& uniforms = *(ndBrainCpuUniformBuffer*)&parameterBuffer;
	//const ndCopyBufferCommandInfo& data = *((ndCopyBufferCommandInfo*)&uniforms.m_data[0]);
	//
	//ndInt32 stride = ndInt32(data.m_strideInByte / sizeof(ndReal));
	//ndInt32 srcStride = ndInt32(data.m_srcStrideInByte / sizeof(ndReal));
	//ndInt32 srcOffset = ndInt32(data.m_srcOffsetInByte / sizeof(ndReal));
	//ndInt32 dstStride = ndInt32(data.m_dstStrideInByte / sizeof(ndReal));
	//ndInt32 dstOffset = ndInt32(data.m_dstOffsetInByte / sizeof(ndReal));
	//
	//ndAssert(stride <= srcStride);
	//ndAssert(stride <= dstStride);
	//for (ndInt32 i = 0; i < workGroupCount; ++i)
	//{
	//	const ndBrainMemVector src(&srcBuffer.m_buffer[i * srcStride + srcOffset], stride);
	//	ndBrainMemVector dst(&m_buffer[i * dstStride + dstOffset], stride);
	//	dst.Set(src);
	//}
}

void ndBrainFloatBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcData)
{
	ndAssert(0);
	//if (m_context->GetAsCpuContext())
	//{
	//	//const ndBrainFloatBuffer& srcBuffer = *((ndBrainFloatBuffer*)&srcData);
	//	//const ndBrainCpuIntegerBuffer& indirectArray = *((ndBrainCpuIntegerBuffer*)&indexBuffer);
	//	//const ndBrainCpuUniformBuffer& uniforms = *(ndBrainCpuUniformBuffer*)&parameterBuffer;
	//	//const ndCopyBufferCommandInfo& data = *((ndCopyBufferCommandInfo*)&uniforms.m_data[0]);
	//	//
	//	//ndInt32 stride = ndInt32(data.m_strideInByte / sizeof(ndReal));
	//	//ndInt32 srcStride = ndInt32(data.m_srcStrideInByte / sizeof(ndReal));
	//	//ndInt32 srcOffset = ndInt32(data.m_srcOffsetInByte / sizeof(ndReal));
	//	//ndInt32 dstStride = ndInt32(data.m_dstStrideInByte / sizeof(ndReal));
	//	//ndInt32 dstOffset = ndInt32(data.m_dstOffsetInByte / sizeof(ndReal));
	//	//ndInt32 count = ndInt32(indirectArray.SizeInBytes() / sizeof(ndUnsigned32));
	//	//
	//	//ndAssert(stride <= srcStride);
	//	//ndAssert(stride <= dstStride);
	//	//for (ndInt32 i = 0; i < count; ++i)
	//	//{
	//	//	ndUnsigned32 index = indirectArray.m_indexArray[i];
	//	//	const ndBrainMemVector src(&srcBuffer.m_buffer[index * srcStride + srcOffset], stride);
	//	//	ndBrainMemVector dst(&m_buffer[i * dstStride + dstOffset], stride);
	//	//	dst.Set(src);
	//	//}
	//}
	//else
	//{
	//	ndAssert(0);
	//}
}

void ndBrainFloatBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainFloatBuffer& srcBuffer)
{
	m_context->CopyBufferIndirect(parameterBuffer, indexBuffer, *this, srcBuffer);
}

void ndBrainFloatBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	m_context->MemoryFromDevice(*this, offsetInBytes, sizeInBytes, outputMemory);
}

#endif