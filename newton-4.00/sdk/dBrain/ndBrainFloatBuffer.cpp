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
#include "ndBrainIntegerBuffer.h"
#include "ndBrainUniformBuffer.h"

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
	m_context->BrainVectorToDevice(*this, flatArray);
}

size_t ndBrainFloatBuffer::GetCount() const
{
	return m_sizeInBytes / sizeof(ndReal);
}

void* ndBrainFloatBuffer::GetCpuPtr()
{
	if (m_context->GetAsCpuContext())
	{
		ndBrainVector& dst = **m_buffer;
		return &dst[0];
	}
	ndAssert(0);
	return nullptr;
}

void* ndBrainFloatBuffer::GetCpuPtr() const
{
	if (m_context->GetAsCpuContext())
	{
		const ndBrainVector& dst = **m_buffer;
		return (void*)&dst[0];
	}
	return nullptr;
}

void ndBrainFloatBuffer::CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, const ndBrainFloatBuffer& srcBuffer)
{
	m_context->CopyBufferIndirect(descriptor, indexBuffer, *this, srcBuffer);
}

void ndBrainFloatBuffer::CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
{
	m_context->CopyBuffer(descriptor, workGroupCount, *this, srcBuffer);
}

void ndBrainFloatBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	m_context->MemoryFromDevice(*this, offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainFloatBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	m_context->MemoryToDevice(*this, offsetInBytes, sizeInBytes, inputData);
}

void ndBrainFloatBuffer::VectorFromDevice(ndBrainVector& vector) const
{
	vector.SetCount(ndInt64(m_sizeInBytes / sizeof (ndReal)));
	MemoryFromDevice(0, m_sizeInBytes, &vector[0]);
}

void ndBrainFloatBuffer::VectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() <= ndInt64(m_sizeInBytes / sizeof(ndReal)));
	MemoryToDevice(0, vector.GetCount() * sizeof (ndReal), &vector[0]);
}