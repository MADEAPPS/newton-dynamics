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
#include "ndBrainGpuBuffer.h"
#include "ndBrainCpuContext.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainUniformBuffer.h"

ndBrainFloatBuffer::ndBrainFloatBuffer(ndBrainContext* const context, ndInt64 sizeInFloat)
	:ndBrainBuffer(context, sizeInFloat * ndInt32(sizeof(ndReal)))
{
	if (m_context->GetAsCpuContext())
	{
		m_buffer = ndSharedPtr<ndBrainVector>(new ndBrainVector());
		m_buffer->SetCount(sizeInFloat);
	}
}

ndBrainFloatBuffer::ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)))
{
	if (m_context->GetAsCpuContext())
	{
		m_buffer = ndSharedPtr<ndBrainVector>(new ndBrainVector());
		m_buffer->SetCount(input.GetCount());
	}
	m_context->BrainVectorToDevice(*this, input);
}

ndBrainFloatBuffer::ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainBuffer(context, matrix.GetColumns() * matrix.GetRows() * ndInt32(sizeof(ndReal)))
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

ndBrainFloatBuffer::ndBrainFloatBuffer(const ndBrainFloatBuffer& src)
	:ndBrainBuffer(src)
{
	if (m_context->GetAsCpuContext())
	{
		m_buffer = ndSharedPtr<ndBrainVector>(new ndBrainVector());
		m_buffer->SetCount(ndInt64(src.m_sizeInBytes / sizeof(ndReal)));
	}
	m_context->CopyBuffer(*this, src);
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
	ndAssert(GetGpuBuffer());
	return GetGpuBuffer()->GetPtr();
}

void* ndBrainFloatBuffer::GetCpuPtr() const
{
	if (m_context->GetAsCpuContext())
	{
		const ndBrainVector& dst = **m_buffer;
		return (void*)&dst[0];
	}
	ndAssert(GetGpuBuffer());
	return (void*)GetGpuBuffer()->GetPtr();
}

void ndBrainFloatBuffer::CopyBuffer(const ndBrainFloatBuffer& srcBuffer)
{
	m_context->CopyBuffer(*this, srcBuffer);
}

void ndBrainFloatBuffer::CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 workGroupCount, const ndBrainFloatBuffer& srcBuffer)
{
	m_context->CopyBuffer(descriptor, workGroupCount, *this, srcBuffer);
}

void ndBrainFloatBuffer::CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, const ndBrainFloatBuffer& srcBuffer)
{
	m_context->CopyBufferIndirect(descriptor, indexBuffer, *this, srcBuffer);
}

void ndBrainFloatBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	m_context->MemoryFromDevice(*this, offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainFloatBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	ndAssert((offsetInBytes + sizeInBytes) <= SizeInBytes());
	m_context->MemoryToDevice(*this, offsetInBytes, sizeInBytes, inputData);
}

void ndBrainFloatBuffer::VectorFromDevice(ndBrainVector& vector) const
{
	vector.SetCount(ndInt64(m_sizeInBytes / sizeof(ndReal)));
	MemoryFromDevice(0, m_sizeInBytes, &vector[0]);
}

void ndBrainFloatBuffer::VectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() <= ndInt64(m_sizeInBytes / sizeof(ndReal)));
	MemoryToDevice(0, vector.GetCount() * sizeof(ndReal), &vector[0]);
}

void ndBrainFloatBuffer::Set(const ndBrainFloatBuffer& buffer)
{
	m_context->Set(*this, buffer);
}

void ndBrainFloatBuffer::Reciprocal(const ndBrainFloatBuffer& buffer)
{
	m_context->Reciprocal(*this, buffer);
}

void ndBrainFloatBuffer::SetOrdinal()
{
	m_context->SetOrdinal(*this);
}

void ndBrainFloatBuffer::ReductionSum()
{
	m_context->ReductionSum(*this);
}

void ndBrainFloatBuffer::Add(const ndBrainFloatBuffer& buffer)
{
	m_context->Add(*this, buffer);
}

void ndBrainFloatBuffer::Sub(const ndBrainFloatBuffer& buffer)
{
	m_context->Sub(*this, buffer);
}

void ndBrainFloatBuffer::Mul(const ndBrainFloatBuffer& buffer)
{
	m_context->Mul(*this, buffer);
}

void ndBrainFloatBuffer::Min(const ndBrainFloatBuffer& buffer)
{
	m_context->Min(*this, buffer);
}

void ndBrainFloatBuffer::Max(const ndBrainFloatBuffer& buffer)
{
	m_context->Max(*this, buffer);
}

void ndBrainFloatBuffer::LessEqual(const ndBrainFloatBuffer& buffer)
{
	m_context->LessEqual(*this, buffer);
}

void ndBrainFloatBuffer::GreaterEqual(const ndBrainFloatBuffer& buffer)
{
	m_context->GreaterEqual(*this, buffer);
}

void ndBrainFloatBuffer::Set(ndBrainFloat value)
{
	m_context->Set(*this, value);
}

void ndBrainFloatBuffer::Scale(ndBrainFloat value)
{
	m_context->Scale(*this, value);
}

void ndBrainFloatBuffer::Min(ndBrainFloat value)
{
	m_context->Min(*this, value);
}

void ndBrainFloatBuffer::Max(ndBrainFloat value)
{
	m_context->Max(*this, value);
}

void ndBrainFloatBuffer::Less(ndBrainFloat value)
{
	m_context->Less(*this, value);
}

void ndBrainFloatBuffer::Greater(ndBrainFloat value)
{
	m_context->Greater(*this, value);
}

void ndBrainFloatBuffer::LessEqual(ndBrainFloat value)
{
	m_context->LessEqual(*this, value);
}

void ndBrainFloatBuffer::GreaterEqual(ndBrainFloat value)
{
	m_context->GreaterEqual(*this, value);
}

void ndBrainFloatBuffer::Blend(const ndBrainFloatBuffer& buffer, ndBrainFloat blendFactor)
{
	m_context->Blend(*this, buffer, blendFactor);
}

void ndBrainFloatBuffer::Blend(const ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& blendFactor)
{
	m_context->Blend(*this, buffer, blendFactor);
}

void ndBrainFloatBuffer::StandardNormalDistribution()
{
	m_context->StandardNormalDistribution(*this);
}

void ndBrainFloatBuffer::BroadcastScaler(const ndBrainFloatBuffer& srcScalars)
{
	ndInt32 stride = ndInt32(GetCount() / srcScalars.GetCount());
	ndAssert(stride * srcScalars.GetCount() == GetCount());
	m_context->BroadcastScaler(*this, stride, srcScalars);
}

void ndBrainFloatBuffer::CalculateEntropyRegularization(const ndBrainFloatBuffer& sample, const ndBrainFloatBuffer& sigma, ndBrainFloat regularization)
{
	m_context->CalculateEntropyRegularization(*this, sample, sigma, regularization);
}

void ndBrainFloatBuffer::CalculateEntropyRegularizationGradient(const ndBrainFloatBuffer& sample, const ndBrainFloatBuffer& sigma, ndBrainFloat regularization, ndInt32 inputSize)
{
	m_context->CalculateEntropyRegularizationGradient(*this, sample, sigma, regularization, inputSize);
}