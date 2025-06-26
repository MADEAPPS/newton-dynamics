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
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"

ndBrainGpuBuffer::ndBrainGpuBuffer(ndBrainContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags)
	:ndBrainBuffer(context, sizeInByte, bufferTypeFlags)
	,m_buffer(**context->GetAsGpuContext()->m_context, cl_mem_flags((bufferTypeFlags == ndStorageData) ? CL_MEM_READ_WRITE : CL_MEM_READ_ONLY), size_t(sizeInByte))
{
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
}

void ndBrainGpuBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory)
{
	LoadData(offsetInBytes, sizeInBytes, inputMemory);
}

void ndBrainGpuBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainGpuBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
{
	ndInt64 size = ndInt64(m_sizeInBytes / sizeof(ndReal));
	vector.SetCount(size);
	UnloadData(0, m_sizeInBytes, &vector[0]);
}

void ndBrainGpuBuffer::BrainVectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() * sizeof (ndReal) <= m_sizeInBytes);
	LoadData(0, vector.GetCount() * sizeof(ndReal), &vector[0]);
}

void ndBrainGpuBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(m_context->GetAsGpuContext());

	cl_int error = 0;
	ndAssert(sizeInBytes <= m_sizeInBytes);
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error = queue->enqueueWriteBuffer(m_buffer, CL_TRUE, offsetInBytes, sizeInBytes, sourceData);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const destinationData) const
{
	cl_int error = 0;
	ndAssert(m_context->GetAsGpuContext());
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error = queue->enqueueReadBuffer(m_buffer, CL_TRUE, offsetInBytes, sizeInBytes, destinationData);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
{
	ndAssert(m_context->GetAsGpuContext());
	ndBrainGpuContext* const context = m_context->GetAsGpuContext();
	ndAssert(context);

	ndBrainGpuFloatBuffer& dst = *(ndBrainGpuFloatBuffer*)this;
	ndBrainGpuFloatBuffer& src = *(ndBrainGpuFloatBuffer*)&srcBuffer;
	ndBrainGpuUniformBuffer& uniforms = *(ndBrainGpuUniformBuffer*)&parameterBuffer;

	context->CopyBuffer(uniforms, workGroupCount, dst, src);
}

void ndBrainGpuBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
{
	ndBrainGpuContext* const context = m_context->GetAsGpuContext();
	ndAssert(context);

	ndBrainGpuFloatBuffer& dst = *(ndBrainGpuFloatBuffer*)this;
	ndBrainGpuFloatBuffer& src = *(ndBrainGpuFloatBuffer*)&srcBuffer;
	ndBrainGpuIntegerBuffer& index = *(ndBrainGpuIntegerBuffer*)&indexBuffer;
	ndBrainGpuUniformBuffer& uniforms = *(ndBrainGpuUniformBuffer*)&parameterBuffer;

	context->CopyBufferIndirect(uniforms, index, dst, src);
}