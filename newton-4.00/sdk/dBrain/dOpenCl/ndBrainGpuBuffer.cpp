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
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"

ndBrainGpuBuffer::ndBrainGpuBuffer(ndBrainContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags)
	:ndBrainBuffer(context, sizeInByte, bufferTypeFlags)
	,m_buffer(**context->GetAsGpuContext()->m_context, cl_mem_flags((bufferTypeFlags == ndStorageData) ? CL_MEM_READ_WRITE : CL_MEM_READ_ONLY), size_t(sizeInByte))
	,m_sizeInBytes(size_t(sizeInByte))
{
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
}

void ndBrainGpuBuffer::CopyData(const ndBrainBuffer& source, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
{
	ndAssert(m_context->GetAsGpuContext());

	const ndBrainGpuBuffer& srcBuffer = *((ndBrainGpuBuffer*)&source);

	size_t srcOffset = srcOffsetInBytes;
	size_t dstOffset = dstOffsetInBytes;
	ndAssert(sizeInBytes <= m_sizeInBytes);
	ndAssert((dstOffset + sizeInBytes) <= m_sizeInBytes);
	ndAssert((srcOffset + sizeInBytes) <= source.SizeInBytes());

	cl_int error = 0;
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error = queue->enqueueCopyBuffer(srcBuffer.m_buffer, m_buffer, srcOffset, dstOffset, sizeInBytes, nullptr, nullptr);
	ndAssert(error == CL_SUCCESS);
}

