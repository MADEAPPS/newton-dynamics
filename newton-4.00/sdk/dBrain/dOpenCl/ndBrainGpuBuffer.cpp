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

void ndBrainGpuBuffer::MemoryToDevive(size_t sizeInBytes, const void* const inputMemory)
{
	LoadData(sizeInBytes, inputMemory);
}

void ndBrainGpuBuffer::MemoryFromDevive(size_t sizeInBytes, void* const outputMemory) const
{
	UnloadData(sizeInBytes, outputMemory);
}

void ndBrainGpuBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
{
	ndInt64 size = ndInt64(m_sizeInBytes / sizeof(ndReal));
	vector.SetCount(size);
	UnloadData(m_sizeInBytes, &vector[0]);
}

void ndBrainGpuBuffer::BrainVectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() * sizeof (ndReal) <= m_sizeInBytes);
	LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

void ndBrainGpuBuffer::BrainMatrixToDevice(const ndBrainMatrix* const matrix)
{
	ndAssert(m_context->GetAsGpuContext());

#if 0
	std::array<size_t, 2> region;
	std::array<size_t, 2> host_offset;
	std::array<size_t, 2> buffer_offset;
		
	host_offset[0] = 0;
	host_offset[1] = 0;
	buffer_offset[0] = 0;
	buffer_offset[1] = 0;
	region[1] = size_t(matrix->GetRows());
	region[0] = size_t(matrix->GetColumns() * sizeof(ndReal));
	
	cl_int error = 0;
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error = queue->enqueueWriteBufferRect(
		m_buffer, CL_TRUE,
		buffer_offset, host_offset, region,
		matrix->GetColumns() * sizeof(ndReal), 0,
		matrix->GetColumns() * sizeof(ndReal), 0,
		&matrix[0][0]);
	ndAssert(error == CL_SUCCESS);
#else

	ndBrainVector flatArray;
	for (ndInt32 i = 0; i < matrix->GetRows(); i ++)
	{
		for (ndInt32 j = 0; j < matrix->GetColumns(); j++)
		{
			flatArray.PushBack((*matrix)[i][j]);
		}
	}
	BrainVectorToDevice(flatArray);
#endif

	//ndBrainVector xxx0;
	//BrainVectorFromDevice(xxx0);
	//error *= 1;
}

void ndBrainGpuBuffer::CopyBuffer(const ndBrainBuffer& source, size_t srcOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes)
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

void ndBrainGpuBuffer::LoadData(size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(m_context->GetAsGpuContext());

	cl_int error = 0;
	ndAssert(sizeInBytes <= m_sizeInBytes);
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error = queue->enqueueWriteBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, sourceData);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuBuffer::UnloadData(size_t sizeInBytes, void* const destinationData) const
{
	cl_int error = 0;
	//ndAssert(sizeInBytes == m_sizeInBytes);
	ndAssert(m_context->GetAsGpuContext());
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error = queue->enqueueReadBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, destinationData);
	ndAssert(error == CL_SUCCESS);
}
