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
	:ndBrainGpuBuffer(context, size * ndInt32(sizeof(ndReal)), ndStorageData)
{
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainGpuBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsGpuContext());
	LoadData(input.GetCount() * sizeof (ndReal), &input[0]);
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainGpuBuffer(context, matrix.GetColumns() * matrix.GetRows() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsGpuContext());
	LoadBuffer(&matrix);
}

void ndBrainGpuFloatBuffer::LoadData(size_t sizeInBytes, const void* const sourceData)
{
	//cl_int clEnqueueWriteBuffer(
	//	cl_command_queue command_queue,
	//	cl_mem buffer,
	//	cl_bool blocking_write,
	//	size_t offset,
	//	size_t size,
	//	const void* ptr,
	//	cl_uint num_events_in_wait_list,
	//	const cl_event * event_wait_list,
	//	cl_event * event);

	//cl_int enqueueWriteBuffer(const Buffer& buffer, cl_bool blocking,
	//	size_type offset, size_type size, const void* ptr,
	//	const vector<Event>* events = nullptr, Event* event = nullptr) const

	ndAssert(m_context->GetAsGpuContext());

	cl_int error = 0;
	ndAssert(sizeInBytes == m_sizeInBytes);
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error =	queue->enqueueWriteBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, sourceData);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const destinationData) const
{
	cl_int error = 0;
	ndAssert(sizeInBytes == m_sizeInBytes);
	ndAssert(m_context->GetAsGpuContext());
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	error = queue->enqueueReadBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, destinationData);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuFloatBuffer::LoadBuffer(const ndBrainMatrix* const matrix)
{
	//cl_int enqueueWriteBufferRect(
	//	const Buffer & buffer,
	//	cl_bool blocking,
	//	const array<size_type, 2>&buffer_offset,
	//	const array<size_type, 2>&host_offset,
	//	const array<size_type, 2>&region,
	//	size_type buffer_row_pitch,
	//	size_type buffer_slice_pitch,
	//	size_type host_row_pitch,
	//	size_type host_slice_pitch,
	//	const void* ptr,
	//	const vector<Event>*events = nullptr,
	//	Event * event = nullptr) const

	ndAssert(m_context->GetAsGpuContext());

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
}