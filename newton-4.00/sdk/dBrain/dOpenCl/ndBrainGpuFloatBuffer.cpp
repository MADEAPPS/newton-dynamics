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
#include "ndBrainGpuContext.h"
#include "ndBrainGpuFloatBuffer.h"
//#include "ndBrainGpuScopeMapBuffer.h"

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt64 size, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, size * ndInt32(sizeof(ndReal)), ndStorageData, deviceType)
{
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData, deviceType)
{
	LoadData(input.GetCount() * sizeof (ndReal), &input[0]);
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

	cl_int error = 0;
	ndAssert(sizeInBytes == m_sizeInBytes);
	ndSharedPtr<cl::CommandQueue>& queue = m_context->m_queue;
	error =	queue->enqueueWriteBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, sourceData);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const destinationData) const
{
	cl_int error = 0;
	ndAssert(sizeInBytes == m_sizeInBytes);
	ndSharedPtr<cl::CommandQueue>& queue = m_context->m_queue;
	error = queue->enqueueReadBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, destinationData);
	ndAssert(error == CL_SUCCESS);
}
