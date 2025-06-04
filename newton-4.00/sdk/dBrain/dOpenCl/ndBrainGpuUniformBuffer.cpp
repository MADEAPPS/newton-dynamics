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
#include "ndBrainGpuUniformBuffer.h"
//#include "ndBrainGpuScopeMapBuffer.h"

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes)
	:ndBrainGpuBuffer(context, sizeInBytes, ndUniformData, ndCpuMappable)
{
}

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainGpuBuffer(context, sizeInBytes, ndUniformData, ndCpuMappable)
{
	LoadData(size_t(sizeInBytes), data);
}

void ndBrainGpuUniformBuffer::UnloadData(size_t, void* const) const
{
}

void ndBrainGpuUniformBuffer::LoadData(size_t sizeInBytes, const void* const sourceData)
{
	ndSharedPtr<cl::CommandQueue>& queue = m_context->m_queue;

	cl_int error = 0;
	ndAssert(sizeInBytes == m_sizeInBytes);
	error = queue->enqueueWriteBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, sourceData);
	ndAssert(error == CL_SUCCESS);
}
