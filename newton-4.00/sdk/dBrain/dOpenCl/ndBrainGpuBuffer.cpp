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
#include "ndBrainBuffer.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"

ndBrainGpuBuffer::ndBrainGpuBuffer(const ndBrainBuffer* const owner)
	:ndClassAlloc()
	,m_buffer()
	,m_owner((ndBrainBuffer*)owner)
{
	cl_int error = CL_SUCCESS;
	ndBrainGpuContext* const context = m_owner->m_context->GetAsGpuContext();
	ndAssert(context);

	size_t size = m_owner->SizeInBytes();
	ndAssert(size < context->m_maxBufferSize);

	m_buffer = ndSharedPtr<cl::Buffer> (new cl::Buffer(**context->m_context, CL_MEM_READ_WRITE, size, nullptr, &error));
	ndAssert(error == CL_SUCCESS);
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
}

void* ndBrainGpuBuffer::GetPtr()
{
	return nullptr;
}

const void* ndBrainGpuBuffer::GetPtr() const
{
	return nullptr;
}
