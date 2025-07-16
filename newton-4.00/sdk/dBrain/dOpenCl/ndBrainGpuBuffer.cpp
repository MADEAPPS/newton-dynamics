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
	,m_memory(nullptr)
{
	cl_int error = CL_SUCCESS;
	ndBrainGpuContext* const context = m_owner->m_context->GetAsGpuContext();
	ndAssert(context);

	size_t size = m_owner->SizeInBytes();
	if (m_owner->m_isMemoryMapped && context->SupportsMappedMemory())
	{
		m_memory = (ndUnsigned32*)clSVMAlloc(context->m_context->get(), CL_MEM_READ_WRITE | CL_MEM_SVM_FINE_GRAIN_BUFFER, size, 32);
		ndAssert(m_memory);
		m_buffer = ndSharedPtr<cl::Buffer>(new cl::Buffer(**context->m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, size, m_memory, &error));
	}
	else
	{
		m_owner->m_isMemoryMapped = false;
		m_buffer = ndSharedPtr<cl::Buffer> (new cl::Buffer(**context->m_context, CL_MEM_READ_WRITE, size, nullptr, &error));
	}
	ndAssert(error == CL_SUCCESS);
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
	if (m_owner->m_isMemoryMapped)
	{
		clSVMFree(m_owner->m_context->GetAsGpuContext()->m_context->get(), m_memory);
	}
}

void* ndBrainGpuBuffer::GetPtr()
{
	return nullptr;
}

