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
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"

ndBrainGpuBuffer::ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags, ndDeviceBufferType)
	:m_buffer(**context->m_context, cl_mem_flags((bufferTypeFlags == ndStorageData) ? CL_MEM_READ_WRITE : CL_MEM_READ_ONLY), size_t(sizeInByte))
	,m_context(context)
	,m_sizeInBytes(size_t(sizeInByte))
{
	//ignore buffers mappable to cpu memory CL_MEM_USE_HOST_PTR
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
}
