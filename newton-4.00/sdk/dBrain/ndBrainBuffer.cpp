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
#include "ndBrainContext.h"
#include "ndBrainGpuBuffer.h"

ndBrainBuffer::ndBrainBuffer(const ndBrainBuffer& src)
	:ndContainersFreeListAlloc<ndBrainBuffer>()
	,m_context(src.m_context)
	,m_sizeInBytes(src.m_sizeInBytes)
{
	if (m_context->GetAsGpuContext())
	{
		m_gpuBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuBuffer(this));
	}
}

ndBrainBuffer::ndBrainBuffer(ndBrainContext* const context, ndInt64 sizeInByte)
	:ndContainersFreeListAlloc<ndBrainBuffer>()
	,m_context(context)
	,m_sizeInBytes(size_t(sizeInByte))
{
	if (m_context->GetAsGpuContext())
	{
		m_gpuBuffer = ndSharedPtr<ndBrainGpuBuffer> (new ndBrainGpuBuffer(this));
	}
}

ndBrainBuffer::~ndBrainBuffer()
{
}

size_t ndBrainBuffer::SizeInBytes() const
{ 
	return m_sizeInBytes; 
}

ndBrainGpuBuffer* ndBrainBuffer::GetGpuBuffer()
{
	return *m_gpuBuffer;
}

const ndBrainGpuBuffer* ndBrainBuffer::GetGpuBuffer() const
{
	return *m_gpuBuffer;
}