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

ndBrainGpuBuffer::ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt64 sizeInBytes, ndStorageBufferType)
	:m_context (context)
	,m_sizeInBytes(size_t(sizeInBytes))
{
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
}

size_t ndBrainGpuBuffer::SizeInBytes() const
{ 
	return m_sizeInBytes; 
}

ndBrainGpuFloatBuffer* ndBrainGpuBuffer::GetAsFloatBuffer()
{
	return nullptr;
}

ndBrainGpuUniformBuffer* ndBrainGpuBuffer::GetAsUniformBuffer()
{
	return nullptr;
}
