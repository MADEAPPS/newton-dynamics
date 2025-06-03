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
#include "ndBrainGpuScopeMapBuffer.h"

ndScopeMapBuffer::ndScopeMapBuffer(const ndBrainGpuBuffer& buffer)
	:m_mappedMemory(nullptr)
	,m_buffer(&buffer)
{
	VkDevice const device = (VkDevice)m_buffer->m_context->GetDevice();

	// Map the buffer memory, so that we can read from it on the CPU.
	vkMapMemory(device, m_buffer->m_bufferMemory, 0, ndUnsigned32(m_buffer->m_sizeInBytes), 0, &m_mappedMemory);
}

ndScopeMapBuffer::~ndScopeMapBuffer()
{
	VkDevice const device = (VkDevice)m_buffer->m_context->GetDevice();
	vkUnmapMemory(device, m_buffer->m_bufferMemory);
}

void* ndScopeMapBuffer::GetPointer() const
{
	return m_mappedMemory;
}
