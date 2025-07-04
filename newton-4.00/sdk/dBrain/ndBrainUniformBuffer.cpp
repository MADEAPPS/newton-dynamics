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
#include "ndBrainCpuContext.h"
#include "ndBrainUniformBuffer.h"

ndBrainUniformBuffer::ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, bool memoryMapped)
	:ndBrainBuffer(context, sizeInBytes, memoryMapped)
{
	if (m_context->GetAsCpuContext())
	{
		ndAssert(0);
	}
	else
	{
		ndAssert(0);
	}
}

ndBrainUniformBuffer::ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data, bool memoryMapped)
	:ndBrainBuffer(context, sizeInBytes, memoryMapped)
{
	sizeInBytes += (sizeof(ndUnsigned32) - 1) & -ndInt32 (sizeof(ndUnsigned32));
	if (m_context->GetAsCpuContext())
	{
		m_data = ndSharedPtr<ndFixSizeArray<ndUnsigned32, 256>>(new ndFixSizeArray<ndUnsigned32, 256>);
		m_data->SetCount(ndInt32(sizeInBytes / sizeof(ndUnsigned32)));
	}
	MemoryToDevice(0, size_t(sizeInBytes), data);
}

void* ndBrainUniformBuffer::GetCpuPtr()
{
	if (m_context->GetAsCpuContext())
	{
		ndFixSizeArray<ndUnsigned32, 256>& dst = **m_data;
		return &dst[0];
	}
	return nullptr;
}

void* ndBrainUniformBuffer::GetCpuPtr() const
{
	if (m_context->GetAsCpuContext())
	{
		const ndFixSizeArray<ndUnsigned32, 256>& dst = **m_data;
		return (void*)&dst[0];
	}
	return nullptr;
}

void ndBrainUniformBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	m_context->MemoryFromDevice(*this, offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainUniformBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	m_context->MemoryToDevice(*this, offsetInBytes, sizeInBytes, inputData);
}