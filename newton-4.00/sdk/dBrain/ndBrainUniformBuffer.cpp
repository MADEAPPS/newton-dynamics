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
		//ndFixSizeArray<ndUnsigned32, 256>& dst = **m_data;
		//ndMemCpy(&dst[0], (ndUnsigned32*)data, m_data->GetCount());
	}
	else
	{
		ndAssert(0);
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


#if 0
void ndBrainUniformBuffer::BrainVectorToDevice(const ndBrainVector&)
{
	ndAssert(0);
	//ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	//LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainUniformBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainUniformBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainUniformBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	ndAssert(0);
	LoadData(offsetInBytes, sizeInBytes, inputData);
}

void ndBrainUniformBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	ndAssert(0);
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainUniformBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(0);
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	m_data.SetCount(ndInt32 (size));
	ndInt32 offset = ndInt32(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy(&m_data[offset], (ndUnsigned32*)sourceData, ndInt64(size));
}

void ndBrainUniformBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
{
	ndAssert(0);
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	ndInt32 offset = ndInt32(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy((ndUnsigned32*)outputData, &m_data[offset], ndInt64(size));
}

//void ndBrainUniformBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
void ndBrainUniformBuffer::CopyBuffer(const ndBrainBuffer&, ndInt32, const ndBrainBuffer&)
{
	ndAssert(0);
	//ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	//ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));
	//
	//ndBrainUniformBuffer& source = *((ndBrainUniformBuffer*)&sourceData);
	//ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);
}

//void ndBrainUniformBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
void ndBrainUniformBuffer::CopyBufferIndirect(const ndBrainBuffer&, const ndBrainBuffer&, const ndBrainBuffer&)
{
	ndAssert(0);
}

#endif

void ndBrainUniformBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	m_context->MemoryFromDevice(*this, offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainUniformBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	m_context->MemoryToDevice(*this, offsetInBytes, sizeInBytes, inputData);
}