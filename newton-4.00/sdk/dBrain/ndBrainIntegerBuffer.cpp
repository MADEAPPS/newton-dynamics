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
#include "ndBrainMatrix.h"
#include "ndBrainCpuContext.h"
#include "ndBrainIntegerBuffer.h"

ndBrainIntegerBuffer::ndBrainIntegerBuffer(ndBrainContext* const context, ndInt64 sizeInElements, bool memoryMapped)
	:ndBrainBuffer(context, sizeInElements * ndInt64(sizeof(ndUnsigned32)), memoryMapped)
{
	if (m_context->GetAsCpuContext())
	{
		m_indexArray = ndSharedPtr<ndArray<ndUnsigned32>>(new ndArray<ndUnsigned32>);
		m_indexArray->SetCount(sizeInElements);
	}
	//m_context->MemoryToDevice(*this, 0, flatArray);
}

ndBrainIntegerBuffer::ndBrainIntegerBuffer(ndBrainContext* const context, ndInt64 numberOfElements, const ndUnsigned32* const indexArray, bool memoryMapped)
	:ndBrainBuffer(context, numberOfElements * ndInt64(sizeof(ndUnsigned32)), memoryMapped)
{
	if (m_context->GetAsCpuContext())
	{
		m_indexArray = ndSharedPtr<ndArray<ndUnsigned32>>(new ndArray<ndUnsigned32>);
		m_indexArray->SetCount(numberOfElements);
	}
	MemoryToDevice(0, m_sizeInBytes, indexArray);
}

void* ndBrainIntegerBuffer::GetCpuPtr()
{
	if (m_context->GetAsCpuContext())
	{
		ndArray<ndUnsigned32>& dst = **m_indexArray;
		return &dst[0];
	}
	return nullptr;
}

void* ndBrainIntegerBuffer::GetCpuPtr() const
{
	if (m_context->GetAsCpuContext())
	{
		const ndArray<ndUnsigned32>& dst = **m_indexArray;
		return (void*)&dst[0];
	}
	return nullptr;
}

void ndBrainIntegerBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	m_context->MemoryFromDevice(*this, offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainIntegerBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData)
{
	m_context->MemoryToDevice(*this, offsetInBytes, sizeInBytes, inputData);
}
