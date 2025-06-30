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
	//ndAssert(m_context->GetAsCpuContext());
	//m_indexArray.SetCount(numberOfElements);
	//MemoryToDevice(0, m_sizeInBytes, indexArray);
	if (m_context->GetAsCpuContext())
	{
		m_indexArray = ndSharedPtr<ndArray<ndUnsigned32>>(new ndArray<ndUnsigned32>);
		m_indexArray->SetCount(numberOfElements);
	}
	else
	{
		ndAssert(0);
		//BrainVectorToDevice(flatArray);
	}
	MemoryToDevice(0, m_sizeInBytes, indexArray);
}

#if 0
void ndBrainIntegerBuffer::BrainVectorToDevice(const ndBrainVector&)
{
	ndAssert(0);
	//ndAssert(vector.GetCount() * sizeof(ndReal) <= m_sizeInBytes);
	//LoadData(vector.GetCount() * sizeof(ndReal), &vector[0]);
}

//void ndBrainIntegerBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
void ndBrainIntegerBuffer::BrainVectorFromDevice(ndBrainVector&) const
{
	ndAssert(0);
}

//void ndBrainIntegerBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
void ndBrainIntegerBuffer::MemoryFromDevice(size_t, size_t, void* const) const
{
	ndAssert(0);
}

void ndBrainIntegerBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(sizeInBytes <= m_sizeInBytes);
	size_t size = ndMin(sizeInBytes, m_sizeInBytes) / sizeof(ndUnsigned32);
	ndInt64 offset = ndInt64(offsetInBytes / sizeof(ndUnsigned32));
	ndMemCpy(&m_indexArray[offset], (ndUnsigned32*)sourceData, ndInt64(size));
}

//void ndBrainIntegerBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const
void ndBrainIntegerBuffer::UnloadData(size_t, size_t, void* const) const
{
	ndAssert(0);
}


//void ndBrainIntegerBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
void ndBrainIntegerBuffer::CopyBuffer(const ndBrainBuffer&, ndInt32, const ndBrainBuffer&)
{
	ndAssert(0);
	//ndInt64 dstOffset = ndInt64(dstOffsetInBytes / sizeof(ndReal));
	//ndInt64 srcOffset = ndInt64(srcOffsetInBytes / sizeof(ndReal));
	//
	//ndBrainIntegerBuffer& source = *((ndBrainIntegerBuffer*)&sourceData);
	//ndBrainMemVector dst(&m_buffer[dstOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//const ndBrainMemVector src(&source.m_buffer[srcOffset], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);
}

//void ndBrainIntegerBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
void ndBrainIntegerBuffer::CopyBufferIndirect(const ndBrainBuffer&, const ndBrainBuffer&, const ndBrainBuffer&)
{
	ndAssert(0);
}
#endif

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
