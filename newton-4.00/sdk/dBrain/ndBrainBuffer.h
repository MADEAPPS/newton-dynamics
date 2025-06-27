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
#ifndef __ND_BRAIN_BUFFER_H__
#define __ND_BRAIN_BUFFER_H__

#include "ndBrainStdafx.h"

class ndBrainVector;
class ndBrainMatrix;
class ndBrainContext;

class ndCopyBufferCommandInfo
{
	public:
	ndCopyBufferCommandInfo()
		:m_strideInByte(0)
		,m_srcStrideInByte(0)
		,m_srcOffsetInByte(0)
		,m_dstStrideInByte(0)
		,m_dstOffsetInByte(0)
	{
	}

	ndInt32 m_strideInByte;
	ndInt32 m_srcStrideInByte;
	ndInt32 m_srcOffsetInByte;
	ndInt32 m_dstStrideInByte;
	ndInt32 m_dstOffsetInByte;
};

class ndBrainBuffer : public ndClassAlloc
{
	protected:
	ndBrainBuffer(ndBrainContext* const context, ndInt64 sizeInByte);

	public:
	virtual ~ndBrainBuffer();

	size_t SizeInBytes() const;

	virtual void BrainVectorToDevice(const ndBrainVector& vector) = 0;
	virtual void BrainVectorFromDevice(ndBrainVector& vector) const = 0;

	virtual void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory) = 0;
	virtual void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const = 0;

	virtual void CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer) = 0;
	virtual void CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer) = 0;

	protected:
	virtual void LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData) = 0;
	virtual void UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const = 0;

	ndBrainContext* m_context;
	size_t m_sizeInBytes;
};

#endif