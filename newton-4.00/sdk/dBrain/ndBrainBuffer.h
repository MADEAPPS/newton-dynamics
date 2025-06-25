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

enum ndStorageBufferType
{
	ndStorageData,
	ndUniformData
};

class ndBrainVector;
class ndBrainMatrix;
class ndBrainContext;

class ndBrainBuffer : public ndClassAlloc
{
	protected:
	ndBrainBuffer(ndBrainContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags);

	public:
	virtual ~ndBrainBuffer();

	size_t SizeInBytes() const { return m_sizeInBytes; }

	virtual void BrainVectorToDevice(const ndBrainVector& vector) = 0;
	virtual void BrainVectorFromDevice(ndBrainVector& vector) const = 0;

	virtual void BrainMatrixToDevice(const ndBrainMatrix* const matrix) = 0;

	virtual void MemoryToDevive(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory) = 0;
	virtual void MemoryFromDevive(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const = 0;

	virtual void CopyBuffer(const ndBrainBuffer& srcBuffer, size_t sourceOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes) = 0;
	virtual void CopyBufferIndirectSource(const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcDataBuffer, size_t strideInBytes) = 0;

	protected:
	virtual void LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData) = 0;
	virtual void UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const = 0;

	ndBrainContext* m_context;
	size_t m_sizeInBytes;
};

#endif