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
#ifndef __ND_BRAIN_FLOAT_BUFFER_H__
#define __ND_BRAIN_FLOAT_BUFFER_H__

#include "ndBrainBuffer.h"

class ndBrainUniformBuffer;
class ndBrainIntegerBuffer;

class ndBrainFloatBuffer : public ndBrainBuffer
{
	public:
	ndBrainFloatBuffer(ndBrainContext* const context, ndInt64 size, bool memoryMapped = false);
	ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainVector& input, bool memoryMapped = false);
	ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix, bool memoryMapped = false);

	size_t GetCount() const;
	virtual void* GetCpuPtr() override;
	virtual void* GetCpuPtr() const override;

	//virtual void BrainVectorToDevice(const ndBrainVector& vector) override;
	//virtual void BrainVectorFromDevice(ndBrainVector& vector) const override;
	//
	//virtual void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData) override;
	//virtual void CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer) override;
	//void CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainFloatBuffer& srcBuffer);

	void VectorToDevice(const ndBrainVector& vector);
	void VectorFromDevice(ndBrainVector& vector) const;

	void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData);
	void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const;
	void CopyBufferIndirect(const ndBrainUniformBuffer& parameterBuffer, const ndBrainIntegerBuffer& indexBuffer, const ndBrainFloatBuffer& srcBuffer);

	protected:
	ndSharedPtr<ndBrainVector> m_buffer;

	friend class ndBrainCpuContext;
};

#endif