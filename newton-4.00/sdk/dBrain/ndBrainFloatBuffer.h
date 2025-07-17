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
	ndBrainFloatBuffer(const ndBrainFloatBuffer& src);
	ndBrainFloatBuffer(ndBrainContext* const context, ndInt64 size, bool memoryMapped = false);
	ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainVector& input, bool memoryMapped = false);
	ndBrainFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix, bool memoryMapped = false);

	size_t GetCount() const;
	virtual void* GetCpuPtr() override;
	virtual void* GetCpuPtr() const override;

	void VectorToDevice(const ndBrainVector& vector);
	void VectorFromDevice(ndBrainVector& vector) const;

	void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData);
	void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const;

	void CopyBuffer(const ndBrainFloatBuffer& srcBuffer);
	void CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 workGroupCount, const ndBrainFloatBuffer& srcBuffer);
	void CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, const ndBrainFloatBuffer& srcBuffer);

	protected:
	ndSharedPtr<ndBrainVector> m_buffer;

	friend class ndBrainCpuContext;
	friend class ndBrainGpuContext;
};

#endif