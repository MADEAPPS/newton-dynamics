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
#ifndef __ND_BRAIN_GPU_UNIFORM_BUFFER_H__
#define __ND_BRAIN_GPU_UNIFORM_BUFFER_H__

#include "ndBrainStdafx.h"
#include "ndBrainBuffer.h"
#include "ndBrainGpuContext.h"

class ndBrainGpuUniformBuffer : public ndBrainBuffer
{
	public:
	ndBrainGpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, bool memoryMapped = false);
	ndBrainGpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data, bool memoryMapped = false);

	virtual void BrainVectorToDevice(const ndBrainVector& vector) override;
	virtual void BrainVectorFromDevice(ndBrainVector& vector) const override;

	virtual void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory) override;
	virtual void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const override;

	virtual void CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer) override;
	virtual void CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer) override;

	void* GetData();

	protected:
	virtual void LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData) override;
	virtual void UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const override;

	ndFixSizeArray<char, 256> m_data;
	friend class ndBrainGpuFloatBuffer;
};

#endif