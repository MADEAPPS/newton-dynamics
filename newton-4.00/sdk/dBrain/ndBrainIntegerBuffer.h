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
#ifndef __ND_BRAIN_INTEGER_BUFFER_H__
#define __ND_BRAIN_INTEGER_BUFFER_H__

#include "ndBrainBuffer.h"

class ndBrainIntegerBuffer : public ndBrainBuffer
{
	public:
	ndBrainIntegerBuffer(ndBrainContext* const context, ndInt64 sizeInElements, bool memoryMapped = false);
	ndBrainIntegerBuffer(ndBrainContext* const context, ndInt64 numberOfElements, const ndUnsigned32* const indexArray, bool memoryMapped = false);

	virtual void* GetCpuPtr() override;
	virtual void* GetCpuPtr() const override;

	//virtual void BrainVectorToDevice(const ndBrainVector& vector) override;
	//virtual void BrainVectorFromDevice(ndBrainVector& vector) const override;
	//
	//virtual void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData) override;
	//virtual void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const override;
	//
	//virtual void CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer) override;
	//virtual void CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer) override;
	
	void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData);
	//protected:
	//virtual void LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData) override;
	//virtual void UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const override;

	ndSharedPtr<ndArray<ndUnsigned32>> m_indexArray;

	friend class ndBrainTrainer;
	//friend class ndBrainCpuContext;
};

#endif