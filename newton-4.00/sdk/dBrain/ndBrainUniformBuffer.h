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
#ifndef __ND_BRAIN_UNIFORM_BUFFER_H__
#define __ND_BRAIN_UNIFORM_BUFFER_H__

#include "ndBrainBuffer.h"

class ndBrainUniformBuffer : public ndBrainBuffer
{
	public:
	ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes);
	ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data);

	//virtual void BrainVectorToDevice(const ndBrainVector& vector) override;
	//virtual void BrainVectorFromDevice(ndBrainVector& vector) const override;
	//
	//
	//virtual void CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer) override;
	//virtual void CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer) override;
	
	void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData);
	void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const;

	//protected:
	//virtual void LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData) override;
	//virtual void UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const outputData) const override;

	protected:
	ndSharedPtr<ndFixSizeArray<ndUnsigned32, 256>> m_data;

	friend class ndBrainCpuContext;
};

#endif