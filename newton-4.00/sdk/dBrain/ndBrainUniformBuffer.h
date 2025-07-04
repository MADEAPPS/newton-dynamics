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
	ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, bool memoryMapped = false);
	ndBrainUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data, bool memoryMapped = false);

	virtual void* GetCpuPtr() override;
	virtual void* GetCpuPtr() const override;
	
	void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData);
	void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const;

	protected:
	ndSharedPtr<ndFixSizeArray<ndUnsigned32, 256>> m_data;

	friend class ndBrainCpuContext;
};

#endif