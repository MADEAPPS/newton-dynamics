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
#ifndef __ND_BRAIN_GPU_BUFFER_H__
#define __ND_BRAIN_GPU_BUFFER_H__

#include "ndBrainStdafx.h"

class ndBrainVector;
class ndBrainGpuContext;
class ndBrainGpuBuffer;

enum ndStorageBufferType
{
	ndStorageData,
	ndUniformData
};

enum ndDeviceBufferType
{
	ndGpuOnly,
	ndCpuMappable,
};

class ndBrainGpuBuffer : public ndClassAlloc
{
	protected:
	ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags, ndDeviceBufferType deviceType);

	public:
	virtual ~ndBrainGpuBuffer();

	size_t SizeInBytes() const { return m_sizeInBytes; }
	virtual void LoadData(size_t sizeInBytes, const void* const inputData) = 0;
	virtual void UnloadData(size_t sizeInBytes, void* const outputData) const = 0;

	protected:
	cl::Buffer m_buffer;
	ndBrainGpuContext* m_context;
	size_t m_sizeInBytes;
	friend class ndBrainGpuContext;
	friend class ndBrainGpuCommand;
};

#endif