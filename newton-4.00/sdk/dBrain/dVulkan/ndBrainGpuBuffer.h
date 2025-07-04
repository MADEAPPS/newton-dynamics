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

//#define ND_GPU_BUFFER_ALIGNMENT	32


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
	public:
	virtual ~ndBrainGpuBuffer();
	VkBuffer GetBuffer() const;
	size_t SizeInBytes() const;
	ndStorageBufferType GetType() const;

	virtual void LoadData(size_t sizeInBytes, const void* const inputData) = 0;
	virtual void UnloadData(size_t sizeInBytes, void* const outputData) const = 0;

	protected:
	ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags, ndDeviceBufferType deviceType);
	uint32_t FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties);

	VkBuffer m_buffer;
	VkDeviceMemory m_bufferMemory;
	ndBrainGpuContext* m_context;
	size_t m_sizeInBytes;
	ndStorageBufferType m_bufferType;
	ndDeviceBufferType m_deviceBufferType;

	static ndInt64 m_memoryUsed;
	friend class ndScopeMapBuffer;
};


#endif