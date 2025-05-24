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

#define ND_GPU_BUFFER_ALIGNMENT	32


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

#if !defined (D_USE_VULKAN_SDK)

class ndBrainGpuBuffer : public ndClassAlloc
{
	protected:
	ndBrainGpuBuffer(ndBrainGpuContext* const, ndInt64, ndStorageBufferType, ndDeviceBufferType):m_sizeInBytes(0){}

	public:
	virtual ~ndBrainGpuBuffer() {}
	void* GetBuffer() const { return nullptr;}
	ndInt32 SizeInBytes() const { return 0; }

	virtual void LoadData(ndInt32 sizeInBytes, const void* const inputData) = 0;
	virtual void UnloadData(ndInt32 sizeInBytes, void* const outputData) const = 0;

	protected:
	ndInt32 m_sizeInBytes;
	friend class ndScopeMapBuffer;
};

#else

class ndBrainGpuBuffer: public ndContainersFreeListAlloc<ndBrainGpuBuffer>
{
	public:
	virtual ~ndBrainGpuBuffer();
	VkBuffer GetBuffer() const;
	ndInt64 SizeInBytes() const;
	ndStorageBufferType GetType() const;

	virtual void LoadData(ndInt32 sizeInBytes, const void* const inputData) = 0;
	virtual void UnloadData(ndInt32 sizeInBytes, void* const outputData) const = 0;

	protected:
	ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags, ndDeviceBufferType deviceType);
	uint32_t FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties);

	VkBuffer m_buffer;
	VkDeviceMemory m_bufferMemory;
	ndBrainGpuContext* m_context;
	ndInt64 m_sizeInBytes;
	ndStorageBufferType m_bufferType;
	ndDeviceBufferType m_deviceBufferType;

	static ndInt64 m_memoryUsed;
	friend class ndScopeMapBuffer;
};
#endif

#endif