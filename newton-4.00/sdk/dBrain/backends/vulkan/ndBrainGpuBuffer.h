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
#ifndef __ND_BRAIN_GPU_VULKAN_BUFFER_H__
#define __ND_BRAIN_GPU_VULKAN_BUFFER_H__

#include "ndBrainStdafx.h"

class ndBrainVector;
class ndBrainGpuContext;
class ndBrainGpuBuffer;

#define ND_GPU_BUFFER_ALIGNMENT	32

#if 0
class ndScopeMapBuffer
{
	public:
	ndScopeMapBuffer(ndBrainGpuBuffer& buffer);
	~ndScopeMapBuffer();
	void* GetPointer() const;

	private:
	void* m_mappedMemory;
	ndBrainGpuBuffer* m_buffer;
};

class ndBrainGpuBuffer : public ndClassAlloc
{
	public:
	virtual ~ndBrainGpuBuffer();
	VkBuffer GetBuffer() const;
	ndInt32 SizeInBytes() const;
	virtual VkDescriptorType GetType() const;
	
	protected:
	ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt32 sizeInByte, ndUnsigned32 bufferTypeFlags);
	uint32_t FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties);

	VkBuffer m_buffer;
	VkDeviceMemory m_bufferMemory;
	ndBrainGpuContext* m_context;
	ndInt32 m_sizeInBytes;
	friend class ndScopeMapBuffer;
};

// **************************************************************************
// 
// **************************************************************************
class ndBrainGpuIntegerBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, ndInt32 size);
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input);

	VkDescriptorType GetType() const;
	void UnloadData(ndArray<ndInt32>& output);
	void LoadData(const ndArray<ndInt32>& input);
};

class ndBrainGpuFloatBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt32 size);
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input);

	VkDescriptorType GetType() const;
	void UnloadData(ndBrainVector& output);
	void LoadData(const ndBrainVector& input);
};

class ndBrainGpuUniformBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes);
	ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data);

	VkDescriptorType GetType() const;
	void LoadData(ndInt32 sizeInBytes, const void* const data);
};
#endif

#endif