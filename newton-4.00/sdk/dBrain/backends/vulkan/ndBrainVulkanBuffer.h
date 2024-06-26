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
class ndBrainVulkanBuffer;

#define ND_GPU_BUFFER_ALIGNMENT	32

#if 1
//class ndScopeMapBuffer
//{
//	public:
//	ndScopeMapBuffer(ndBrainVulkanBuffer& buffer);
//	~ndScopeMapBuffer();
//	void* GetPointer() const;
//
//	private:
//	void* m_mappedMemory;
//	ndBrainVulkanBuffer* m_buffer;
//};

class ndBrainVulkanBuffer : public ndClassAlloc
{
	public:
	virtual ~ndBrainVulkanBuffer();
	VkBuffer GetBuffer() const;
	ndInt32 SizeInBytes() const;
	virtual VkDescriptorType GetType() const;
	
	protected:
	ndBrainVulkanBuffer(ndBrainGpuContext* const context, ndInt32 sizeInByte, ndUnsigned32 bufferTypeFlags);
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
class ndBrainVulkanIntegerBuffer : public ndBrainVulkanBuffer
{
	public:
	ndBrainVulkanIntegerBuffer(ndBrainGpuContext* const context, ndInt32 size);
	ndBrainVulkanIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input);

	VkDescriptorType GetType() const;
	void UnloadData(ndArray<ndInt32>& output);
	void LoadData(const ndArray<ndInt32>& input);
};

class ndBrainVulkanFloatBuffer : public ndBrainVulkanBuffer
{
	public:
	ndBrainVulkanFloatBuffer(ndBrainGpuContext* const context, ndInt32 size);
	ndBrainVulkanFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input);

	VkDescriptorType GetType() const;
	void UnloadData(ndBrainVector& output);
	void LoadData(const ndBrainVector& input);
};

class ndBrainVulkanUniformBuffer : public ndBrainVulkanBuffer
{
	public:
	ndBrainVulkanUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes);
	ndBrainVulkanUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data);

	VkDescriptorType GetType() const;
	void LoadData(ndInt32 sizeInBytes, const void* const data);
};
#endif

#endif