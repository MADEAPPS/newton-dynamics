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

#include "ndBrainVulkanStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainVulkanBuffer.h"
#include "ndBrainVulkanContext.h"

#if 1
//ndScopeMapBuffer::ndScopeMapBuffer(ndBrainVulkanBuffer& buffer)
//	:m_buffer(&buffer)
//{
//	VkDevice const device = (VkDevice)m_buffer->m_context->GetDevice();
//
//	// Map the buffer memory, so that we can read from it on the CPU.
//	vkMapMemory(device, m_buffer->m_bufferMemory, 0, ndUnsigned32(m_buffer->m_sizeInBytes), 0, &m_mappedMemory);
//}
//
//ndScopeMapBuffer::~ndScopeMapBuffer()
//{
//	VkDevice const device = (VkDevice)m_buffer->m_context->GetDevice();
//	vkUnmapMemory(device, m_buffer->m_bufferMemory);
//}
//
//void* ndScopeMapBuffer::GetPointer() const
//{
//	return m_mappedMemory;
//}

ndBrainVulkanBuffer::ndBrainVulkanBuffer(ndBrainGpuContext* const context, ndInt32 sizeInByte, ndUnsigned32 bufferTypeFlags)
	:ndClassAlloc()
	,m_context(context)
	,m_sizeInBytes(sizeInByte)
{
	ndAssert(0);
	//VkDevice const device = (VkDevice)m_context->GetDevice();
	//VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();
	//
	//VkBufferCreateInfo bufferCreateInfo = {};
	//bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	//bufferCreateInfo.size = VkDeviceSize(sizeInByte); 
	//bufferCreateInfo.usage = VkBufferUsageFlagBits(bufferTypeFlags);
	//bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; 
	//ndBrainGpuContext::CheckResultVulkan(vkCreateBuffer(device, &bufferCreateInfo, allocators, &m_buffer));
	//
	//VkMemoryRequirements memoryRequirements;
	//vkGetBufferMemoryRequirements(device, m_buffer, &memoryRequirements);
	//
	//VkMemoryAllocateInfo allocateInfo = {};
	//allocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	//allocateInfo.allocationSize = memoryRequirements.size; // specify required memory.
	//
	////allocateInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
	//VkMemoryPropertyFlags memoryProperty = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
	//allocateInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, memoryProperty);
	//
	//ndBrainGpuContext::CheckResultVulkan(vkAllocateMemory(device, &allocateInfo, allocators, &m_bufferMemory)); 
	//
	//// Now associate that allocated memory with the buffer. With that, the buffer is backed by actual memory. 
	//ndBrainGpuContext::CheckResultVulkan(vkBindBufferMemory(device, m_buffer, m_bufferMemory, 0));
}

ndBrainVulkanBuffer::~ndBrainVulkanBuffer()
{
	ndAssert(0);
	//VkDevice device = (VkDevice)m_context->GetDevice();
	//VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();
	//vkFreeMemory(device, m_bufferMemory, allocators);
	//vkDestroyBuffer(device, m_buffer, allocators);
}

VkDescriptorType ndBrainVulkanBuffer::GetType() const
{
	ndAssert(0);
	return VK_DESCRIPTOR_TYPE_MAX_ENUM;
}

uint32_t ndBrainVulkanBuffer::FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties)
{
	ndAssert(0);
	//VkPhysicalDeviceMemoryProperties memoryProperties;
	//
	//VkPhysicalDevice physicalDevice = (VkPhysicalDevice)m_context->GetPhysicalDevice();
	//vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memoryProperties);
	//
	////How does this search work?
	////See the documentation of VkPhysicalDeviceMemoryProperties for a detailed description.
	//for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
	//{
	//	if ((memoryTypeBits & (1 << i)) && ((memoryProperties.memoryTypes[i].propertyFlags & properties) == properties))
	//	{
	//		return i;
	//	}
	//}
	return uint32_t(-1);
}

VkBuffer ndBrainVulkanBuffer::GetBuffer() const
{
	return m_buffer;
}

ndInt32 ndBrainVulkanBuffer::SizeInBytes() const
{
	return m_sizeInBytes;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainVulkanFloatBuffer::ndBrainVulkanFloatBuffer(ndBrainGpuContext* const context, ndInt32 size)
	:ndBrainVulkanBuffer(context, size * ndInt32(sizeof(ndReal)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
}

ndBrainVulkanFloatBuffer::ndBrainVulkanFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input)
	:ndBrainVulkanBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
	LoadData(input);
}

VkDescriptorType ndBrainVulkanFloatBuffer::GetType() const
{
	return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainVulkanIntegerBuffer::ndBrainVulkanIntegerBuffer(ndBrainGpuContext* const context, ndInt32 size)
	:ndBrainVulkanBuffer(context, size* ndInt32(sizeof(ndInt32)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
}

ndBrainVulkanIntegerBuffer::ndBrainVulkanIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input)
	:ndBrainVulkanBuffer(context, input.GetCount()* ndInt32(sizeof(ndInt32)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
	LoadData(input);
}

VkDescriptorType ndBrainVulkanIntegerBuffer::GetType() const
{
	return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainVulkanUniformBuffer::ndBrainVulkanUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes)
	:ndBrainVulkanBuffer(context, sizeInBytes, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT)
{
}

ndBrainVulkanUniformBuffer::ndBrainVulkanUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainVulkanBuffer(context, sizeInBytes, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT)
{
	LoadData(sizeInBytes, data);
}

void ndBrainVulkanFloatBuffer::LoadData(const ndBrainVector& input)
{
	ndAssert(0);
	//ndScopeMapBuffer mapBuffer(*this);
	//ndReal* const dst = (ndReal*)mapBuffer.GetPointer();
	//if (dst)
	//{
	//	const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndReal));
	//	ndAssert(size == input.GetCount());
	//
	//	for (ndInt32 i = 0; i < size; ++i)
	//	{
	//		dst[i] = ndReal(input[i]);
	//	}
	//}
}

void ndBrainVulkanFloatBuffer::UnloadData(ndBrainVector& output)
{
	ndAssert(0);
	//ndScopeMapBuffer mapBuffer(*this);
	//const ndReal* const src = (ndReal*)mapBuffer.GetPointer();
	//if (src)
	//{
	//	const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndReal));
	//	output.SetCount(size);
	//	const ndBrainMemVector srcData(src, size);
	//	output.Set(srcData);
	//}
}

void ndBrainVulkanIntegerBuffer::LoadData(const ndArray<ndInt32>& input)
{
	ndAssert(0);
	//ndScopeMapBuffer mapBuffer(*this);
	//ndInt32* const dst = (ndInt32*)mapBuffer.GetPointer();
	//if (dst)
	//{
	//	const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndInt32));
	//	ndAssert(size == input.GetCount());
	//
	//	for (ndInt32 i = 0; i < size; ++i)
	//	{
	//		dst[i] = input[i];
	//	}
	//}
}

void ndBrainVulkanIntegerBuffer::UnloadData(ndArray<ndInt32>& output)
{
	ndAssert(0);
	//ndScopeMapBuffer mapBuffer(*this);
	//const ndInt32* const src = (ndInt32*)mapBuffer.GetPointer();
	//if (src)
	//{
	//	const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndInt32));
	//	output.SetCount(size);
	//	for (ndInt32 i = 0; i < size; ++i)
	//	{
	//		output[i] = src[i];
	//	}
	//}
}

void ndBrainVulkanUniformBuffer::LoadData(ndInt32 sizeInBytes, const void* const data)
{
	ndAssert(0);
	//ndScopeMapBuffer mapBuffer(*this);
	//ndUnsigned8* const dst = (ndUnsigned8*)mapBuffer.GetPointer();
	//if (dst)
	//{
	//	ndMemCpy(dst, (ndUnsigned8*)data, sizeInBytes);
	//}
}

VkDescriptorType ndBrainVulkanUniformBuffer::GetType() const
{
	return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
}
#endif