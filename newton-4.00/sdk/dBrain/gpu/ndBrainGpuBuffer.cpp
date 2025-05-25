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

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuScopeMapBuffer.h"

#if defined (D_USE_VULKAN_SDK)

ndInt64 ndBrainGpuBuffer::m_memoryUsed = 0;

ndBrainGpuBuffer::ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags, ndDeviceBufferType deviceType)
	:ndContainersFreeListAlloc<ndBrainGpuBuffer>()
	,m_context(context)
	,m_sizeInBytes(sizeInByte)
	,m_bufferType(bufferTypeFlags)
	,m_deviceBufferType(deviceType)
{
	VkDevice const device = (VkDevice)m_context->GetDevice();
	VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();

	ndAssert((deviceType == ndCpuMappable) || (deviceType == ndGpuOnly) && (bufferTypeFlags == ndStorageData));

	VkBufferCreateInfo bufferCreateInfo = {};
	bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferCreateInfo.size = VkDeviceSize(sizeInByte); 
	//bufferCreateInfo.usage = (bufferTypeFlags == ndStorageData) ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER : VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	bufferCreateInfo.usage = (bufferTypeFlags == ndStorageData) ? VK_BUFFER_USAGE_STORAGE_BUFFER_BIT : VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
	bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; 
	ndBrainGpuContext::CheckResultVulkan(vkCreateBuffer(device, &bufferCreateInfo, allocators, &m_buffer));

	VkMemoryRequirements memoryRequirements;
	vkGetBufferMemoryRequirements(device, m_buffer, &memoryRequirements);

	VkMemoryAllocateInfo allocateInfo = {};
	allocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocateInfo.allocationSize = memoryRequirements.size; // specify required memory.

	//VkMemoryPropertyFlags memoryProperty = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
	//VkMemoryPropertyFlags memoryProperty = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
	VkMemoryPropertyFlags memoryProperty = (deviceType == ndCpuMappable) ? VkMemoryPropertyFlags(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
	allocateInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, memoryProperty);

	ndBrainGpuContext::CheckResultVulkan(vkAllocateMemory(device, &allocateInfo, allocators, &m_bufferMemory)); 

	// Now associate that allocated memory with the buffer. With that, the buffer is backed by actual memory. 
	ndBrainGpuContext::CheckResultVulkan(vkBindBufferMemory(device, m_buffer, m_bufferMemory, 0));

	m_memoryUsed += m_sizeInBytes;
	//ndTrace(("buffers memory %d (megBytes)\n", ndInt32(m_memoryUsed >> 20)));
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
	m_memoryUsed -= m_sizeInBytes;
	VkDevice device = (VkDevice)m_context->GetDevice();
	VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();
	vkFreeMemory(device, m_bufferMemory, allocators);
	vkDestroyBuffer(device, m_buffer, allocators);
	//ndTrace(("buffers memory %d (megBytes)\n", ndInt32(m_memoryUsed >> 20)));
}

ndStorageBufferType ndBrainGpuBuffer::GetType() const
{
	return m_bufferType;
}

uint32_t ndBrainGpuBuffer::FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties)
{
	VkPhysicalDeviceMemoryProperties memoryProperties;

	VkPhysicalDevice physicalDevice = (VkPhysicalDevice)m_context->GetPhysicalDevice();
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memoryProperties);

	//How does this search work?
	//See the documentation of VkPhysicalDeviceMemoryProperties for a detailed description.
	for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
	{
		if ((memoryTypeBits & (1 << i)) && ((memoryProperties.memoryTypes[i].propertyFlags & properties) == properties))
		{
			return i;
		}
	}
	ndAssert(0);
	return uint32_t(-1);
}

VkBuffer ndBrainGpuBuffer::GetBuffer() const
{
	return m_buffer;
}

size_t ndBrainGpuBuffer::SizeInBytes() const
{
	return m_sizeInBytes;
}
#endif


