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

#if defined (D_USE_VULKAN_SDK)

ndScopeMapBuffer::ndScopeMapBuffer(ndBrainGpuBuffer& buffer)
	:m_buffer(&buffer)
{
	VkDevice const device = (VkDevice)m_buffer->m_context->GetDevice();

	// Map the buffer memory, so that we can read from it on the CPU.
	vkMapMemory(device, m_buffer->m_bufferMemory, 0, ndUnsigned32(m_buffer->m_sizeInBytes), 0, &m_mappedMemory);
}

ndScopeMapBuffer::~ndScopeMapBuffer()
{
	VkDevice const device = (VkDevice)m_buffer->m_context->GetDevice();
	vkUnmapMemory(device, m_buffer->m_bufferMemory);
}

void* ndScopeMapBuffer::GetPointer() const
{
	return m_mappedMemory;
}


ndBrainGpuBuffer::ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt32 sizeInByte, ndUnsigned32 bufferTypeFlags)
	:ndClassAlloc()
	,m_context(context)
	,m_sizeInBytes(sizeInByte)
{
	VkDevice const device = (VkDevice)m_context->GetDevice();
	VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();

	VkBufferCreateInfo bufferCreateInfo = {};
	bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferCreateInfo.size = VkDeviceSize(sizeInByte); 
	bufferCreateInfo.usage = VkBufferUsageFlagBits(bufferTypeFlags);
	bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; 
	ndBrainGpuContext::CheckResultVulkan(vkCreateBuffer(device, &bufferCreateInfo, allocators, &m_buffer));

	VkMemoryRequirements memoryRequirements;
	vkGetBufferMemoryRequirements(device, m_buffer, &memoryRequirements);

	VkMemoryAllocateInfo allocateInfo = {};
	allocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocateInfo.allocationSize = memoryRequirements.size; // specify required memory.

	//There are several types of memory that can be allocated, and we must choose a memory type that:
	//1) Satisfies the memory requirements(memoryRequirements.memoryTypeBits).
	//2) Satisfy our own usage requirements. We want to be able to read the buffer memory from the GPU to the CPU
	//   with vkMapMemory, so we set VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT.
	//Also, by setting VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, memory written by the device(GPU) will be easily
	//visible to the host(CPU), without having to call any extra flushing commands. So mainly for convenience, we set
	//this flag.
	allocateInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);

	ndBrainGpuContext::CheckResultVulkan(vkAllocateMemory(device, &allocateInfo, allocators, &m_bufferMemory)); 

	// Now associate that allocated memory with the buffer. With that, the buffer is backed by actual memory. 
	ndBrainGpuContext::CheckResultVulkan(vkBindBufferMemory(device, m_buffer, m_bufferMemory, 0));
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
	VkDevice device = (VkDevice)m_context->GetDevice();
	VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();
	vkFreeMemory(device, m_bufferMemory, allocators);
	vkDestroyBuffer(device, m_buffer, allocators);
}

VkDescriptorType ndBrainGpuBuffer::GetType() const
{
	ndAssert(0);
	return VK_DESCRIPTOR_TYPE_MAX_ENUM;
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
	return uint32_t(-1);
}

VkBuffer ndBrainGpuBuffer::GetBuffer() const
{
	return m_buffer;
}

ndInt32 ndBrainGpuBuffer::SizeInBytes() const
{
	return m_sizeInBytes;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt32 size)
	:ndBrainGpuBuffer(context, size* ndInt32(sizeof(ndReal)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input)
	:ndBrainGpuBuffer(context, input.GetCount()* ndInt32(sizeof(ndReal)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
	LoadData(input);
}

VkDescriptorType ndBrainGpuFloatBuffer::GetType() const
{
	return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, ndInt32 size)
	:ndBrainGpuBuffer(context, size* ndInt32(sizeof(ndInt32)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
}

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input)
	:ndBrainGpuBuffer(context, input.GetCount()* ndInt32(sizeof(ndInt32)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)
{
	LoadData(input);
}

VkDescriptorType ndBrainGpuIntegerBuffer::GetType() const
{
	return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainGpuBuffer(context, sizeInBytes, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT)
{
	LoadData(sizeInBytes, data);
}

void ndBrainGpuFloatBuffer::LoadData(const ndBrainVector& input)
{
	ndScopeMapBuffer mapBuffer(*this);
	ndReal* const dst = (ndReal*)mapBuffer.GetPointer();
	if (dst)
	{
		const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndReal));
		ndAssert(size == input.GetCount());

		for (ndInt32 i = 0; i < size; ++i)
		{
			dst[i] = ndReal(input[i]);
		}
	}
}

void ndBrainGpuFloatBuffer::UnloadData(ndBrainVector& output)
{
	ndScopeMapBuffer mapBuffer(*this);
	const ndReal* const src = (ndReal*)mapBuffer.GetPointer();
	if (src)
	{
		const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndReal));
		output.SetCount(size);
		for (ndInt32 i = 0; i < size; ++i)
		{
			output[i] = ndBrainFloat(src[i]);
		}
	}
}

void ndBrainGpuIntegerBuffer::LoadData(const ndArray<ndInt32>& input)
{
	ndScopeMapBuffer mapBuffer(*this);
	ndInt32* const dst = (ndInt32*)mapBuffer.GetPointer();
	if (dst)
	{
		const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndInt32));
		ndAssert(size == input.GetCount());

		for (ndInt32 i = 0; i < size; ++i)
		{
			dst[i] = input[i];
		}
	}
}

void ndBrainGpuIntegerBuffer::UnloadData(ndArray<ndInt32>& output)
{
	ndScopeMapBuffer mapBuffer(*this);
	const ndInt32* const src = (ndInt32*)mapBuffer.GetPointer();
	if (src)
	{
		const ndInt32 size = m_sizeInBytes / ndInt32(sizeof(ndInt32));
		output.SetCount(size);
		for (ndInt32 i = 0; i < size; ++i)
		{
			output[i] = src[i];
		}
	}
}

void ndBrainGpuUniformBuffer::LoadData(ndInt32 sizeInBytes, const void* const data)
{
	ndScopeMapBuffer mapBuffer(*this);
	ndUnsigned8* const dst = (ndUnsigned8*)mapBuffer.GetPointer();
	if (dst)
	{
		ndMemCpy(dst, (ndUnsigned8*)data, sizeInBytes);
	}
}

VkDescriptorType ndBrainGpuUniformBuffer::GetType() const
{
	return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
}

#endif


