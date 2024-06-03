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

ndScopeMapBuffer::ndScopeMapBuffer(ndBrainGpuBufferBase& buffer)
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


ndBrainGpuBufferBase::ndBrainGpuBufferBase(ndBrainGpuContext* const context, ndInt32 sizeInByte)
	:ndClassAlloc()
	,m_context(context)
	,m_sizeInBytes(sizeInByte)
{
	ndAssert(0);
	VkDevice const device = (VkDevice)m_context->GetDevice();
	VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();

	VkBufferCreateInfo bufferCreateInfo = {};
	bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferCreateInfo.size = VkDeviceSize(sizeInByte); 
	bufferCreateInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT; 
	bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; 

	ndBrainGpuContext::CheckResultVulkan(vkCreateBuffer(device, &bufferCreateInfo, allocators, &m_buffer));

	//But the buffer doesn't allocate memory for itself, so we must do that manually.
	//First, we find the memory requirements for the buffer.
	VkMemoryRequirements memoryRequirements;
	vkGetBufferMemoryRequirements(device, m_buffer, &memoryRequirements);

	//Now use obtained memory requirements info to allocate the memory for the buffer.
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

ndBrainGpuBufferBase::~ndBrainGpuBufferBase()
{
	VkDevice device = (VkDevice)m_context->GetDevice();
	VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();
	vkFreeMemory(device, m_bufferMemory, allocators);
	vkDestroyBuffer(device, m_buffer, allocators);
}

uint32_t ndBrainGpuBufferBase::FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties)
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

VkBuffer ndBrainGpuBufferBase::GetBuffer() const
{
	return m_buffer;
}

ndInt32 ndBrainGpuBufferBase::SizeInBytes() const
{
	return m_sizeInBytes;
}


#endif


//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt32 size)
	:ndBrainGpuBufferBase(context, size * ndInt32(sizeof(ndReal)))
{
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input)
	:ndBrainGpuBufferBase(context, input.GetCount() * ndInt32(sizeof(ndReal)))
{
	LoadData(input);
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

//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, ndInt32 size)
	:ndBrainGpuBufferBase(context, size * ndInt32(sizeof(ndInt32)))
{
}

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input)
	:ndBrainGpuBufferBase(context, input.GetCount() * ndInt32(sizeof(ndInt32)))
{
	LoadData(input);
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
