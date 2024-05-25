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
#include <vulkan/vulkan.h>
#include "ndBrainFloat4.h"
#include "ndBrainVector.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"

#define VK_CHECK_RESULT(f) 																				\
{																										\
	VkResult res = (f);																					\
	if (res != VK_SUCCESS)																				\
	{																									\
		ndTrace(("Fatal : VkResult is %d in %s at line %d\n", res,  __FILE__, __LINE__));							\
		ndAssert(res == VK_SUCCESS);																		\
	}																									\
}

class ndBrainGpuBufferBase::ndBrainGpuBufferBase::ndImplementation : public ndClassAlloc
{
	public:
	ndImplementation(ndBrainGpuContext* const context, ndInt32 sizeInByte)
		:ndClassAlloc()
		,m_context(context)
		,m_sizeInBytes(sizeInByte)
	{
		VkDevice const device = (VkDevice)m_context->GetDevice();
		VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();

		VkBufferCreateInfo bufferCreateInfo = {};
		bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferCreateInfo.size = VkDeviceSize(sizeInByte); // buffer size in bytes. 
		bufferCreateInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT; // buffer is used as a storage buffer.
		bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; // buffer is exclusive to a single queue family at a time. 
		
		VK_CHECK_RESULT(vkCreateBuffer(device, &bufferCreateInfo, allocators, &m_buffer));
		
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

		allocateInfo.memoryTypeIndex = FindMemoryType(
		    memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
		
		
		VK_CHECK_RESULT(vkAllocateMemory(device, &allocateInfo, allocators, &m_bufferMemory)); // allocate memory on device.
		
		// Now associate that allocated memory with the buffer. With that, the buffer is backed by actual memory. 
		VK_CHECK_RESULT(vkBindBufferMemory(device, m_buffer, m_bufferMemory, 0));
	}

	~ndImplementation()
	{
		VkDevice device = (VkDevice)m_context->GetDevice();
		VkAllocationCallbacks* const allocators = (VkAllocationCallbacks*)m_context->GetAllocator();
		vkFreeMemory(device, m_bufferMemory, allocators);
		vkDestroyBuffer(device, m_buffer, allocators);
	}

	uint32_t FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties)
	{
		VkPhysicalDeviceMemoryProperties memoryProperties;

		VkPhysicalDevice physicalDevice = (VkPhysicalDevice)m_context->GetPhysicalDevice();
		vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memoryProperties);

		//How does this search work?
		//See the documentation of VkPhysicalDeviceMemoryProperties for a detailed description.
		for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
		{
			if ((memoryTypeBits & (1 << i)) && ((memoryProperties.memoryTypes[i].propertyFlags & properties) == properties))
				return i;
		}
		return uint32_t (-1);
	}

	VkBuffer m_buffer;
	VkDeviceMemory m_bufferMemory;
	ndBrainGpuContext* m_context;
	ndInt32 m_sizeInBytes;
};

ndScopeMapBuffer::ndScopeMapBuffer(ndBrainGpuBufferBase& buffer)
	:m_buffer(&buffer)
{
	VkDevice const device = (VkDevice)m_buffer->m_buffer->m_context->GetDevice();
	
	// Map the buffer memory, so that we can read from it on the CPU.
	vkMapMemory(device, m_buffer->m_buffer->m_bufferMemory, 0, ndUnsigned32 (m_buffer->m_buffer->m_sizeInBytes), 0, &m_mappedMemory);
}

ndScopeMapBuffer::~ndScopeMapBuffer()
{
	VkDevice const device = (VkDevice)m_buffer->m_buffer->m_context->GetDevice();
	vkUnmapMemory(device, m_buffer->m_buffer->m_bufferMemory);
}

void* ndScopeMapBuffer::GetPointer() const
{
	return m_mappedMemory;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuBufferBase::ndBrainGpuBufferBase(ndBrainGpuContext* const context, ndInt32 sizeInByte)
	:ndClassAlloc()
	,m_buffer(new ndImplementation(context, sizeInByte))
{
}

ndBrainGpuBufferBase::~ndBrainGpuBufferBase()
{
	delete m_buffer;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt32 size)
	:ndBrainGpuBufferBase(context, size* ndInt32(sizeof(ndReal)))
{
}

void ndBrainGpuFloatBuffer::LoadData(const ndBrainVector& input)
{
	ndScopeMapBuffer mapBuffer(*this);
	const ndInt32 size = m_buffer->m_sizeInBytes / ndInt32(sizeof(ndReal));
	ndAssert(size == input.GetCount());
	ndReal* const dst = (ndReal*)mapBuffer.GetPointer();
	for (ndInt32 i = 0; i < size; ++i)
	{
		dst[i] = ndReal(input[i]);
	}
}

void ndBrainGpuFloatBuffer::UnloadData(ndBrainVector& output)
{
	ndScopeMapBuffer mapBuffer(*this);
	const ndInt32 size = m_buffer->m_sizeInBytes / ndInt32(sizeof(ndReal));
	const ndReal* const src = (ndReal*)mapBuffer.GetPointer();
	output.SetCount(size);
	for (ndInt32 i = 0; i < size; ++i)
	{
		output[i] = ndBrainFloat(src[i]);
	}
}