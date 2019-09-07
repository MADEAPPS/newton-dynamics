/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _DG_VULCAN_VECTOR_H_
#define _DG_VULCAN_VECTOR_H_


#include "dgPhysicsStdafx.h"


template<class T>
class dgArrayVector
{
	public:
	dgArrayVector()
		:m_buffer(0)
	{
	}

	~dgArrayVector()
	{
	}

	void Alloc(dgVulkanContext& context, dgInt32 size)
	{
		dgAssert((size % DG_GPU_WORKGROUP_SIZE) == 0);

		VkBufferCreateInfo bufferInfo;
		Clear(&bufferInfo);
		bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferInfo.size = size * sizeof(T);
		bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		bufferInfo.flags = 0;

		VkResult result = VK_SUCCESS;
		result = vkCreateBuffer(context.m_device, &bufferInfo, &context.m_allocator, &m_buffer);
		dgAssert(result == VK_SUCCESS);

		VkMemoryRequirements memReqs;
		vkGetBufferMemoryRequirements(context.m_device, m_buffer, &memReqs);
		dgAssert(memReqs.size == bufferInfo.size);

		uint32_t memoryTypeIndex;
		//VkFlags memoryProperty = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
		VkFlags memoryProperty = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
		VkPhysicalDeviceMemoryProperties& memoryProperties = context.m_memory_properties;
		for (memoryTypeIndex = 0; memoryTypeIndex < memoryProperties.memoryTypeCount; ++memoryTypeIndex) {
			if (((memoryProperties.memoryTypes[memoryTypeIndex].propertyFlags & memoryProperty) == memoryProperty) &&
				((memReqs.memoryTypeBits >> memoryTypeIndex) & 1)) {
				break;
			}
		}
		dgAssert(memoryTypeIndex < memoryProperties.memoryTypeCount);

		VkMemoryAllocateInfo memInfo;
		Clear(&memInfo);
		memInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		memInfo.allocationSize = bufferInfo.size;
		memInfo.memoryTypeIndex = memoryTypeIndex;

		result = vkAllocateMemory(context.m_device, &memInfo, &context.m_allocator, &m_memory);
		dgAssert(result == VK_SUCCESS);

		result = vkBindBufferMemory(context.m_device, m_buffer, m_memory, 0);
		dgAssert(result == VK_SUCCESS);
	}

	void Free(dgVulkanContext& context)
	{
		if (m_buffer) {
			vkFreeMemory(context.m_device, m_memory, &context.m_allocator);
			vkDestroyBuffer(context.m_device, m_buffer, &context.m_allocator);
		}
	}

	T* Lock(dgVulkanContext& context, dgInt32 size)
	{
		void* ptr;
		VkResult result = VK_SUCCESS;
		result = vkMapMemory(context.m_device, m_memory, 0, size * sizeof (T), 0, &ptr);
		dgAssert(result == VK_SUCCESS);
		dgAssert((((dgUnsigned64)ptr) & 0xf) == 0);
		return (T*)ptr;
	}

	void Unlock(dgVulkanContext& context)
	{
		vkUnmapMemory(context.m_device, m_memory);
	}

	VkBuffer m_buffer;
	VkDeviceMemory m_memory;
};




#endif
