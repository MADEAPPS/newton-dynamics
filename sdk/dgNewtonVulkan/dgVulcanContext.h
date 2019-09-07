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

#ifndef _DG_VULCAN_CONTEXT_H_
#define _DG_VULCAN_CONTEXT_H_


#include "dgPhysicsStdafx.h"
#include "dgVulcanShader.h"

#define DG_GPU_WORKGROUP_SIZE		256 
#define DG_GPU_BODY_INITIAL_COUNT	4096


class dgVulkanContext 
{
	public:
	dgVulkanContext();

	void InitDevice(VkInstance instance);
	void DestroyDevice();

	static VkInstance CreateInstance(dgMemoryAllocator* const allocator);
	static void VKAPI_PTR vkFreeFunction(void* pUserData, void* pMemory);
	static void* VKAPI_PTR vkAllocationFunction(void* pUserData, size_t size, size_t alignment, VkSystemAllocationScope allocationScope);
	static void* VKAPI_PTR vkReallocationFunction(void* pUserData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope);
	static void VKAPI_PTR vkInternalAllocationNotification(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope);
	static void VKAPI_PTR vkInternalFreeNotification(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope);

	static dgInt32 FindLayer(const char* const name, dgInt32 count, VkLayerProperties* const layers);

	dgInt32 m_computeQueueIndex;
	VkQueue m_queue;
	VkDevice m_device;
	VkInstance m_instance;
	VkPhysicalDevice m_gpu;
	VkPhysicalDeviceProperties m_gpu_props;
	VkPhysicalDeviceMemoryProperties m_memory_properties;

	VkPipelineCache m_pipeLineCache;
	dgVulkanShaderInfo m_initBody;

	static dgInt32 m_totalMemory;
	static const char* m_validationLayer0;
	static const char* m_validationLayer1;
	static VkAllocationCallbacks m_allocator;
};



#endif
