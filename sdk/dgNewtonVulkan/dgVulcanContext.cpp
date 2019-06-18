/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgNewtonPluginStdafx.h"
#include "dgVulcanContext.h"

dgInt32 dgVulkanContext::m_totalMemory = 0;
VkAllocationCallbacks dgVulkanContext::m_allocator;
const char* dgVulkanContext::m_validationLayer0 = "VK_LAYER_KHRONOS_validation";
const char* dgVulkanContext::m_validationLayer1 = "VK_LAYER_LUNARG_standard_validation";


dgVulkanContext::dgVulkanContext()
{
	memset(this, 0, sizeof(dgVulkanContext));
}

VkInstance dgVulkanContext::CreateInstance(dgMemoryAllocator* const allocator)
{
	VkResult error = VK_SUCCESS;
	VkApplicationInfo app;
	Clear(&app);
	app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	app.pApplicationName = "NewtonVulkanSolver";
	app.applicationVersion = 100;
	app.pEngineName = "Newton-3.14";
	app.engineVersion = 314;
	app.apiVersion = VK_API_VERSION_1_0;

	uint32_t layersCount;
	const char* layersNames[64];
	error = vkEnumerateInstanceLayerProperties(&layersCount, NULL);
	dgAssert(error == VK_SUCCESS);

	if (layersCount) {
		VkLayerProperties layers[64];
		dgAssert(layersCount < sizeof(layers) / sizeof(layers[0]));
		error = vkEnumerateInstanceLayerProperties(&layersCount, layers);
		dgAssert(error == VK_SUCCESS);

		dgAssert(FindLayer(dgVulkanContext::m_validationLayer0, layersCount, layers) != -1);
		dgAssert(FindLayer(dgVulkanContext::m_validationLayer1, layersCount, layers) != -1);

		layersNames[0] = dgVulkanContext::m_validationLayer0;
		layersNames[1] = dgVulkanContext::m_validationLayer1;
		layersCount = 2;
	}


	VkInstanceCreateInfo inst_info;
	Clear(&inst_info);
	inst_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	inst_info.pApplicationInfo = &app;
	inst_info.enabledLayerCount = layersCount;
	inst_info.ppEnabledLayerNames = layersNames;
	inst_info.enabledExtensionCount = 0;
	inst_info.ppEnabledExtensionNames = NULL;

	m_allocator.pUserData = allocator;
	m_allocator.pfnFree = vkFreeFunction;
	m_allocator.pfnAllocation = vkAllocationFunction;
	m_allocator.pfnReallocation = vkReallocationFunction;
	m_allocator.pfnInternalAllocation = vkInternalAllocationNotification;
	m_allocator.pfnInternalFree = vkInternalFreeNotification;

	VkInstance instance;
	error = vkCreateInstance(&inst_info, &m_allocator, &instance);
	if (error != VK_SUCCESS) {
		return NULL;
	}

	uint32_t gpu_count = 0;
	error = vkEnumeratePhysicalDevices(instance, &gpu_count, NULL);
	if ((error != VK_SUCCESS) || (gpu_count == 0)) {
		vkDestroyInstance(instance, &m_allocator);
		return NULL;
	}
	return instance;
}

void dgVulkanContext::InitDevice(VkInstance instance)
{
	VkResult err = VK_SUCCESS;
	uint32_t gpu_count = 0;
	VkPhysicalDevice physical_gpus[8];

	err = vkEnumeratePhysicalDevices(instance, &gpu_count, NULL);
	dgAssert(err == VK_SUCCESS);

	err = vkEnumeratePhysicalDevices(instance, &gpu_count, &physical_gpus[0]);
	dgAssert(err == VK_SUCCESS);
	dgAssert(gpu_count >= 1);

	if (gpu_count > 1) {
		dgAssert(0);
	}

	VkPhysicalDeviceProperties gpu_props;
	Clear(&gpu_props);
	vkGetPhysicalDeviceProperties(physical_gpus[0], &gpu_props);

	uint32_t queue_family_count;
	vkGetPhysicalDeviceQueueFamilyProperties(physical_gpus[0], &queue_family_count, NULL);
	dgAssert(queue_family_count >= 1);
	dgAssert(queue_family_count < 16);

	VkQueueFamilyProperties queue_props[16];
	vkGetPhysicalDeviceQueueFamilyProperties(physical_gpus[0], &queue_family_count, queue_props);

	int computeQueueIndex = -1;
	for (uint32_t i = 0; i < queue_family_count; i++) {
		int hasComputeCapability = queue_props[i].queueFlags & VK_QUEUE_COMPUTE_BIT;
		if (hasComputeCapability) {
			if (computeQueueIndex == -1) {
				computeQueueIndex = i;
			}
		}
	}
	dgAssert(computeQueueIndex != -1);

	VkPhysicalDeviceFeatures physDevFeatures;
	vkGetPhysicalDeviceFeatures(physical_gpus[0], &physDevFeatures);

	m_instance = instance;
	m_gpu = physical_gpus[0];
	m_gpu_props = gpu_props;
	m_computeQueueIndex = computeQueueIndex;

	float queue_priorities = 1.0f;
	VkDeviceQueueCreateInfo queues[2];
	Clear(queues, sizeof(queues) / sizeof(queues[0]));
	queues[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queues[0].pNext = NULL;
	queues[0].queueFamilyIndex = m_computeQueueIndex;
	queues[0].queueCount = 1;
	queues[0].pQueuePriorities = &queue_priorities;
	queues[0].flags = 0;

	VkDeviceCreateInfo deviceInfo;
	Clear(&deviceInfo);
	deviceInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	deviceInfo.pNext = NULL;
	deviceInfo.queueCreateInfoCount = 1;
	deviceInfo.pQueueCreateInfos = queues;
	deviceInfo.enabledLayerCount = 0;
	deviceInfo.ppEnabledLayerNames = NULL;
	deviceInfo.enabledExtensionCount = 0;
	deviceInfo.ppEnabledExtensionNames = NULL;
	deviceInfo.pEnabledFeatures = NULL;
	err = vkCreateDevice(m_gpu, &deviceInfo, &m_allocator, &m_device);
	dgAssert(err == VK_SUCCESS);

	vkGetPhysicalDeviceMemoryProperties(m_gpu, &m_memory_properties);
	vkGetDeviceQueue(m_device, m_computeQueueIndex, 0, &m_queue);

	// Create the shaders
	VkPipelineCacheCreateInfo pipeLineCacheInfo;
	Clear(&pipeLineCacheInfo);
	pipeLineCacheInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	err = vkCreatePipelineCache(m_device, &pipeLineCacheInfo, &m_allocator, &m_pipeLineCache);
	dgAssert(err == VK_SUCCESS);

	m_initBody.CreateInitBody(*this);
}

void dgVulkanContext::DestroyDevice()
{
	vkDeviceWaitIdle(m_device);

	m_initBody.Destroy(*this);
	vkDestroyPipelineCache(m_device, m_pipeLineCache, &m_allocator);
	vkDestroyDevice(m_device, &m_allocator);
	vkDestroyInstance(m_instance, &m_allocator);
}

void* dgVulkanContext::vkAllocationFunction(void* pUserData, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	dgMemoryAllocator* const allocator = (dgMemoryAllocator*)pUserData;
	void* const ptr = allocator->MallocLow(dgInt32(size), dgInt32(alignment));
	dgAssert(alignment * ((size_t)ptr / alignment) == (size_t)ptr);
	dgVulkanContext::m_totalMemory += allocator->GetSize(ptr);
	return ptr;
}

void* dgVulkanContext::vkReallocationFunction(void* pUserData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	void* const newPtr = vkAllocationFunction(pUserData, size, alignment, allocationScope);
	if (pOriginal) {
		dgMemoryAllocator* const allocator = (dgMemoryAllocator*)pUserData;
		int copyBytes = dgMin(allocator->GetSize(pOriginal), int(size));
		memcpy(newPtr, pOriginal, copyBytes);
		vkFreeFunction(pUserData, pOriginal);
	}
	return newPtr;
}

void dgVulkanContext::vkFreeFunction(void* pUserData, void* pMemory)
{
	if (pMemory) {
		dgMemoryAllocator* const allocator = (dgMemoryAllocator*)pUserData;
		dgVulkanContext::m_totalMemory -= allocator->GetSize(pMemory);
		allocator->Free(pMemory);
	}
}

void dgVulkanContext::vkInternalAllocationNotification(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope)
{
	dgAssert(0);
}

void dgVulkanContext::vkInternalFreeNotification(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope)
{
	dgAssert(0);
}


dgInt32 dgVulkanContext::FindLayer(const char* const name, dgInt32 count, VkLayerProperties* const layers)
{
	for (dgInt32 i = 0; i < count; i++) {
		if (!strcmp(name, layers[i].layerName)) {
			return i;
		}
	}
	return -1;
}
