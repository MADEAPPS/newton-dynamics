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
#include "dgWorldBase.h"




// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgWorld* const world, dgMemoryAllocator* const allocator)
{
	VkApplicationInfo app;
	Clear(&app);
	app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	app.pApplicationName = "NewtonVulkanSolver";
	app.applicationVersion = 100;
	app.pEngineName = "Newton-3.14";
	app.engineVersion = 314;
	app.apiVersion = VK_API_VERSION_1_0;

	VkInstanceCreateInfo inst_info;
	Clear(&inst_info);
	inst_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	inst_info.pApplicationInfo = &app;
	inst_info.enabledLayerCount = 0;
	inst_info.ppEnabledLayerNames = NULL;
	inst_info.enabledExtensionCount = 0;
	inst_info.ppEnabledExtensionNames = NULL;

	VkAllocationCallbacks vkAllocators;
	Clear(&inst_info);
	vkAllocators.pUserData = allocator; 
	vkAllocators.pfnFree = dgWorldBase::vkFreeFunction;
	vkAllocators.pfnAllocation = dgWorldBase::vkAllocationFunction;
	vkAllocators.pfnReallocation = dgWorldBase::vkReallocationFunction;
	vkAllocators.pfnInternalAllocation = dgWorldBase::vkInternalAllocationNotification;
	vkAllocators.pfnInternalFree = dgWorldBase::vkInternalFreeNotification;

	VkInstance instance;
	VkResult error = vkCreateInstance(&inst_info, &vkAllocators, &instance);
	if (error != VK_SUCCESS) {
		return NULL;
	}

	uint32_t gpu_count = 0;
	error = vkEnumeratePhysicalDevices(instance, &gpu_count, NULL);
	if ((error != VK_SUCCESS) || (gpu_count == 0)) {
		vkDestroyInstance (instance, &vkAllocators);
		return NULL;
	}

	dgAssert (gpu_count < 8);
	VkPhysicalDevice physical_gpus[8];
	error = vkEnumeratePhysicalDevices(instance, &gpu_count, &physical_gpus[0]);
	dgAssert (error == VK_SUCCESS);
	dgAssert (gpu_count >= 1);

	if (gpu_count > 1) {
		dgAssert (0);
		// try sort gpus such that the best gpu is the first
		//VkPhysicalDevice *physical_devices = malloc(sizeof(VkPhysicalDevice)* gpu_count);
		//err = vkEnumeratePhysicalDevices(demo->inst, &gpu_count, physical_devices);
		//assert(!err);
	}

	VkPhysicalDeviceProperties gpu_props;
	Clear(&gpu_props);
	vkGetPhysicalDeviceProperties(physical_gpus[0], &gpu_props);

	/* Call with NULL data to get count */
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

	if (computeQueueIndex == -1) {
		vkDestroyInstance(instance, &vkAllocators);
		return NULL;
	}

//	m_computeQueueIndex

	// Query fine-grained feature support for this device.
	//  If app has specific feature requirements it should check supported
	//  features based on this query
	VkPhysicalDeviceFeatures physDevFeatures;
	vkGetPhysicalDeviceFeatures(physical_gpus[0], &physDevFeatures);

//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfaceSupportKHR);
//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfaceCapabilitiesKHR);
//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfaceFormatsKHR);
//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfacePresentModesKHR);
//	GET_INSTANCE_PROC_ADDR(demo->inst, GetSwapchainImagesKHR);
//	PFN_vkVoidFunction xxxx = vkGetInstanceProcAddr(instance, GetPhysicalDeviceSurfaceSupportKHR);

	static dgWorldBase module(world, allocator);
	module.m_score = 0;
	module.m_gpu = physical_gpus[0];
	module.m_gpu_props = gpu_props;
	module.m_instance = instance;
	module.m_allocators = vkAllocators;
	module.m_computeQueueIndex = computeQueueIndex;
	sprintf (module.m_hardwareDeviceName, "Newton gpu: %s", module.m_gpu_props.deviceName);
	module.InitDevice();
	return &module;
}

dgWorldBase::dgWorldBase(dgWorld* const world, dgMemoryAllocator* const allocator)
	:dgWorldPlugin(world, allocator)
	,dgSolver(world, allocator)
{
}

dgWorldBase::~dgWorldBase()
{
	vkDeviceWaitIdle(m_device);
	vkDestroyDevice(m_device, &m_allocators);
	vkDestroyInstance(m_instance, &m_allocators);
}

void dgWorldBase::InitDevice ()
{
	float queue_priorities[1] = { 0.0 };
	VkDeviceQueueCreateInfo queues[2];
	Clear(queues, sizeof(queues) / sizeof (queues[0]));
	queues[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queues[0].pNext = NULL;
	queues[0].queueFamilyIndex = m_computeQueueIndex;
	queues[0].queueCount = 1;
	queues[0].pQueuePriorities = queue_priorities;
	queues[0].flags = 0;

	VkDeviceCreateInfo deviceInfo;
	Clear(&deviceInfo);
	deviceInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	deviceInfo.pNext = NULL;
	deviceInfo.queueCreateInfoCount = 1;
	deviceInfo.pQueueCreateInfos = queues;
	deviceInfo.enabledLayerCount = 0;
	deviceInfo.ppEnabledLayerNames = NULL;
	//	deviceInfo.enabledExtensionCount = demo->enabled_extension_count,
	deviceInfo.enabledExtensionCount = 0;
	deviceInfo.ppEnabledExtensionNames = NULL;
	deviceInfo.pEnabledFeatures = NULL;
	VkResult err = vkCreateDevice(m_gpu, &deviceInfo, &m_allocators, &m_device);
	dgAssert(err == VK_SUCCESS);
}


void* dgWorldBase::vkAllocationFunction(void* pUserData, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	dgMemoryAllocator* const allocator = (dgMemoryAllocator*) pUserData;
	void* const ptr = allocator->Malloc (size);
	dgAssert (alignment * ((dgInt64)ptr / alignment) == (dgInt64)ptr);
	return ptr;
}

void* dgWorldBase::vkReallocationFunction(void* pUserData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	void* const newPtr = vkAllocationFunction(pUserData, size, alignment, allocationScope);
	if (pOriginal) {
		dgMemoryAllocator* const allocator = (dgMemoryAllocator*) pUserData;
		int copyBytes = dgMin (allocator->GetSize (pOriginal), int (size));
		memcpy (newPtr, pOriginal, copyBytes);
		vkFreeFunction(pUserData, pOriginal);
	}
	return newPtr;
}

void dgWorldBase::vkFreeFunction(void* pUserData, void* pMemory)
{
	if (pMemory) {
		dgMemoryAllocator* const allocator = (dgMemoryAllocator*) pUserData;
		allocator->Free(pMemory);
	}
}

void dgWorldBase::vkInternalAllocationNotification(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope)
{
	dgAssert(0);
}

void dgWorldBase::vkInternalFreeNotification(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope)
{
	dgAssert(0);
}


const char* dgWorldBase::GetId() const
{
//	return m_gpu_props.deviceName;
	return m_hardwareDeviceName;
}

dgInt32 dgWorldBase::GetScore() const
{
	return m_score;
}

void dgWorldBase::CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
{
	dgSolver::CalculateJointForces(cluster, bodyArray, jointArray, timestep);
}