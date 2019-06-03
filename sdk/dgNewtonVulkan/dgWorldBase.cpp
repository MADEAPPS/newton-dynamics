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

dgInt32 dgWorldBase::m_totalMemory = 0;
char dgWorldBase::m_libPath[256];

const char* dgWorldBase::m_validationLayer0 = "VK_LAYER_KHRONOS_validation";


// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgWorld* const world, dgMemoryAllocator* const allocator)
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
		dgAssert(layersCount < sizeof (layers) / sizeof (layers[0]));
		error = vkEnumerateInstanceLayerProperties(&layersCount, layers);
		dgAssert(error == VK_SUCCESS);

		dgInt32 index = dgWorldBase::FindLayer(dgWorldBase::m_validationLayer0, layersCount, layers);
		dgAssert (index != -1);

		layersNames[0] = dgWorldBase::m_validationLayer0;
		layersCount = 1;
	}


	VkInstanceCreateInfo inst_info;
	Clear(&inst_info);
	inst_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	inst_info.pApplicationInfo = &app;
	inst_info.enabledLayerCount = layersCount;
	inst_info.ppEnabledLayerNames = layersNames;
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
	error = vkCreateInstance(&inst_info, &vkAllocators, &instance);
	if (error != VK_SUCCESS) {
		return NULL;
	}

	uint32_t gpu_count = 0;
	error = vkEnumeratePhysicalDevices(instance, &gpu_count, NULL);
	if ((error != VK_SUCCESS) || (gpu_count == 0)) {
		vkDestroyInstance(instance, &vkAllocators);
		return NULL;
	}

	static dgWorldBase module(world, allocator);
	module.m_score = 10;
	module.InitDevice(instance, &vkAllocators);

	dgVulkanContext& context = module.GetContext();
#ifdef _DEBUG
	sprintf (module.m_hardwareDeviceName, "NewtonVK_d %s", context.m_gpu_props.deviceName);
#else
	sprintf(module.m_hardwareDeviceName, "NewtonVK %s", context.m_gpu_props.deviceName);
#endif

	return &module;
}

dgWorldBase::dgWorldBase(dgWorld* const world, dgMemoryAllocator* const allocator)
	:dgWorldPlugin(world, allocator)
	,dgSolver(world, allocator)
{
}

dgWorldBase::~dgWorldBase()
{
	Cleanup();
	DestroyDevice ();
}

void dgWorldBase::InitDevice (VkInstance instance, VkAllocationCallbacks* const allocators)
{
	dgVulkanContext& context = GetContext();

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
	dgAssert (computeQueueIndex != -1);

	VkPhysicalDeviceFeatures physDevFeatures;
	vkGetPhysicalDeviceFeatures(physical_gpus[0], &physDevFeatures);

	//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfaceSupportKHR);
	//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfaceCapabilitiesKHR);
	//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfaceFormatsKHR);
	//	GET_INSTANCE_PROC_ADDR(demo->inst, GetPhysicalDeviceSurfacePresentModesKHR);
	//	GET_INSTANCE_PROC_ADDR(demo->inst, GetSwapchainImagesKHR);
	//	PFN_vkVoidFunction xxxx = vkGetInstanceProcAddr(instance, GetPhysicalDeviceSurfaceSupportKHR);

	context.m_instance = instance;
	context.m_allocators = *allocators;
	context.m_gpu = physical_gpus[0];
	context.m_gpu_props = gpu_props;
	context.m_computeQueueIndex = computeQueueIndex;


	float queue_priorities = 1.0f;	
	VkDeviceQueueCreateInfo queues[2];
	Clear(queues, sizeof(queues) / sizeof (queues[0]));
	queues[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queues[0].pNext = NULL;
	queues[0].queueFamilyIndex = context.m_computeQueueIndex;
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
  	err = vkCreateDevice(context.m_gpu, &deviceInfo, &context.m_allocators, &context.m_device);
	dgAssert(err == VK_SUCCESS);

	vkGetPhysicalDeviceMemoryProperties(context.m_gpu, &context.m_memory_properties);
	vkGetDeviceQueue(context.m_device, context.m_computeQueueIndex, 0, &context.m_queue);

	// Create the shaders
	VkPipelineCacheCreateInfo pipeLineCacheInfo;
	Clear (&pipeLineCacheInfo);
	pipeLineCacheInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	err = vkCreatePipelineCache(context.m_device, &pipeLineCacheInfo, &context.m_allocators, &context.m_pipeLineCache);
	dgAssert(err == VK_SUCCESS);

	context.m_initBodyModule = CreateShaderModule("InitBodyArray");


//	VkDescriptorSetLayoutBinding descriptorSetLayoutBindings[2] = {
//		{
//			0,
//			VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
//			1,
//			VK_SHADER_STAGE_COMPUTE_BIT,
//			0
//		},
//		{
//			1,
//			VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
//			1,
//			VK_SHADER_STAGE_COMPUTE_BIT,
//			0
//		}
//	};
//
	VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo;
//		VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
//		0,
//		0,
//		2,
//		descriptorSetLayoutBindings
//	};
	Clear (&descriptorSetLayoutCreateInfo);
	descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;

	err = vkCreateDescriptorSetLayout(context.m_device, &descriptorSetLayoutCreateInfo, 0, &context.m_initBodyLayout);
	dgAssert(err == VK_SUCCESS);

//	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {
//		VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
//		0,
//		0,
//		1,
//		&descriptorSetLayout,
//		0,
//		0
//	};
	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo;
	Clear (&pipelineLayoutCreateInfo);
	pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutCreateInfo.setLayoutCount;
	pipelineLayoutCreateInfo.pSetLayouts = &context.m_initBodyLayout;

	err = vkCreatePipelineLayout(context.m_device, &pipelineLayoutCreateInfo, 0, &context.m_initBodyPipelineLayout);
	dgAssert(err == VK_SUCCESS);

	VkComputePipelineCreateInfo computePipeLineInfo;
	Clear (&computePipeLineInfo);
	computePipeLineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	computePipeLineInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	computePipeLineInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	computePipeLineInfo.stage.module = context.m_initBodyModule;
	computePipeLineInfo.stage.pName = "main";
	computePipeLineInfo.layout = context.m_initBodyPipelineLayout;

	err = vkCreateComputePipelines(context.m_device, context.m_pipeLineCache, 1, &computePipeLineInfo, &context.m_allocators, &context.m_initBodyPipeLine);
	dgAssert(err == VK_SUCCESS);
}

void dgWorldBase::DestroyDevice ()
{
	dgVulkanContext& context = GetContext();

	vkDeviceWaitIdle(context.m_device);

	vkDestroyPipelineLayout(context.m_device, context.m_initBodyPipelineLayout, &context.m_allocators);
	vkDestroyDescriptorSetLayout(context.m_device, context.m_initBodyLayout, &context.m_allocators);
	vkDestroyPipeline(context.m_device, context.m_initBodyPipeLine, &context.m_allocators);
	vkDestroyShaderModule (context.m_device, context.m_initBodyModule, &context.m_allocators);
	vkDestroyPipelineCache(context.m_device, context.m_pipeLineCache, &context.m_allocators);

	vkDestroyDevice(context.m_device, &context.m_allocators);
	vkDestroyInstance(context.m_instance, &context.m_allocators);
}

VkShaderModule dgWorldBase::CreateShaderModule(const char* const shaderName)
{
	char fullPath[1024];
	uint32_t shaderByteCode[1024 * 32];

	sprintf(fullPath, "%s%s.spv", m_libPath, shaderName);
	FILE* const file = fopen(fullPath, "rb");
	dgAssert(file);
	fgets(fullPath, sizeof (fullPath), file);

	int count = 1;
	uint32_t code;
	fscanf(file, "%x", &shaderByteCode);
	while (!feof(file) && fscanf(file, ", %x", &code)) {
		shaderByteCode[count] = code;
		count++;
		dgAssert(count < sizeof(shaderByteCode) / sizeof(shaderByteCode[0]));
	}

	fclose(file);
	
	VkShaderModuleCreateInfo shaderModuleCreateInfo;
	Clear(&shaderModuleCreateInfo);
	shaderModuleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	shaderModuleCreateInfo.codeSize = count * sizeof(uint32_t);
	shaderModuleCreateInfo.pCode = (uint32_t*)&shaderByteCode[0];
	
	VkShaderModule module;
	VkResult err = VK_SUCCESS;

	dgVulkanContext& context = GetContext();
	err = vkCreateShaderModule(context.m_device, &shaderModuleCreateInfo, &context.m_allocators, &module);
	dgAssert(err == VK_SUCCESS);

	return module;
}

dgInt32 dgWorldBase::FindLayer(const char* const name, dgInt32 count, VkLayerProperties* const layers)
{
	for (dgInt32 i = 0; i < count; i++) {
		if (!strcmp(name, layers[i].layerName)) {
			return i;
		}
	}
	return -1;
}

void* dgWorldBase::vkAllocationFunction(void* pUserData, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	dgMemoryAllocator* const allocator = (dgMemoryAllocator*) pUserData;
	void* const ptr = allocator->Malloc (dgInt32 (size));
	dgAssert (alignment * ((size_t)ptr / alignment) == (size_t)ptr);
	dgWorldBase::m_totalMemory += allocator->GetSize(ptr);
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
		dgWorldBase::m_totalMemory -= allocator->GetSize(pMemory);
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