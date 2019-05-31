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

char dgWorldBase::m_libPath[256];


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
	dgVulkanContext& context = module.GetContext();
	module.m_score = 10;
	context.m_gpu = physical_gpus[0];
	context.m_gpu_props = gpu_props;
	context.m_instance = instance;
	context.m_allocators = vkAllocators;
	context.m_computeQueueIndex = computeQueueIndex;
#ifdef _DEBUG
	sprintf (module.m_hardwareDeviceName, "NewtonVK_d %s", context.m_gpu_props.deviceName);
#else
	sprintf(module.m_hardwareDeviceName, "NewtonVK %s", context.m_gpu_props.deviceName);
#endif
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
	Cleanup();
	dgVulkanContext& context = GetContext();

//	vkDeviceWaitIdle(demo->device);
	vkDeviceWaitIdle(context.m_device);

//	// Wait for fences from present operations
//	for (i = 0; i < FRAME_LAG; i++) {
//		vkWaitForFences(demo->device, 1, &demo->fences[i], VK_TRUE, UINT64_MAX);
//		vkDestroyFence(demo->device, demo->fences[i], NULL);
//		vkDestroySemaphore(demo->device, demo->image_acquired_semaphores[i], NULL);
//		vkDestroySemaphore(demo->device, demo->draw_complete_semaphores[i], NULL);
//		if (demo->separate_present_queue) {
//			vkDestroySemaphore(demo->device, demo->image_ownership_semaphores[i], NULL);
//		}
//	}
//
//	// If the window is currently minimized, demo_resize has already done some cleanup for us.
//	if (!demo->is_minimized) {
//		for (i = 0; i < demo->swapchainImageCount; i++) {
//			vkDestroyFramebuffer(demo->device, demo->swapchain_image_resources[i].framebuffer, NULL);
//		}
//		vkDestroyDescriptorPool(demo->device, demo->desc_pool, NULL);
//
//		vkDestroyPipeline(demo->device, demo->pipeline, NULL);
//		vkDestroyPipelineCache(demo->device, demo->pipelineCache, NULL);
//		vkDestroyRenderPass(demo->device, demo->render_pass, NULL);
//		vkDestroyPipelineLayout(demo->device, demo->pipeline_layout, NULL);
//		vkDestroyDescriptorSetLayout(demo->device, demo->desc_layout, NULL);
//
//		for (i = 0; i < DEMO_TEXTURE_COUNT; i++) {
//			vkDestroyImageView(demo->device, demo->textures[i].view, NULL);
//			vkDestroyImage(demo->device, demo->textures[i].image, NULL);
//			vkFreeMemory(demo->device, demo->textures[i].mem, NULL);
//			vkDestroySampler(demo->device, demo->textures[i].sampler, NULL);
//		}
//		demo->fpDestroySwapchainKHR(demo->device, demo->swapchain, NULL);
//
//		vkDestroyImageView(demo->device, demo->depth.view, NULL);
//		vkDestroyImage(demo->device, demo->depth.image, NULL);
//		vkFreeMemory(demo->device, demo->depth.mem, NULL);
//
//		for (i = 0; i < demo->swapchainImageCount; i++) {
//			vkDestroyImageView(demo->device, demo->swapchain_image_resources[i].view, NULL);
//			vkFreeCommandBuffers(demo->device, demo->cmd_pool, 1, &demo->swapchain_image_resources[i].cmd);
//			vkDestroyBuffer(demo->device, demo->swapchain_image_resources[i].uniform_buffer, NULL);
//			vkFreeMemory(demo->device, demo->swapchain_image_resources[i].uniform_memory, NULL);
//		}
//		free(demo->swapchain_image_resources);
//		free(demo->queue_props);
//		vkDestroyCommandPool(demo->device, demo->cmd_pool, NULL);
//
//		if (demo->separate_present_queue) {
//			vkDestroyCommandPool(demo->device, demo->present_cmd_pool, NULL);
//		}
//	}

//	vkDeviceWaitIdle(demo->device);
//	vkDestroyDevice(demo->device, NULL);

	vkDeviceWaitIdle(context.m_device);
	vkDestroyDevice(context.m_device, &context.m_allocators);

//	if (demo->validate) {
//		demo->DestroyDebugUtilsMessengerEXT(demo->inst, demo->dbg_messenger, NULL);
//	}
//	vkDestroySurfaceKHR(demo->inst, demo->surface, NULL);
//
//#if defined(VK_USE_PLATFORM_XLIB_KHR)
//	XDestroyWindow(demo->display, demo->xlib_window);
//	XCloseDisplay(demo->display);
//#elif defined(VK_USE_PLATFORM_XCB_KHR)
//	xcb_destroy_window(demo->connection, demo->xcb_window);
//	xcb_disconnect(demo->connection);
//	free(demo->atom_wm_delete_window);
//#elif defined(VK_USE_PLATFORM_WAYLAND_KHR)
//	wl_keyboard_destroy(demo->keyboard);
//	wl_pointer_destroy(demo->pointer);
//	wl_seat_destroy(demo->seat);
//	wl_shell_surface_destroy(demo->shell_surface);
//	wl_surface_destroy(demo->window);
//	wl_shell_destroy(demo->shell);
//	wl_compositor_destroy(demo->compositor);
//	wl_registry_destroy(demo->registry);
//	wl_display_disconnect(demo->display);
//#endif
//


//	vkDestroyInstance(demo->inst, NULL);
	vkDestroyInstance(context.m_instance, &context.m_allocators);
}

void dgWorldBase::InitDevice ()
{
	dgVulkanContext& context = GetContext();

	VkResult err = VK_SUCCESS;
	float queue_priorities[1] = { 0.0 };
	VkDeviceQueueCreateInfo queues[2];
	Clear(queues, sizeof(queues) / sizeof (queues[0]));
	queues[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queues[0].pNext = NULL;
	queues[0].queueFamilyIndex = context.m_computeQueueIndex;
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
  	err = vkCreateDevice(context.m_gpu, &deviceInfo, &context.m_allocators, &context.m_device);
	dgAssert(err == VK_SUCCESS);

    vkGetDeviceQueue(context.m_device, context.m_computeQueueIndex, 0, &context.m_queue);
	vkGetPhysicalDeviceMemoryProperties(context.m_gpu, &context.m_memory_properties);

	VkShaderModule xxx = CreateShaderModule("InitBodyArray");
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
	
	VkShaderModuleCreateInfo shaderModuleCreateInfo = {
		VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
		0,
		0,
		count * sizeof (uint32_t),
		(uint32_t*)&shaderByteCode[0]
	};
	

	return 0;
}

void* dgWorldBase::vkAllocationFunction(void* pUserData, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	dgMemoryAllocator* const allocator = (dgMemoryAllocator*) pUserData;
	void* const ptr = allocator->Malloc (dgInt32 (size));
	dgAssert (alignment * ((size_t)ptr / alignment) == (size_t)ptr);
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