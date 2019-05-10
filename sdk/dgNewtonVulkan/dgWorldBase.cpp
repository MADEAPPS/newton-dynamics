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

	VkAllocationCallbacks* allocators = NULL;

	VkInstance instance;
	VkResult error = vkCreateInstance(&inst_info, allocators, &instance);
	if (error != VK_SUCCESS) {
		return NULL;
	}

	uint32_t gpu_count = 0;
	error = vkEnumeratePhysicalDevices(instance, &gpu_count, NULL);
	if ((error != VK_SUCCESS) || (gpu_count == 0)) {
		vkDestroyInstance (instance, allocators);
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

	static dgWorldBase module(world, allocator);
	module.m_score = 0;
	module.m_gpu = physical_gpus[0];
	module.m_gpu_props = gpu_props;
	module.m_instance = instance;
	return &module;
}

dgWorldBase::dgWorldBase(dgWorld* const world, dgMemoryAllocator* const allocator)
	:dgWorldPlugin(world, allocator)
	,dgSolver(world, allocator)
{
}

dgWorldBase::~dgWorldBase()
{
	VkAllocationCallbacks* allocators = NULL;
	vkDestroyInstance(m_instance, allocators);
}

const char* dgWorldBase::GetId() const
{
	return m_gpu_props.deviceName;
}

dgInt32 dgWorldBase::GetScore() const
{
	return m_score;
}

void dgWorldBase::CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
{
	dgSolver::CalculateJointForces(cluster, bodyArray, jointArray, timestep);
}