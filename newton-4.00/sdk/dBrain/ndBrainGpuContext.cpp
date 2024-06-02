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
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"

#if !defined (D_USE_VULKAN_SDK)

ndBrainGpuContext::ndBrainGpuContext()
{

}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

void ndBrainGpuContext::ExecuteTest(ndBrainGpuFloatBuffer&)
{

}

#else


#define ND_SELECT_DISCRETE_GPU

const char* ndBrainGpuContext::m_apiLayers[] =
{
	"VK_LAYER_KHRONOS_validation"
};

const char* ndBrainGpuContext::m_apiExtensionLayers[] =
{
	"VK_EXT_debug_report"
};

ndBrainGpuContext::ndBrainGpuContext()
	:ndClassAlloc()
	,m_allocator(&m_allocatorStruct)
	,m_fence(VK_NULL_HANDLE)
	,m_queue(VK_NULL_HANDLE)
	,m_device(VK_NULL_HANDLE)
	,m_instance(VK_NULL_HANDLE)
	,m_commandPool(VK_NULL_HANDLE)
	,m_descriptorPool(VK_NULL_HANDLE)
	,m_physicalDevice(VK_NULL_HANDLE)
	,m_debugMessenger(VK_NULL_HANDLE)
	,m_queueFamilyIndex(0xffffffff)
	,m_hasValidationLayers(false)
{
	m_allocatorStruct.pUserData = this;
	m_allocatorStruct.pfnFree = VulkanFree;
	m_allocatorStruct.pfnAllocation = VulkanAlloc;
	m_allocatorStruct.pfnReallocation = VulkanRealloc;
	m_allocatorStruct.pfnInternalFree = VulkanInternalFree;
	m_allocatorStruct.pfnInternalAllocation = VulkanInternalAlloc;

	CreateInstance();
	SetupDebugMessenger();
	CreatePhysicalDevice();
	SelectGraphicsQueue();
	CreateLogicalDevice();
	CreateCommandPool();
	CreateDescriptorPool();
	CreateFence();
	LoadShaderPrograms();
}

ndBrainGpuContext::~ndBrainGpuContext()
{
	if (m_hasValidationLayers)
	{
		//auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance, "vkDestroyDebugUtilsMessengerEXT");
		auto func = (PFN_vkDestroyDebugReportCallbackEXT)vkGetInstanceProcAddr(m_instance, "vkDestroyDebugReportCallbackEXT");
		if (func)
		{
			func(m_instance, m_debugMessenger, m_allocator);
		}
	}

	vkDestroyFence(m_device, m_fence, m_allocator);
	vkDestroyShaderModule(m_device, m_computeShaderModule0, m_allocator);
	vkDestroyShaderModule(m_device, m_computeShaderModule1, m_allocator);
	vkDestroyDescriptorPool(m_device, m_descriptorPool, m_allocator);
	vkDestroyCommandPool(m_device, m_commandPool, m_allocator);
	vkDestroyDevice(m_device, m_allocator);
	vkDestroyInstance(m_instance, m_allocator);
}

void ndBrainGpuContext::CheckResultVulkan(VkResult err)
{
	if (err != VK_SUCCESS)
	{
		ndTrace(("[vulkan] Error: VkResult = %d\n", err));
		ndAssert(0);
	}
}

void* ndBrainGpuContext::VulkanAlloc(void*, size_t size, size_t, VkSystemAllocationScope)
{
	//ndAssert(alignment);
	//ndAssert(alignment <= D_MEMORY_ALIGMNET);
	return ndMemory::Malloc(size);
}

void ndBrainGpuContext::VulkanFree(void*, void* memory)
{
	ndMemory::Free(memory);
}

void* ndBrainGpuContext::VulkanRealloc(void* pUserData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	size_t oldSize = ndMemory::GetOriginalSize(pOriginal);
	if (size > oldSize)
	{
		void* const ptr = VulkanAlloc(pUserData, size, alignment, allocationScope);
		ndMemCpy((char*)ptr, (char*)pOriginal, ndInt32(oldSize));
		VulkanFree(pUserData, pOriginal);
		return ptr;
	}
	else
	{
		ndAssert(0);
	}

	return nullptr;
}

void ndBrainGpuContext::VulkanInternalAlloc(void*, size_t, VkInternalAllocationType, VkSystemAllocationScope)
{
	ndAssert(0);
}

void ndBrainGpuContext::VulkanInternalFree(void*, size_t, VkInternalAllocationType, VkSystemAllocationScope)
{
	ndAssert(0);
}

VkDevice ndBrainGpuContext::GetDevice() const
{
	return m_device;
}

VkAllocationCallbacks* ndBrainGpuContext::GetAllocator() const
{
	return m_allocator;
}

VkPhysicalDevice ndBrainGpuContext::GetPhysicalDevice() const
{
	return m_physicalDevice;
}

VkCommandPool ndBrainGpuContext::GetCommandPool() const
{
	return m_commandPool;
}

VkDescriptorPool ndBrainGpuContext::GetDescriptorPool() const
{
	return m_descriptorPool;
}


void ndBrainGpuContext::CreateInstance()
{
	uint32_t layerCount;
	ndFixSizeArray <VkLayerProperties, 128> layerProperties;
	vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
	layerProperties.SetCount(ndInt32(layerCount));
	vkEnumerateInstanceLayerProperties(&layerCount, &layerProperties[0]);

	bool apiLayersFound = false;
	for (ndInt32 i = 0; i < ndInt32(layerCount); ++i)
	{
		if (strcmp(m_apiLayers[0], layerProperties[i].layerName) == 0)
		{
			apiLayersFound = true;
			break;
		}
	}

	m_hasValidationLayers = apiLayersFound;
	uint32_t extensionCount;
	ndFixSizeArray <VkExtensionProperties, 128> extensionProperties;
	vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);
	extensionProperties.SetCount(ndInt32(extensionCount));
	vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, &extensionProperties[0]);

	bool foundExtension = false;
	for (ndInt32 i = 0; i < ndInt32(extensionCount); ++i)
	{
		if (strcmp(m_apiExtensionLayers[0], extensionProperties[i].extensionName) == 0)
		{
			foundExtension = true;
			break;
		}
	}

	VkApplicationInfo appInfo = {};
	appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pApplicationName = "newton 4 demos";
	appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.pEngineName = "newton dynamics 4";
	appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.apiVersion = VK_MAKE_VERSION(1, 0, 0);

	VkInstanceCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	createInfo.pApplicationInfo = &appInfo;
	createInfo.enabledLayerCount = apiLayersFound ? uint32_t(1) : uint32_t(0);
	createInfo.ppEnabledLayerNames = &m_apiLayers[0];
	createInfo.enabledExtensionCount = foundExtension ? uint32_t(1) : uint32_t(0);
	createInfo.ppEnabledExtensionNames = &m_apiExtensionLayers[0];

	//VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo{};
	//debugCreateInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
	//debugCreateInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
	//debugCreateInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
	//debugCreateInfo.pfnUserCallback = DebugReportVulkan;
	//createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*)&debugCreateInfo;

	CheckResultVulkan(vkCreateInstance(&createInfo, m_allocator, &m_instance));
}

VKAPI_ATTR VkBool32 VKAPI_CALL ndBrainGpuContext::DebugReportVulkan(
	//VkDebugReportFlagsEXT                       flags,
	//VkDebugReportObjectTypeEXT                  objectType,
	//uint64_t                                    object,
	//size_t                                      location,
	//int32_t                                     messageCode,
	//const char* pLayerPrefix,
	//const char* pMessage,
	//void* pUserData) 
	VkDebugReportFlagsEXT, VkDebugReportObjectTypeEXT, uint64_t, size_t, int32_t,
	const char* pLayerPrefix, const char* pMessage, void*)
{
	ndTrace(("Debug Report: %s: %s\n", pLayerPrefix, pMessage));

	return VK_FALSE;
}

void ndBrainGpuContext::SetupDebugMessenger()
{
	if (m_hasValidationLayers)
	{
		auto func = (PFN_vkCreateDebugReportCallbackEXT)vkGetInstanceProcAddr(m_instance, "vkCreateDebugReportCallbackEXT");
		if (func != nullptr)
		{
			VkDebugReportCallbackCreateInfoEXT createInfo = {};
			createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT;
			createInfo.flags = VK_DEBUG_REPORT_ERROR_BIT_EXT | VK_DEBUG_REPORT_WARNING_BIT_EXT | VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT;
			createInfo.pfnCallback = &DebugReportVulkan;

			CheckResultVulkan(func(m_instance, &createInfo, m_allocator, &m_debugMessenger));
		}
	}
}

void ndBrainGpuContext::CreatePhysicalDevice()
{
	m_physicalDevice = VK_NULL_HANDLE;

	uint32_t gpuCount;
	CheckResultVulkan(vkEnumeratePhysicalDevices(m_instance, &gpuCount, nullptr));
	ndAssert(gpuCount > 0);
	ndAssert(gpuCount < 128);

	ndFixSizeArray<VkPhysicalDevice, 128> gpus;
	gpus.SetCount(ndInt32(gpuCount));
	CheckResultVulkan(vkEnumeratePhysicalDevices(m_instance, &gpuCount, &gpus[0]));

	// If a number >1 of GPUs got reported, find discrete GPU if present, or use first one available. This covers
	// most common cases (multi-gpu/integrated+dedicated graphics). Handling more complicated setups (multiple
	// dedicated GPUs) is out of scope of this sample.
	ndInt32 use_gpu = 0;
	for (ndInt32 i = 0; i < ndInt32(gpuCount); i++)
	{
		VkPhysicalDeviceProperties properties;
		vkGetPhysicalDeviceProperties(gpus[i], &properties);
#ifdef ND_SELECT_DISCRETE_GPU
		if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)
#else
		if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU)
#endif
		{
			use_gpu = i;
			break;
		}
	}
	m_physicalDevice = gpus[use_gpu];
	vkGetPhysicalDeviceProperties(m_physicalDevice, &m_gpuProps);
	ndTrace(("vulkan accelerator: %s\n", m_gpuProps.deviceName));
}

void ndBrainGpuContext::SelectGraphicsQueue()
{
	uint32_t count;
	m_queueFamilyIndex = uint32_t(-1);

	vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &count, nullptr);
	ndAssert(count > 0);
	ndAssert(count < 128);
	ndFixSizeArray<VkQueueFamilyProperties, 128> queues;
	queues.SetCount(ndInt32(count));
	vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &count, &queues[0]);
	for (ndInt32 i = 0; i < ndInt32(count); i++)
	{
		if (queues[i].queueFlags & VK_QUEUE_COMPUTE_BIT)
		{
			m_queueFamilyIndex = uint32_t(i);
			break;
		}
	}
	ndAssert(m_queueFamilyIndex != (uint32_t)-1);
}

void ndBrainGpuContext::CreateLogicalDevice()
{
	m_queue = VK_NULL_HANDLE;
	m_device = VK_NULL_HANDLE;

	const float queue_priority[] = { 1.0f };
	VkDeviceQueueCreateInfo queue_info[1] = {};
	queue_info[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queue_info[0].queueFamilyIndex = m_queueFamilyIndex;
	queue_info[0].queueCount = 1;
	queue_info[0].pQueuePriorities = queue_priority;

	VkPhysicalDeviceFeatures deviceFeatures = {};
	VkDeviceCreateInfo create_info = {};
	create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	create_info.queueCreateInfoCount = sizeof(queue_info) / sizeof(queue_info[0]);
	create_info.pQueueCreateInfos = queue_info;
	create_info.pEnabledFeatures = &deviceFeatures;

	create_info.enabledLayerCount = 1;
	create_info.ppEnabledLayerNames = &m_apiLayers[0];
	//create_info.enabledExtensionCount = 1;
	//create_info.ppEnabledExtensionNames = &m_apiExtensionLayers[0];

	CheckResultVulkan(vkCreateDevice(m_physicalDevice, &create_info, m_allocator, &m_device));
	vkGetDeviceQueue(m_device, m_queueFamilyIndex, 0, &m_queue);
}

void ndBrainGpuContext::CreateCommandPool()
{
	VkCommandPoolCreateInfo commandPoolCreateInfo = {};
	commandPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	commandPoolCreateInfo.flags = 0;
	// the queue family of this command pool. All command buffers allocated from this command pool,
	// must be submitted to queues of this family ONLY. 
	commandPoolCreateInfo.queueFamilyIndex = m_queueFamilyIndex;
	CheckResultVulkan(vkCreateCommandPool(m_device, &commandPoolCreateInfo, m_allocator, &m_commandPool));
}

void ndBrainGpuContext::CreateDescriptorPool()
{
	VkDescriptorPoolSize descriptorPoolSize = {};
	descriptorPoolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorPoolSize.descriptorCount = 2;

	// we only need to allocate one descriptor set from the pool.
	VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
	descriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	descriptorPoolCreateInfo.maxSets = 1000; 
	descriptorPoolCreateInfo.poolSizeCount = 1;
	descriptorPoolCreateInfo.pPoolSizes = &descriptorPoolSize;
	CheckResultVulkan(vkCreateDescriptorPool(m_device, &descriptorPoolCreateInfo, m_allocator, &m_descriptorPool));
}

void ndBrainGpuContext::CreateFence()
{
	VkFenceCreateInfo fenceCreateInfo = {};
	fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceCreateInfo.flags = 0;
	CheckResultVulkan(vkCreateFence(m_device, &fenceCreateInfo, m_allocator, &m_fence));
}

void ndBrainGpuContext::GetShaderFileName(const char* const name, char* const outPathName)
{
#if (defined(WIN32) || defined(_WIN32))
	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	strtolwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	sprintf(outPathName, "%ssdk/dMedia/dbrain/%s", appPath, name);
#elif defined(__APPLE__)
	char tmp[2048];
	CFURLRef appURL(CFBundleCopyBundleURL(CFBundleGetMainBundle()));
	CFStringRef filePath(CFURLCopyFileSystemPath(appURL, kCFURLPOSIXPathStyle));
	CFStringGetCString(filePath, tmp, PATH_MAX, kCFStringEncodingUTF8);
	//char* const ptr = strstr (tmp, "applications");
	//ptr [0] = 0;
	//sprintf (outPathName, "%sapplications/media/%s", tmp, name);
	sprintf(outPathName, "%s/Contents/Resources/%s", tmp, name);

	// Clean up 
	CFRelease(appURL);
	CFRelease(filePath);
#elif defined(__linux__)
	char id[1024];
	char appPath[1024];

	sprintf(id, "/proc/%d/exe", getpid());
	memset(appPath, 0, sizeof(appPath));
	size_t ret = readlink(id, appPath, sizeof(appPath));
	ret = 0;
	char* const end = strstr(appPath, "applications");
	*end = 0;
	sprintf(outPathName, "%ssdk/dMedia/dbrain/%s", appPath, name);
#else
#error  "error: need to implement \"dGetWorkingFileName\" here for this platform"
#endif
}

void ndBrainGpuContext::LoadShaderPrograms()
{
	//Create a shader module. A shader module basically just encapsulates some shader code.

	ndFixSizeArray<char, 1024 * 64> code;
	auto LoadShaderCode = [this, &code](const char* const name)
	{
		char fileName[1024 * 2];
		GetShaderFileName(name, fileName);

		code.SetCount(0);
		FILE* const fp = fopen(fileName, "rb");
		if (fp)
		{
			fseek(fp, 0, SEEK_END);
			long filesize = ftell(fp);
			fseek(fp, 0, SEEK_SET);

			long filesizepadded = long(ceil(filesize / 4.0)) * 4;
			ndAssert(filesizepadded < code.GetCapacity());

			code.SetCount(filesizepadded);
			fread(&code[0], size_t(filesize), sizeof(char), fp);
			fclose(fp);

			for (ndInt32 i = filesize; i < filesizepadded; i++)
			{
				code[i] = 0;
			}
		}
	};

	LoadShaderCode("testShader0-comp.spv");
	VkShaderModuleCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.pCode = (uint32_t*)&code[0];
	createInfo.codeSize = size_t(code.GetCount());
	CheckResultVulkan(vkCreateShaderModule(m_device, &createInfo, m_allocator, &m_computeShaderModule0));

	LoadShaderCode("testShader1-comp.spv");
	//VkShaderModuleCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.pCode = (uint32_t*)&code[0];
	createInfo.codeSize = size_t(code.GetCount());
	CheckResultVulkan(vkCreateShaderModule(m_device, &createInfo, m_allocator, &m_computeShaderModule1));

}


void ndBrainGpuContext::SubmitQueue(ndBrainGpuCommand** commands, ndInt32 commandCount)
{
	VkCommandBuffer xxxx[10];
	for (ndInt32 i = 0; i < commandCount; ++i)
	{
		xxxx[i] = commands[i]->m_commandBuffer;
	}

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = uint32_t(commandCount);
	submitInfo.pCommandBuffers = xxxx; 
	CheckResultVulkan(vkQueueSubmit(m_queue, 1, &submitInfo, m_fence));
	//CheckResultVulkan(vkQueueSubmit(m_queue, uint32_t(commandCount), &submitInfo, m_fence));
	CheckResultVulkan(vkWaitForFences(m_device, 1, &m_fence, VK_TRUE, 100000000000));
}

#endif