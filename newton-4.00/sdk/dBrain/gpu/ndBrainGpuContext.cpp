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

#if defined (D_USE_VULKAN_SDK)

#define ND_SELECT_DISCRETE_GPU

const char* ndBrainGpuContext::m_apiLayers[] =
{
	"VK_LAYER_KHRONOS_validation"
};

const char* ndBrainGpuContext::m_apiExtensionLayers[] =
{
	"VK_EXT_debug_report",
	"VK_EXT_memory_budget",
	"VK_EXT_device_memory_report"
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
	,m_subGroupSize(0)
	,m_queueFamilyIndex(0xffffffff)
	,m_queueInProgress(false)
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
	CheckSubGroupSupport();
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

	for (ndInt32 i = 0; i < sizeof(m_modules) / sizeof(m_modules[0]); ++i)
	{
		if (m_modules[i])
		{
			vkDestroyShaderModule(m_device, m_modules[i], m_allocator);
		}
	}
	
	vkDestroyFence(m_device, m_fence, m_allocator);
	vkDestroyDescriptorPool(m_device, m_descriptorPool, m_allocator);
	vkDestroyCommandPool(m_device, m_commandPool, m_allocator);
	vkDestroyDevice(m_device, m_allocator);
	vkDestroyInstance(m_instance, m_allocator);

	ndAssert(!m_memoryDictionary.GetCount());
}

bool ndBrainGpuContext::HasGpuSupport()
{
	return true;
}

void ndBrainGpuContext::CheckResultVulkan(VkResult err)
{
	if (err != VK_SUCCESS)
	{
		ndTrace(("[vulkan] Error: VkResult = %d\n", err));
		ndAssert(0);
	}
}

void* ndBrainGpuContext::VulkanAlloc(void* userData, size_t size, size_t alignment, VkSystemAllocationScope)
{
	ndBrainGpuContext* const context = (ndBrainGpuContext*)userData;

	ndTree<ndMemoryEntry, void*>& dictionary = context->m_memoryDictionary;
	
	size_t originalSize = size;
	alignment = ndMax(alignment, size_t(D_MEMORY_ALIGMNET));
	if (alignment > D_MEMORY_ALIGMNET)
	{
		size += alignment;
	}

	char* const allocatedPtr = (char*)ndMemory::Malloc(size);
	size_t const ptrValue = ((size_t)(allocatedPtr + alignment - 1)) & ~(alignment - 1);
	char* const ptr = (char*)ptrValue;

	ndTree<ndMemoryEntry, void*>::ndNode* const node = dictionary.Insert(ptr);
	ndMemoryEntry& entry = node->GetInfo();

	entry.m_ptr = allocatedPtr;
	entry.m_size = originalSize;
	return ptr;
}

void ndBrainGpuContext::VulkanFree(void* userData, void* memory)
{
	if (memory)
	{
		ndBrainGpuContext* const context = (ndBrainGpuContext*)userData;
		ndTree<ndMemoryEntry, void*>& dictionary = context->m_memoryDictionary;
		ndTree<ndMemoryEntry, void*>::ndNode* const node = dictionary.Find(memory);
		const ndMemoryEntry& entry = node->GetInfo();
		ndMemory::Free(entry.m_ptr);
		dictionary.Remove(node);
	}
}

void* ndBrainGpuContext::VulkanRealloc(void* userData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
{
	ndBrainGpuContext* const context = (ndBrainGpuContext*)userData;
	ndTree<ndMemoryEntry, void*>& dictionary = context->m_memoryDictionary;
	ndTree<ndMemoryEntry, void*>::ndNode* const node = dictionary.Find(pOriginal);
	const ndMemoryEntry& entry = node->GetInfo();

	size_t oldSize = entry.m_size;
	void* const ptr = VulkanAlloc(userData, size, alignment, allocationScope);
	ndMemCpy((char*)ptr, (char*)pOriginal, ndInt32(ndMin (size, oldSize)));
	VulkanFree(userData, pOriginal);
	return ptr;
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

ndInt32 ndBrainGpuContext::GetSubGroupSize() const
{
	return m_subGroupSize;
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

	ndInt32 extCount = 0;
	const char* apiExtensionLayers[32];

	for (ndInt32 j = 0; j < sizeof(m_apiExtensionLayers) / sizeof(m_apiExtensionLayers[3]); ++j)
	{
		for (ndInt32 i = 0; i < ndInt32(extensionCount); ++i)
		{
			if (strcmp(m_apiExtensionLayers[j], extensionProperties[i].extensionName) == 0)
			{
				apiExtensionLayers[extCount] = m_apiExtensionLayers[j];
				extCount++;
				break;
			}
		}
	}

	VkApplicationInfo appInfo = {};
	appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pApplicationName = "newton 4 demos";
	appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.pEngineName = "newton dynamics 4";
	appInfo.engineVersion = VK_MAKE_VERSION(1, 1, 0);
	appInfo.apiVersion = VK_MAKE_VERSION(1, 1, 0);

	VkInstanceCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	createInfo.pApplicationInfo = &appInfo;
	createInfo.enabledLayerCount = apiLayersFound ? uint32_t(1) : uint32_t(0);
	createInfo.ppEnabledLayerNames = &m_apiLayers[0];
	createInfo.enabledExtensionCount = uint32_t(extCount);
	createInfo.ppEnabledExtensionNames = &apiExtensionLayers[0];

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
	ndExpandTraceMessage("Debug Report: %s: %s\n", pLayerPrefix, pMessage);
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
		if ((properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) ||
			(properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU))
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
	ndExpandTraceMessage("vulkan accelerator: %s\n", m_gpuProps.deviceName);
}

void ndBrainGpuContext::CheckSubGroupSupport()
{
	VkPhysicalDeviceSubgroupProperties subgroupProperties = {};
	subgroupProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SUBGROUP_PROPERTIES; 
	subgroupProperties.pNext = VK_NULL_HANDLE;

	VkPhysicalDeviceProperties2 physicalDeviceProperties = {};
	physicalDeviceProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
	physicalDeviceProperties.pNext = &subgroupProperties;

	vkGetPhysicalDeviceProperties2(m_physicalDevice, &physicalDeviceProperties);
	m_subGroupSize = ndInt32((subgroupProperties.supportedOperations & VK_SUBGROUP_FEATURE_ARITHMETIC_BIT) ? subgroupProperties.subgroupSize : 0);
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
	//VkDescriptorPoolSize descriptorPoolSize = {};
	VkDescriptorPoolSize descriptorPoolSize[2];

	descriptorPoolSize[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	descriptorPoolSize[0].descriptorCount = 100;

	descriptorPoolSize[1].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorPoolSize[1].descriptorCount = 100;

	// we only need to allocate one descriptor set from the pool.
	VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
	descriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	descriptorPoolCreateInfo.maxSets = 1000; 
	descriptorPoolCreateInfo.poolSizeCount = 2;
	descriptorPoolCreateInfo.pPoolSizes = descriptorPoolSize;
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
	snprintf(outPathName, sizeof (appPath), "%ssdk/dMedia/dbrain/%s", appPath, name);
#elif defined(__APPLE__)
	char tmp[2048];
	CFURLRef appURL(CFBundleCopyBundleURL(CFBundleGetMainBundle()));
	CFStringRef filePath(CFURLCopyFileSystemPath(appURL, kCFURLPOSIXPathStyle));
	CFStringGetCString(filePath, tmp, PATH_MAX, kCFStringEncodingUTF8);
	//char* const ptr = strstr (tmp, "applications");
	//ptr [0] = 0;
	snprintf(outPathName, sizeof(tmp), "%s/Contents/Resources/%s", tmp, name);

	// Clean up 
	CFRelease(appURL);
	CFRelease(filePath);
#elif defined(__linux__)
	char id[1024];
	char appPath[1024];

	snprintf(id, sizeof (id), "/proc/%d/exe", getpid());
	memset(appPath, 0, sizeof(appPath));
	size_t ret = readlink(id, appPath, sizeof(appPath));
	ret = 0;
	char* const end = strstr(appPath, "applications");
	*end = 0;
	snprintf(outPathName, sizeof(appPath), "%ssdk/dMedia/dbrain/%s", appPath, name);
#else
#error  "error: need to implement \"dGetWorkingFileName\" here for this platform"
#endif
}


VkShaderModule ndBrainGpuContext::LoadShaderProgram(const char* const name)
{
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

	LoadShaderCode(name);
	VkShaderModuleCreateInfo createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.pCode = (uint32_t*)&code[0];
	createInfo.codeSize = size_t(code.GetCount());

	VkShaderModule computeShaderModule;
	CheckResultVulkan(vkCreateShaderModule(m_device, &createInfo, m_allocator, &computeShaderModule));
	return computeShaderModule;
}

void ndBrainGpuContext::LoadShaderPrograms()
{
	VkShaderModule clean (VK_NULL_HANDLE);
	ndMemSet(m_modules, clean, sizeof(m_modules) / sizeof(m_modules[0]));

	m_testShader = LoadShaderProgram("testShader-comp.spv");
	m_ndBrainCopyInput = LoadShaderProgram("ndBrainCopyInput-comp.spv");
	m_ndBrainCopyBuffer = LoadShaderProgram("ndBrainCopyBuffer-comp.spv");
	m_ndBrainGetResults = LoadShaderProgram("ndBrainGetResults-comp.spv");
	//m_ndBrainLayerLinear = LoadShaderProgram("ndBrainLayerLinear-comp.spv");
	m_ndBrainLayerLinearTiled = LoadShaderProgram("ndBrainLayerLinearTiled-comp.spv");
	m_ndBrainLayerEluActivation = LoadShaderProgram("ndBrainLayerEluActivation-comp.spv");
	m_ndBrainLayerReluActivation = LoadShaderProgram("ndBrainLayerReluActivation-comp.spv");
	m_ndBrainLayerTanhActivation = LoadShaderProgram("ndBrainLayerTanhActivation-comp.spv");
	m_ndBrainLayerSoftmaxActivation = LoadShaderProgram("ndBrainLayerSoftmaxActivation-comp.spv");
	m_ndBrainLayerSoftmaxActivationSubGroup = LoadShaderProgram("ndBrainLayerSoftmaxActivationSubGroup-comp.spv");
}

void ndBrainGpuContext::SubmitQueue(const ndList<ndSharedPtr<ndBrainGpuCommand>>& displayList)
{
	ndAssert(!m_queueInProgress);
	m_queueInProgress = true;
	m_displayList.SetCount(0);
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = displayList.GetFirst(); node; node = node->GetNext())
	{
		ndBrainGpuCommand* command = *node->GetInfo();
		m_displayList.PushBack(command->m_commandBuffer);
	}

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = uint32_t(m_displayList.GetCount());
	submitInfo.pCommandBuffers = &m_displayList[0];
	CheckResultVulkan(vkQueueSubmit(m_queue, uint32_t(1), &submitInfo, m_fence));
}

void ndBrainGpuContext::Sync()
{
	if (m_queueInProgress)
	{
		CheckResultVulkan(vkWaitForFences(m_device, uint32_t(1), &m_fence, VK_TRUE, 100000000000));
	}
	m_queueInProgress = false;
}

#endif