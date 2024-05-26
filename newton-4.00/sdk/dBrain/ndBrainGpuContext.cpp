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
#include "ndBrainGpuContext.h"


#define ND_SELECT_DISCRETE_GPU


class ndBrainGpuContext::ndBrainGpuContext::ndImplementation : public ndClassAlloc
{
	public:
	ndImplementation();
	~ndImplementation();

	void CreateInstance();
	void SetupDebugMessenger();
	void CreatePhysicalDevice();
	void SelectGraphicsQueue();
	void CreateLogicalDevice();
	//void CreateDescriptorPools();
	void CreateDescriptorSetLayout();
	void CreateDescriptorSet();
	void CreateComputePipeline();
	void CreateCommandBuffer();

	static void CheckResultVulkan(VkResult err);
	static void VulkanFree(void* pUserData, void* memory);
	static void* VulkanAlloc(void* pUserData, size_t size, size_t alignment, VkSystemAllocationScope allocationScope);
	static void* VulkanRealloc(void* pUserData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope);
	static void VulkanInternalFree(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope);
	static void VulkanInternalAlloc(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope);

	static VKAPI_ATTR VkBool32 VKAPI_CALL DebugReportVulkan(
		VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objectType,
		uint64_t object, size_t location, int32_t messageCode,
		const char* pLayerPrefix, const char* pMessage, void* pUserData);

	void GetShaderFileName(const char* const name, char* const outPathName);

	VkAllocationCallbacks m_allocatorStruct;
	VkAllocationCallbacks* m_allocator;
	VkDevice m_device;
	VkQueue m_queue;
	VkInstance m_instance;
	VkDescriptorSet m_descriptorSet;
	VkDescriptorPool m_descriptorPool;
	VkDescriptorSetLayout m_descriptorSetLayout;
	VkDebugReportCallbackEXT m_debugMessenger;
	VkPhysicalDevice m_physicalDevice;
	VkPhysicalDeviceProperties m_gpuProps;

	VkPipeline m_pipeline;
	VkPipelineLayout m_pipelineLayout;
	VkShaderModule m_computeShaderModule;

	VkCommandPool m_commandPool;
	VkCommandBuffer m_commandBuffer;


	uint32_t m_queueFamilyIndex;
	bool m_hasValidationLayers;
	static const char* m_apiLayers[];
	static const char* m_apiExtensionLayers[];
};

const char* ndBrainGpuContext::ndBrainGpuContext::ndImplementation::m_apiLayers[] =
{
	"VK_LAYER_KHRONOS_validation"
};

const char* ndBrainGpuContext::ndBrainGpuContext::ndImplementation::m_apiExtensionLayers[] =
{
	"VK_EXT_debug_report"
};

ndBrainGpuContext::ndBrainGpuContext()
	:ndClassAlloc()
	,m_context(nullptr)
{
}

ndBrainGpuContext::~ndBrainGpuContext()
{
	if (m_context)
	{
		delete m_context;
	}
}

ndInt32 ndBrainGpuContext::Init()
{
	ndAssert(!m_context);
	m_context = new ndImplementation;
	return 0;
}

void* ndBrainGpuContext::GetDevice() const
{
	return m_context->m_device;
}

void* ndBrainGpuContext::GetPhysicalDevice() const
{
	return m_context->m_physicalDevice;
}

void* ndBrainGpuContext::GetAllocator() const
{
	return m_context->m_allocator;
}

//*************************************************************************************
//
//*************************************************************************************
ndBrainGpuContext::ndBrainGpuContext::ndImplementation::ndImplementation()
	:ndClassAlloc()
	,m_allocator(&m_allocatorStruct)
	,m_device(VK_NULL_HANDLE)
	,m_queue(VK_NULL_HANDLE)
	,m_instance(VK_NULL_HANDLE)
	,m_descriptorSet(VK_NULL_HANDLE)
	,m_descriptorPool(VK_NULL_HANDLE)
	,m_descriptorSetLayout(VK_NULL_HANDLE)
	,m_debugMessenger(VK_NULL_HANDLE)
	,m_physicalDevice(VK_NULL_HANDLE)
	,m_queueFamilyIndex(0xffffffff)
	,m_hasValidationLayers(false)
{
	m_allocatorStruct.pUserData = this;
	m_allocatorStruct.pfnFree = VulkanFree;
	m_allocatorStruct.pfnAllocation = VulkanAlloc;
	m_allocatorStruct.pfnReallocation = VulkanRealloc;
	m_allocatorStruct.pfnInternalFree = VulkanInternalFree;
	m_allocatorStruct.pfnInternalAllocation = VulkanInternalAlloc;

	// Create Vulkan Instance
	CreateInstance();

	// Set Debug layer utility
	SetupDebugMessenger();

	// Select GPU
	CreatePhysicalDevice();

	// Select graphics queue family
	SelectGraphicsQueue();
	
	// Create Logical Device (with 1 queue)
	CreateLogicalDevice();

	// Create descriptor layout
	CreateDescriptorSetLayout();

	// Create descriptor
	CreateDescriptorSet();

	// Create Descriptor Pool
	//CreateDescriptorPools();

	// Create Compute Pipelines
	CreateComputePipeline();

	// Create the commandBuffer
	CreateCommandBuffer();
}

ndBrainGpuContext::ndBrainGpuContext::ndImplementation::~ndImplementation()
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

	vkDestroyCommandPool(m_device, m_commandPool, m_allocator);
	vkDestroyShaderModule(m_device, m_computeShaderModule, m_allocator);
	vkDestroyDescriptorSetLayout(m_device, m_descriptorSetLayout, m_allocator);
	vkDestroyPipelineLayout(m_device, m_pipelineLayout, m_allocator);
	vkDestroyPipeline(m_device, m_pipeline, m_allocator);
	vkDestroyDescriptorPool(m_device, m_descriptorPool, m_allocator);
	vkDestroyDevice(m_device, m_allocator);
	vkDestroyInstance(m_instance, m_allocator);
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::SetupDebugMessenger()
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

void* ndBrainGpuContext::ndBrainGpuContext::ndImplementation::VulkanAlloc(void*, size_t size, size_t alignment, VkSystemAllocationScope)
{
	ndAssert(alignment);
	ndAssert(alignment <= D_MEMORY_ALIGMNET);
	void* const ptr = ndMemory::Malloc(size);
	return ptr;
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::VulkanFree(void*, void* memory)
{
	ndMemory::Free(memory);
}

void* ndBrainGpuContext::ndBrainGpuContext::ndImplementation::VulkanRealloc(void* pUserData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope)
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

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::VulkanInternalAlloc(void*, size_t, VkInternalAllocationType, VkSystemAllocationScope)
{
	ndAssert(0);
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::VulkanInternalFree(void*, size_t, VkInternalAllocationType, VkSystemAllocationScope)
{
	ndAssert(0);
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CheckResultVulkan(VkResult err)
{
	if (err != VK_SUCCESS)
	{
		ndTrace(("[vulkan] Error: VkResult = %d\n", err));
		if (err < 0)
		{
			abort();
		}
	}
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::GetShaderFileName(const char* const name, char* const outPathName)
{
	#if (defined(WIN32) || defined(_WIN32))
		char appPath[256];
		GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
		strtolwr(appPath);

		char* const end = strstr(appPath, "applications");
		end[0] = 0;
		sprintf(outPathName, "%sapplications/media/dBrain/%s", appPath, name);
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
		sprintf(outPathName, "%sapplications/media/dBrain/%s", appPath, name);
	#else
		#error  "error: need to implement \"dGetWorkingFileName\" here for this platform"
	#endif
}

VKAPI_ATTR VkBool32 VKAPI_CALL ndBrainGpuContext::ndBrainGpuContext::ndImplementation::DebugReportVulkan(
	//VkDebugReportFlagsEXT                       flags,
	//VkDebugReportObjectTypeEXT                  objectType,
	//uint64_t                                    object,
	//size_t                                      location,
	//int32_t                                     messageCode,
	//const char* pLayerPrefix,
	//const char* pMessage,
	//void* pUserData) 
	VkDebugReportFlagsEXT, VkDebugReportObjectTypeEXT, uint64_t, size_t, int32_t,
	const char* pLayerPrefix, const char* pMessage,	void*) 
{
	ndTrace (("Debug Report: %s: %s\n", pLayerPrefix, pMessage));

	return VK_FALSE;
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreateInstance()
{
	uint32_t layerCount;
	ndFixSizeArray <VkLayerProperties,128> layerProperties;
	vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
	layerProperties.SetCount(ndInt32 (layerCount));
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

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreatePhysicalDevice()
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
	for (ndInt32 i = 0; i < (int)gpuCount; i++)
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

	//ndFixSizeArray<const char*, 128> extensionsNames;
	//bool swapchainExtFound = false;
	//uint32_t deviceExtensionCount;
	//CheckResultVulkan(vkEnumerateDeviceExtensionProperties(m_physicalDevice, nullptr, &deviceExtensionCount, nullptr));
	//ndAssert(deviceExtensionCount < 512);
	//ndFixSizeArray<VkExtensionProperties, 512> deviceExtensions;
	//deviceExtensions.SetCount(ndInt32(deviceExtensionCount));
	//CheckResultVulkan(vkEnumerateDeviceExtensionProperties(m_physicalDevice, nullptr, &deviceExtensionCount, &deviceExtensions[0]));
	//for (ndInt32 i = 0; i < ndInt32(deviceExtensionCount); i++)
	//{
	//	const char* const extName = deviceExtensions[i].extensionName;
	//	if (!strcmp(VK_KHR_SWAPCHAIN_EXTENSION_NAME, extName))
	//	{
	//		swapchainExtFound = true;
	//		//extensionsNames.PushBack(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
	//	}
	//	//if (!strcmp("VK_KHR_portability_subset", extName))
	//	//{
	//	//	extensionsNames.PushBack("VK_KHR_portability_subset");
	//	//}
	//}
	//ndAssert(swapchainExtFound);
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::SelectGraphicsQueue()
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

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreateLogicalDevice()
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

//void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreateDescriptorPools()
//{
//	m_descriptorPool = VK_NULL_HANDLE;
//	VkDescriptorPoolSize pool_sizes[] =
//	{
//		//{ VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
//		//{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
//		//{ VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
//		//{ VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
//		//{ VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
//		//{ VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
//		{ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
//		{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
//		{ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
//		{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
//		//{ VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 }
//	};
//	VkDescriptorPoolCreateInfo pool_info = {};
//	pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
//	pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
//	//pool_info.maxSets = 1000 * IM_ARRAYSIZE(pool_sizes);
//	//pool_info.poolSizeCount = (uint32_t)IM_ARRAYSIZE(pool_sizes);
//	pool_info.maxSets = 1 * sizeof(pool_sizes) / sizeof(pool_sizes[0]);
//	pool_info.poolSizeCount = sizeof(pool_sizes) / sizeof(pool_sizes[0]);;
//	pool_info.pPoolSizes = pool_sizes;
//	CheckResultVulkan(vkCreateDescriptorPool(m_device, &pool_info, m_allocator, &m_descriptorPool));
//
//	////With the pool allocated, we can now allocate the descriptor set.
//	//VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
//	//descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
//	//descriptorSetAllocateInfo.descriptorPool = m_descriptorPool; // pool to allocate from.
//	//descriptorSetAllocateInfo.descriptorSetCount = 1; // allocate a single descriptor set.
//	//descriptorSetAllocateInfo.pSetLayouts = &m_descriptorSetLayout;
//	//
//	//// allocate descriptor set.
//	//CheckResultVulkan(vkAllocateDescriptorSets(m_device, &descriptorSetAllocateInfo, &m_vulkanDescriptorSet));
//	//
//	////Next, we need to connect our actual storage buffer with the descrptor.
//	////We use vkUpdateDescriptorSets() to update the descriptor set.
//	//// Specify the buffer to bind to the descriptor.
//	//
//	//VkBuffer buffer;
//	//uint32_t bufferSize; 
//	//VkDescriptorBufferInfo descriptorBufferInfo = {};
//	//descriptorBufferInfo.buffer = buffer;
//	//descriptorBufferInfo.offset = 0;
//	//descriptorBufferInfo.range = bufferSize;
//	//
//	//VkWriteDescriptorSet writeDescriptorSet = {};
//	//writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
//	//writeDescriptorSet.dstSet = descriptorSet; // write to this descriptor set.
//	//writeDescriptorSet.dstBinding = 0; // write to the first, and only binding.
//	//writeDescriptorSet.descriptorCount = 1; // update a single descriptor.
//	//writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; // storage buffer.
//	//writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
//	//
//	//// perform the update of the descriptor set.
//	//vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, NULL);
//}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreateDescriptorSetLayout()
{
	VkDescriptorSetLayoutBinding descriptorSetLayoutBinding = {};
	descriptorSetLayoutBinding.binding = 0; // binding = 0
	descriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorSetLayoutBinding.descriptorCount = 1;
	descriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

	VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
	descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	descriptorSetLayoutCreateInfo.bindingCount = 1; // only a single binding in this descriptor set layout. 
	descriptorSetLayoutCreateInfo.pBindings = &descriptorSetLayoutBinding;

	// Create the descriptor set layout. 
	CheckResultVulkan(vkCreateDescriptorSetLayout(m_device, &descriptorSetLayoutCreateInfo, m_allocator, &m_descriptorSetLayout));
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreateComputePipeline()
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
	CheckResultVulkan(vkCreateShaderModule(m_device, &createInfo, m_allocator, &m_computeShaderModule));
	
	//Now let us actually create the compute pipeline.
	//A compute pipeline is very simple compared to a graphics pipeline.
	//It only consists of a single stage with a compute shader.
	//So first we specify the compute shader stage, and it's entry point(main).
	VkPipelineShaderStageCreateInfo shaderStageCreateInfo = {};
	shaderStageCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	shaderStageCreateInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	shaderStageCreateInfo.module = m_computeShaderModule;
	shaderStageCreateInfo.pName = "main";

	//The pipeline layout allows the pipeline to access descriptor sets.
	//So we just specify the descriptor set layout we created earlier.
	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
	pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutCreateInfo.setLayoutCount = 1;
	pipelineLayoutCreateInfo.pSetLayouts = &m_descriptorSetLayout;
	CheckResultVulkan(vkCreatePipelineLayout(m_device, &pipelineLayoutCreateInfo, m_allocator, &m_pipelineLayout));
	
	VkComputePipelineCreateInfo pipelineCreateInfo = {};
	pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineCreateInfo.stage = shaderStageCreateInfo;
	pipelineCreateInfo.layout = m_pipelineLayout;
	CheckResultVulkan(vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineCreateInfo, m_allocator, &m_pipeline));
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreateCommandBuffer()
{
	VkCommandPoolCreateInfo commandPoolCreateInfo = {};
	commandPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	commandPoolCreateInfo.flags = 0;
	// the queue family of this command pool. All command buffers allocated from this command pool,
	// must be submitted to queues of this family ONLY. 
	commandPoolCreateInfo.queueFamilyIndex = m_queueFamilyIndex;
	CheckResultVulkan(vkCreateCommandPool(m_device, &commandPoolCreateInfo, m_allocator, &m_commandPool));
	
	VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
	commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	commandBufferAllocateInfo.commandPool = m_commandPool; // specify the command pool to allocate from. 
	// if the command buffer is primary, it can be directly submitted to queues. 
	// A secondary buffer has to be called from some primary command buffer, and cannot be directly 
	// submitted to a queue. To keep things simple, we use a primary command buffer. 
	commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	commandBufferAllocateInfo.commandBufferCount = 1; // allocate a single command buffer. 
	CheckResultVulkan(vkAllocateCommandBuffers(m_device, &commandBufferAllocateInfo, &m_commandBuffer)); // allocate command buffer.
	
	VkCommandBufferBeginInfo beginInfo = {};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; // the buffer is only submitted and used once in this application.
	CheckResultVulkan(vkBeginCommandBuffer(m_commandBuffer, &beginInfo)); // start recording commands.

	vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
	vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);

	//vkCmdDispatch(commandBuffer, (uint32_t)ceil(WIDTH / float(WORKGROUP_SIZE)), (uint32_t)ceil(HEIGHT / float(WORKGROUP_SIZE)), 1);
	//CheckResultVulkan(vkEndCommandBuffer(commandBuffer)); // end recording commands.
}

void ndBrainGpuContext::ndBrainGpuContext::ndImplementation::CreateDescriptorSet()
{
	VkDescriptorPoolSize descriptorPoolSize = {};
	descriptorPoolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorPoolSize.descriptorCount = 1;

	VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
	descriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	descriptorPoolCreateInfo.maxSets = 1; // we only need to allocate one descriptor set from the pool.
	descriptorPoolCreateInfo.poolSizeCount = 1;
	descriptorPoolCreateInfo.pPoolSizes = &descriptorPoolSize;
	CheckResultVulkan(vkCreateDescriptorPool(m_device, &descriptorPoolCreateInfo, m_allocator, &m_descriptorPool));

	VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
	descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	descriptorSetAllocateInfo.descriptorPool = m_descriptorPool; // pool to allocate from.
	descriptorSetAllocateInfo.descriptorSetCount = 1; // allocate a single descriptor set.
	descriptorSetAllocateInfo.pSetLayouts = &m_descriptorSetLayout;

	// allocate descriptor set.
	CheckResultVulkan(vkAllocateDescriptorSets(m_device, &descriptorSetAllocateInfo, &m_descriptorSet));

	//// Specify the buffer to bind to the descriptor.
	//VkDescriptorBufferInfo descriptorBufferInfo = {};
	//descriptorBufferInfo.buffer = buffer;
	//descriptorBufferInfo.offset = 0;
	//descriptorBufferInfo.range = bufferSize;
	//
	//VkWriteDescriptorSet writeDescriptorSet = {};
	//writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	//writeDescriptorSet.dstSet = m_descriptorSet; // write to this descriptor set.
	//writeDescriptorSet.dstBinding = 0; // write to the first, and only binding.
	//writeDescriptorSet.descriptorCount = 1; // update a single descriptor.
	//writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; // storage buffer.
	//writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
	//
	//// perform the update of the descriptor set.
	//vkUpdateDescriptorSets(m_device, 1, &writeDescriptorSet, 0, nullptr);
}


void ndBrainGpuContext::ExecuteTest(ndBrainGpuFloatBuffer& buffer)
{

}