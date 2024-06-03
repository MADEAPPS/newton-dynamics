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
#ifndef __ND_BRAIN_GPU_CONTEXT_H__
#define __ND_BRAIN_GPU_CONTEXT_H__

class ndBrainGpuCommand;
class ndBrainGpuFloatBuffer;

#if !defined (D_USE_VULKAN_SDK)

class ndBrainGpuContext : public ndClassAlloc
{
	public:
	ndBrainGpuContext(){}
	virtual ~ndBrainGpuContext(){}
	void SubmitQueue(ndBrainGpuCommand**, ndInt32) {}
	static bool HasGpuSupport(){return false;}
};

#else
class ndBrainGpuContext: public ndClassAlloc
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();

	static bool HasGpuSupport();

	VkDevice GetDevice() const;
	VkCommandPool GetCommandPool() const;
	VkDescriptorPool GetDescriptorPool() const;
	VkAllocationCallbacks* GetAllocator() const;
	VkPhysicalDevice GetPhysicalDevice() const;

	void SubmitQueue(ndBrainGpuCommand** commands, ndInt32 commandCount);
	static void CheckResultVulkan(VkResult err);

	private:
	void CreateInstance();
	void CreateCommandPool();
	void SetupDebugMessenger();
	void SelectGraphicsQueue();
	void CreateLogicalDevice();
	void CreatePhysicalDevice();
	void CreateDescriptorPool();
	void CreateFence();
	void LoadShaderPrograms();
	VkShaderModule LoadShaderProgram(const char* const name);
	void GetShaderFileName(const char* const name, char* const outPathName);

	static void VulkanFree(void* pUserData, void* memory);
	static void* VulkanAlloc(void* pUserData, size_t size, size_t alignment, VkSystemAllocationScope allocationScope);
	static void* VulkanRealloc(void* pUserData, void* pOriginal, size_t size, size_t alignment, VkSystemAllocationScope allocationScope);
	static void VulkanInternalFree(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope);
	static void VulkanInternalAlloc(void* pUserData, size_t size, VkInternalAllocationType allocationType, VkSystemAllocationScope allocationScope);

	static VKAPI_ATTR VkBool32 VKAPI_CALL DebugReportVulkan(
		VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objectType,
		uint64_t object, size_t location, int32_t messageCode,
		const char* pLayerPrefix, const char* pMessage, void* pUserData);


	VkAllocationCallbacks m_allocatorStruct;
	VkAllocationCallbacks* m_allocator;
	VkFence m_fence;
	VkQueue m_queue;
	VkDevice m_device;
	VkInstance m_instance;
	VkCommandPool m_commandPool;
	VkDescriptorPool m_descriptorPool;
	VkPhysicalDevice m_physicalDevice;
	VkPhysicalDeviceProperties m_gpuProps;
	VkDebugReportCallbackEXT m_debugMessenger;

public:
	VkShaderModule m_copyInputData;
	VkShaderModule m_computeShaderModule0;
	VkShaderModule m_computeShaderModule1;

	uint32_t m_queueFamilyIndex;
	bool m_hasValidationLayers;
	static const char* m_apiLayers[];
	static const char* m_apiExtensionLayers[];

	friend class ndBrainGpuCommand;
};

#endif


#endif