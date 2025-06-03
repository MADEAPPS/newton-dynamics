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

#include "ndBrainStdafx.h"
#include "ndBrainContext.h"

class ndBrainGpuCommand;
class ndBrainGpuFloatBuffer;

typedef VkShaderModule ndVulkanShader;

class ndBrainGpuContext : public ndBrainContext
{
	class ndMemoryEntry
	{
		public:
		void* m_ptr;
		size_t m_size;
	};

	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();

	static bool HasGpuSupport();

	VkDevice GetDevice() const;
	ndInt32 GetSubGroupSize() const;
	virtual ndContextType GetType() const override;
	virtual ndBrainGpuContext* GetAsGpuContext() override;

	VkCommandPool GetCommandPool() const;
	VkDescriptorPool GetDescriptorPool() const;
	VkAllocationCallbacks* GetAllocator() const;
	VkPhysicalDevice GetPhysicalDevice() const;

	void SyncQueue();
	void AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>& command);
	
	static void CheckResultVulkan(VkResult err);

	private:
	void CreateInstance();
	void CreateCommandPool();
	void SetupDebugMessenger();
	void SelectGraphicsQueue();
	void CreateLogicalDevice();
	void CreatePhysicalDevice();
	void CheckSubGroupSupport();
	void CreateDescriptorPool();
	void CreateFence();
	void LoadShaderPrograms();
	ndVulkanShader LoadShaderProgram(const char* const name);
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

	ndArray<VkCommandBuffer> m_displayList;
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
	ndTree<ndMemoryEntry, void*> m_memoryDictionary;

	public:
	union
	{
		struct
		{
			ndVulkanShader m_ndBrainCopyInput;
			ndVulkanShader m_ndBrainCopyOutput;
			ndVulkanShader m_ndBrainLayerLinear;
			ndVulkanShader m_ndBrainCopyOutputGradients;
			ndVulkanShader m_ndBrainLayerReluActivation;
			ndVulkanShader m_ndBrainLayerTanhActivation;
			ndVulkanShader m_ndBrainLayerSoftmaxActivation;
			ndVulkanShader m_ndBrainLayerLinearDropOutActivation;
		};
		ndVulkanShader m_modules[128];
	};

	ndInt32 m_subGroupSize;
	uint32_t m_queueFamilyIndex;
	bool m_hasValidationLayers;

	static const char* m_apiLayers[];
	static const char* m_apiExtensionLayers[];

	friend class ndBrainGpuCommand;
};

#endif