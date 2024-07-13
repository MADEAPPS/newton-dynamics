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
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuIntegerBuffer.h"
#include "ndBrainGpuUniformBuffer.h"

#if defined (D_USE_VULKAN_SDK)

ndBrainGpuCommand::ndBrainGpuCommand(ndBrainGpuContext* const context)
	:ndClassAlloc()
	,m_context(context)
	,m_pipeline(VK_NULL_HANDLE)
	,m_descriptorSet(VK_NULL_HANDLE)
	,m_commandBuffer(VK_NULL_HANDLE)
	,m_pipelineLayout(VK_NULL_HANDLE)
	,m_descriptorSetLayout(VK_NULL_HANDLE)
{
}

ndBrainGpuCommand::~ndBrainGpuCommand()
{
	if (m_descriptorSetLayout)
	{
		vkDestroyDescriptorSetLayout(m_context->m_device, m_descriptorSetLayout, m_context->m_allocator);
		vkDestroyPipelineLayout(m_context->m_device, m_pipelineLayout, m_context->m_allocator);
		vkDestroyPipeline(m_context->m_device, m_pipeline, m_context->m_allocator);
	}
}

void ndBrainGpuCommand::Assembly(void* const shaderHandle, ndInt32 workGroups, ndInt32 paramCount, ndBrainGpuBuffer** params)
{
	VkDevice const device = m_context->GetDevice();
	VkShaderModule shader = (VkShaderModule)shaderHandle;
	VkCommandPool const commandPool = m_context->GetCommandPool();
	VkAllocationCallbacks* const allocator = m_context->GetAllocator();
	VkDescriptorPool const descriptorPool = m_context->GetDescriptorPool();

	VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
	commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	commandBufferAllocateInfo.commandPool = commandPool;
	commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	commandBufferAllocateInfo.commandBufferCount = 1;
	m_context->CheckResultVulkan(vkAllocateCommandBuffers(device, &commandBufferAllocateInfo, &m_commandBuffer));
	
	ndFixSizeArray<VkDescriptorSetLayoutBinding, 32> layoutBinding;
	for (ndInt32 i = 0; i < paramCount; ++i)
	{
		VkDescriptorSetLayoutBinding binding = {};

		binding.binding = uint32_t(i);
		binding.descriptorType = (params[i]->GetType() == ndUniformData) ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		binding.descriptorCount = 1;
		binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		layoutBinding.PushBack(binding);
	}
	VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
	descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	descriptorSetLayoutCreateInfo.bindingCount = uint32_t(layoutBinding.GetCount());
	descriptorSetLayoutCreateInfo.pBindings = &layoutBinding[0];
	m_context->CheckResultVulkan(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, allocator, &m_descriptorSetLayout));
	
	VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
	descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	descriptorSetAllocateInfo.descriptorPool = descriptorPool;
	descriptorSetAllocateInfo.descriptorSetCount = 1;
	descriptorSetAllocateInfo.pSetLayouts = &m_descriptorSetLayout;
	m_context->CheckResultVulkan(vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, &m_descriptorSet));

	ndFixSizeArray<VkWriteDescriptorSet, 32> descriptorSet;
	ndFixSizeArray<VkDescriptorBufferInfo, 32> bufferInfo;
	for (ndInt32 i = 0; i < paramCount; ++i)
	{
		VkDescriptorBufferInfo descriptorBufferInfo = {};
		descriptorBufferInfo.buffer = params[i]->GetBuffer();
		descriptorBufferInfo.offset = 0;
		descriptorBufferInfo.range = (VkDeviceSize)params[i]->SizeInBytes();
		bufferInfo.PushBack(descriptorBufferInfo);

		VkWriteDescriptorSet set = {};
		set.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		set.dstSet = m_descriptorSet;
		set.dstBinding = uint32_t(i);
		set.descriptorCount = 1;
		//set.descriptorType = params[i]->GetType();
		set.descriptorType = (params[i]->GetType() == ndUniformData) ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		set.pBufferInfo = &bufferInfo[i];
		descriptorSet.PushBack(set);
	}
	vkUpdateDescriptorSets(device, uint32_t(descriptorSet.GetCount()), &descriptorSet[0], 0, nullptr);
	
	VkPipelineShaderStageCreateInfo shaderStageCreateInfo = {};
	shaderStageCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	shaderStageCreateInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	shaderStageCreateInfo.module = shader;
	shaderStageCreateInfo.pName = "main";
	
	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
	pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutCreateInfo.setLayoutCount = 1;
	pipelineLayoutCreateInfo.pSetLayouts = &m_descriptorSetLayout;
	m_context->CheckResultVulkan(vkCreatePipelineLayout(device, &pipelineLayoutCreateInfo, allocator, &m_pipelineLayout));
	
	VkComputePipelineCreateInfo pipelineCreateInfo = {};
	pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineCreateInfo.stage = shaderStageCreateInfo;
	pipelineCreateInfo.layout = m_pipelineLayout;
	m_context->CheckResultVulkan(vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipelineCreateInfo, allocator, &m_pipeline));
	
	VkCommandBufferBeginInfo beginInfo = {};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	//beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
	m_context->CheckResultVulkan(vkBeginCommandBuffer(m_commandBuffer, &beginInfo)); // start recording commands.
	
	vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
	vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);
	vkCmdDispatch(m_commandBuffer, uint32_t(workGroups), 1, 1);

	//VkMemoryBarrier memoryBarrier = {};
	//memoryBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
	//memoryBarrier.pNext = VK_NULL_HANDLE;
	//memoryBarrier.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
	//memoryBarrier.dstAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
	//vkCmdPipelineBarrier(m_commandBuffer,
	//	VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
	//	VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
	//	0,
	//	0, &memoryBarrier,
	//	0, VK_NULL_HANDLE,
	//	0, VK_NULL_HANDLE);

	m_context->CheckResultVulkan(vkEndCommandBuffer(m_commandBuffer));
}
#endif

ndBrainGpuCommandTest::ndBrainGpuCommandTest(ndBrainGpuContext* const context,
	ndBrainGpuUniformBuffer& parammeters,
	ndBrainGpuFloatBuffer& input, ndBrainGpuFloatBuffer& output)
	:ndBrainGpuCommand(context)
{
	ndBrainGpuBuffer* params[3];
	params[0] = &parammeters;
	params[1] = &input;
	params[2] = &output;
#if defined (D_USE_VULKAN_SDK)
	Assembly(context->m_testShader, 1, 3, params);
#endif
};