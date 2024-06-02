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

ndBrainGpuCommandTest0::ndBrainGpuCommandTest0(ndBrainGpuContext* const context, ndBrainGpuFloatBuffer& buffer)
	:ndBrainGpuCommand(context)
{
	VkDevice device = m_context->GetDevice();
	VkCommandPool commandPool = m_context->GetCommandPool();
	VkAllocationCallbacks* allocator = m_context->GetAllocator();
	VkDescriptorPool descriptorPool = m_context->GetDescriptorPool();

	VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
	commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	commandBufferAllocateInfo.commandPool = commandPool;
	commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	commandBufferAllocateInfo.commandBufferCount = 1;
	m_context->CheckResultVulkan(vkAllocateCommandBuffers(device, &commandBufferAllocateInfo, &m_commandBuffer));

	// binding = 0
	VkDescriptorSetLayoutBinding descriptorSetLayoutBinding = {};
	descriptorSetLayoutBinding.binding = 0; 
	descriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorSetLayoutBinding.descriptorCount = 1;
	descriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	
	// only a single binding in this descriptor set layout. 
	VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
	descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	descriptorSetLayoutCreateInfo.bindingCount = 1; 
	descriptorSetLayoutCreateInfo.pBindings = &descriptorSetLayoutBinding;
	m_context->CheckResultVulkan(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, allocator, &m_descriptorSetLayout));
	
	VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
	descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	descriptorSetAllocateInfo.descriptorPool = descriptorPool;
	descriptorSetAllocateInfo.descriptorSetCount = 1;
	descriptorSetAllocateInfo.pSetLayouts = &m_descriptorSetLayout;
	m_context->CheckResultVulkan(vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, &m_descriptorSet));
	
	// Specify the buffer to bind to the descriptor.
	VkDescriptorBufferInfo descriptorBufferInfo = {};
	descriptorBufferInfo.buffer = buffer.GetBuffer();
	descriptorBufferInfo.offset = 0;
	descriptorBufferInfo.range = (VkDeviceSize)buffer.SizeInBytes();
	
	VkWriteDescriptorSet writeDescriptorSet = {};
	writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	writeDescriptorSet.dstSet = m_descriptorSet; // write to this descriptor set.
	writeDescriptorSet.dstBinding = 0; // write to the first, and only binding.
	writeDescriptorSet.descriptorCount = 1; // update a single descriptor.
	writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; // storage buffer.
	writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;
	
	// perform the update of the descriptor set.
	vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);
	
	
	VkPipelineShaderStageCreateInfo shaderStageCreateInfo = {};
	shaderStageCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	shaderStageCreateInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	shaderStageCreateInfo.module = context->m_computeShaderModule0;
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
	m_context->CheckResultVulkan(vkBeginCommandBuffer(m_commandBuffer, &beginInfo)); // start recording commands.
	
	vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
	vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);
	////vkCmdDispatch(commandBuffer, (uint32_t)ceil(WIDTH / float(WORKGROUP_SIZE)), (uint32_t)ceil(HEIGHT / float(WORKGROUP_SIZE)), 1);
	vkCmdDispatch(m_commandBuffer, 1, 1, 1);
	m_context->CheckResultVulkan(vkEndCommandBuffer(m_commandBuffer));
}


ndBrainGpuCommandTest1::ndBrainGpuCommandTest1(ndBrainGpuContext* const context, ndBrainGpuFloatBuffer& buffer)
	:ndBrainGpuCommand(context)
{
	VkDevice device = m_context->GetDevice();
	VkCommandPool commandPool = m_context->GetCommandPool();
	VkAllocationCallbacks* allocator = m_context->GetAllocator();
	VkDescriptorPool descriptorPool = m_context->GetDescriptorPool();

	VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
	commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	commandBufferAllocateInfo.commandPool = commandPool;
	commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	commandBufferAllocateInfo.commandBufferCount = 1;
	m_context->CheckResultVulkan(vkAllocateCommandBuffers(device, &commandBufferAllocateInfo, &m_commandBuffer));

	// binding = 0
	VkDescriptorSetLayoutBinding descriptorSetLayoutBinding = {};
	descriptorSetLayoutBinding.binding = 0;
	descriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorSetLayoutBinding.descriptorCount = 1;
	descriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

	// only a single binding in this descriptor set layout. 
	VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
	descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	descriptorSetLayoutCreateInfo.bindingCount = 1;
	descriptorSetLayoutCreateInfo.pBindings = &descriptorSetLayoutBinding;
	m_context->CheckResultVulkan(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, allocator, &m_descriptorSetLayout));

	VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
	descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	descriptorSetAllocateInfo.descriptorPool = descriptorPool;
	descriptorSetAllocateInfo.descriptorSetCount = 1;
	descriptorSetAllocateInfo.pSetLayouts = &m_descriptorSetLayout;
	m_context->CheckResultVulkan(vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, &m_descriptorSet));

	// Specify the buffer to bind to the descriptor.
	VkDescriptorBufferInfo descriptorBufferInfo = {};
	descriptorBufferInfo.buffer = buffer.GetBuffer();
	descriptorBufferInfo.offset = 0;
	descriptorBufferInfo.range = (VkDeviceSize)buffer.SizeInBytes();

	VkWriteDescriptorSet writeDescriptorSet = {};
	writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	writeDescriptorSet.dstSet = m_descriptorSet; // write to this descriptor set.
	writeDescriptorSet.dstBinding = 0; // write to the first, and only binding.
	writeDescriptorSet.descriptorCount = 1; // update a single descriptor.
	writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; // storage buffer.
	writeDescriptorSet.pBufferInfo = &descriptorBufferInfo;

	// perform the update of the descriptor set.
	vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);


	VkPipelineShaderStageCreateInfo shaderStageCreateInfo = {};
	shaderStageCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	shaderStageCreateInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	shaderStageCreateInfo.module = context->m_computeShaderModule1;
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
	m_context->CheckResultVulkan(vkBeginCommandBuffer(m_commandBuffer, &beginInfo)); // start recording commands.

	vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
	vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);
	////vkCmdDispatch(commandBuffer, (uint32_t)ceil(WIDTH / float(WORKGROUP_SIZE)), (uint32_t)ceil(HEIGHT / float(WORKGROUP_SIZE)), 1);
	vkCmdDispatch(m_commandBuffer, 1, 1, 1);
	m_context->CheckResultVulkan(vkEndCommandBuffer(m_commandBuffer));
}