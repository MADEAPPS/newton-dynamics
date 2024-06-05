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
#include "ndBrain.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuInference.h"


#if defined (D_USE_VULKAN_SDK)

class ndBrainGpuInference::ndBrainLoadInputData : public ndBrainGpuCommand
{
	public:
	ndBrainLoadInputData(ndBrainGpuContext* const context)
		:ndBrainGpuCommand(context)
	{
		VkDevice const device = m_context->GetDevice();
		VkCommandPool const commandPool = m_context->GetCommandPool();
		VkAllocationCallbacks* const allocator = m_context->GetAllocator();
		VkDescriptorPool const descriptorPool = m_context->GetDescriptorPool();
		
		VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
		commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandBufferAllocateInfo.commandPool = commandPool;
		commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		commandBufferAllocateInfo.commandBufferCount = 1;
		m_context->CheckResultVulkan(vkAllocateCommandBuffers(device, &commandBufferAllocateInfo, &m_commandBuffer));
		
		VkDescriptorSetLayoutBinding descriptorSetLayoutBinding[3] = {};
		// binding = 0
		descriptorSetLayoutBinding[0].binding = 0;
		descriptorSetLayoutBinding[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		descriptorSetLayoutBinding[0].descriptorCount = 1;
		descriptorSetLayoutBinding[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		
		// binding = 1
		descriptorSetLayoutBinding[1].binding = 1;
		descriptorSetLayoutBinding[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		descriptorSetLayoutBinding[1].descriptorCount = 1;
		descriptorSetLayoutBinding[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		
		// binding = 2
		descriptorSetLayoutBinding[2].binding = 2;
		descriptorSetLayoutBinding[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		descriptorSetLayoutBinding[2].descriptorCount = 1;
		descriptorSetLayoutBinding[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		
		VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
		descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		descriptorSetLayoutCreateInfo.bindingCount = 3;
		descriptorSetLayoutCreateInfo.pBindings = descriptorSetLayoutBinding;
		m_context->CheckResultVulkan(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, allocator, &m_descriptorSetLayout));
		
		VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
		descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		descriptorSetAllocateInfo.descriptorPool = descriptorPool;
		descriptorSetAllocateInfo.descriptorSetCount = 1;
		descriptorSetAllocateInfo.pSetLayouts = &m_descriptorSetLayout;
		m_context->CheckResultVulkan(vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, &m_descriptorSet));
		
		//// Specify the buffer to bind to the descriptor.
		//VkDescriptorBufferInfo descriptorBufferInfo[3] = {};
		//descriptorBufferInfo[0].buffer = parammeters.GetBuffer();
		//descriptorBufferInfo[0].offset = 0;
		//descriptorBufferInfo[0].range = (VkDeviceSize)parammeters.SizeInBytes();
		//
		//descriptorBufferInfo[1].buffer = input.GetBuffer();
		//descriptorBufferInfo[1].offset = 0;
		//descriptorBufferInfo[1].range = (VkDeviceSize)input.SizeInBytes();
		//
		//descriptorBufferInfo[2].buffer = output.GetBuffer();
		//descriptorBufferInfo[2].offset = 0;
		//descriptorBufferInfo[2].range = (VkDeviceSize)output.SizeInBytes();
		//
		//VkWriteDescriptorSet writeDescriptorSet[3] = {};
		//writeDescriptorSet[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		//writeDescriptorSet[0].dstSet = m_descriptorSet;
		//writeDescriptorSet[0].dstBinding = 0;
		//writeDescriptorSet[0].descriptorCount = 1;
		//writeDescriptorSet[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		//writeDescriptorSet[0].pBufferInfo = &descriptorBufferInfo[0];
		//
		//writeDescriptorSet[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		//writeDescriptorSet[1].dstSet = m_descriptorSet;
		//writeDescriptorSet[1].dstBinding = 1;
		//writeDescriptorSet[1].descriptorCount = 1;
		//writeDescriptorSet[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		//writeDescriptorSet[1].pBufferInfo = &descriptorBufferInfo[1];
		//
		//writeDescriptorSet[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		//writeDescriptorSet[2].dstSet = m_descriptorSet;
		//writeDescriptorSet[2].dstBinding = 2;
		//writeDescriptorSet[2].descriptorCount = 1;
		//writeDescriptorSet[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		//writeDescriptorSet[2].pBufferInfo = &descriptorBufferInfo[2];
		//
		//// perform the update of the descriptor set.
		//vkUpdateDescriptorSets(device, 3, writeDescriptorSet, 0, nullptr);
		//
		//VkPipelineShaderStageCreateInfo shaderStageCreateInfo = {};
		//shaderStageCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		//shaderStageCreateInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
		//shaderStageCreateInfo.module = context->m_computeShaderModule0;
		//shaderStageCreateInfo.pName = "main";
		//
		//VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
		//pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		//pipelineLayoutCreateInfo.setLayoutCount = 1;
		//pipelineLayoutCreateInfo.pSetLayouts = &m_descriptorSetLayout;
		//m_context->CheckResultVulkan(vkCreatePipelineLayout(device, &pipelineLayoutCreateInfo, allocator, &m_pipelineLayout));
		//
		//VkComputePipelineCreateInfo pipelineCreateInfo = {};
		//pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
		//pipelineCreateInfo.stage = shaderStageCreateInfo;
		//pipelineCreateInfo.layout = m_pipelineLayout;
		//m_context->CheckResultVulkan(vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipelineCreateInfo, allocator, &m_pipeline));
		//
		//VkCommandBufferBeginInfo beginInfo = {};
		//beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		//beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		//m_context->CheckResultVulkan(vkBeginCommandBuffer(m_commandBuffer, &beginInfo)); // start recording commands.
		//
		//vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
		//vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);
		//vkCmdDispatch(m_commandBuffer, 1, 1, 1);
		//m_context->CheckResultVulkan(vkEndCommandBuffer(m_commandBuffer));
	}
};

ndBrainGpuInference::ndBrainGpuInference(ndBrainGpuContext* const context, ndBrain* const brain, const ndBrainMatrix& input, ndInt32 inputBatchSize)
	:ndClassAlloc()
	,m_brain(brain)
	,m_context(context)
	,m_inputBuffer()
	,m_workingBuffer()
	,m_displayList()
	//,m_gpuParameters(nullptr)
	//,m_gpuParametersOffsets(nullptr)
	,m_inputBatchSize(inputBatchSize)
{
	SetInputBuffer(input);
	SetWorkingBuffer();
	//SetParameterVector();

	BuildDisplayList();
}

ndBrainGpuInference::~ndBrainGpuInference()
{
	for (ndInt32 i = m_displayList.GetCount() - 1; i >= 0; --i)
	{
		delete m_displayList[i];
	}

	if (m_workingBuffer.m_buffer)
	{
		delete m_workingBuffer.m_buffer;
	}
	//delete m_input;
	//delete m_gpuParameters;
	//delete m_gpuWorkingBuffer;
	//delete m_gpuParametersOffsets;
}

void ndBrainGpuInference::SetParameterVector()
{
	ndAssert(0);
	//m_offsets.SetCount(0);
	//m_parameters.SetCount(0);
	//
	//const ndArray<ndBrainLayer*>& layers = *m_brain;
	//for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	//{
	//	const ndBrainLayer* const layer = layers[i];
	//	layer->GetNumberOfParameters(m_parameters, m_offsets);
	//}
	//
	//ndInt32 sum = 0;
	//for (ndInt32 i = 0; i < m_offsets.GetCount(); ++i)
	//{
	//	ndInt32 count = m_offsets[i];
	//	m_offsets[i] = sum;
	//	sum += count;
	//}
	//m_offsets.PushBack(sum);
	//m_gpuParametersOffsets = new ndBrainGpuIntegerBuffer(m_context, m_offsets);
	//m_gpuParameters = new ndBrainGpuFloatBuffer(m_context, m_parameters);
}

void ndBrainGpuInference::SetWorkingBuffer()
{
	ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat);
	ndAssert(!(rounding & (rounding - 1)));
	const ndArray<ndBrainLayer*>& layers = *m_brain;
	
	ndInt32 size = (layers[0]->GetInputSize() + rounding - 1) & -rounding;
	m_workingBuffer.m_offsets.PushBack(size);
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		size = (layers[i]->GetOutputBufferSize() + rounding - 1) & -rounding;
		m_workingBuffer.m_offsets.PushBack(size);
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_workingBuffer.m_offsets.GetCount(); ++i)
	{
		ndInt32 offset = m_workingBuffer.m_offsets[i];
		m_workingBuffer.m_offsets[i] = sum;
		sum += offset;
	}

	m_workingBuffer.m_offsets.PushBack(sum);
	m_workingBuffer.m_buffer = new ndBrainGpuFloatBuffer(m_context, sum * m_inputBatchSize);
}

void ndBrainGpuInference::SetInputBuffer(const ndBrainMatrix& input)
{
	ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat); 
	ndInt32 width = (input.GetColumns() + rounding - 1) & -rounding;
	ndInt32 size = width * input.GetRows();

	ndBrainVector temp;
	temp.SetCount(size);
	for (ndInt32 i = 0; i < input.GetRows(); ++i)
	{
		const ndBrainVector& src = input[i];
		ndBrainMemVector dst(&temp[i * width], src.GetCount());
		dst.Set(src);
	}
	m_inputBuffer.m_offsets.PushBack(size);
	m_inputBuffer.m_buffer = new ndBrainGpuFloatBuffer(m_context, temp);
}

void ndBrainGpuInference::BuildDisplayList()
{
	ndBrainGpuCommand* const comand = new ndBrainLoadInputData(m_context);

	m_displayList.PushBack(comand);
}
#endif