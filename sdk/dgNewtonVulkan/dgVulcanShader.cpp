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
#include "dgSolver.h"
#include "dgWorldBase.h"
#include "dgVulcanShader.h"

dgVulkanShaderInfo::dgVulkanShaderInfo()
{
	memset(this, 0, sizeof(dgVulkanShaderInfo));
}

void dgVulkanShaderInfo::CreateInitBody (dgVulkanContext& context)
{
	VkResult err = VK_SUCCESS;

	context.m_initBody.m_module = CreateShaderModule(context, "InitBodyArray");
	VkDescriptorSetLayoutBinding descriptorSetLayoutBindings[2] =
	{
		{
			0,
			VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			1,
			VK_SHADER_STAGE_COMPUTE_BIT,
			0
		},
		{
			1,
			VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			1,
			VK_SHADER_STAGE_COMPUTE_BIT,
			0
		}
	};

	VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo;
	Clear(&descriptorSetLayoutCreateInfo);

	descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	descriptorSetLayoutCreateInfo.bindingCount = 2;
	descriptorSetLayoutCreateInfo.pBindings = descriptorSetLayoutBindings;
	err = vkCreateDescriptorSetLayout(context.m_device, &descriptorSetLayoutCreateInfo, &context.m_allocator, &context.m_initBody.m_layout);
	dgAssert(err == VK_SUCCESS);

	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo;
	Clear(&pipelineLayoutCreateInfo);
	pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutCreateInfo.setLayoutCount = 1;
	pipelineLayoutCreateInfo.pSetLayouts = &context.m_initBody.m_layout;
	err = vkCreatePipelineLayout(context.m_device, &pipelineLayoutCreateInfo, &context.m_allocator, &context.m_initBody.m_pipelineLayout);
	dgAssert(err == VK_SUCCESS);

	VkPipelineShaderStageCreateInfo loadPipelineShaderStage;
	Clear(&loadPipelineShaderStage);
	loadPipelineShaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	loadPipelineShaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	loadPipelineShaderStage.module = context.m_initBody.m_module;
	loadPipelineShaderStage.pName = "main";
	loadPipelineShaderStage.pSpecializationInfo = NULL;

	VkComputePipelineCreateInfo computePipeLineInfo;
	Clear(&computePipeLineInfo);
	computePipeLineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	computePipeLineInfo.stage = loadPipelineShaderStage;
	computePipeLineInfo.flags = 0;
	computePipeLineInfo.layout = context.m_initBody.m_pipelineLayout;

	err = vkCreateComputePipelines(context.m_device, context.m_pipeLineCache, 1, &computePipeLineInfo, &context.m_allocator, &context.m_initBody.m_pipeLine);
	dgAssert(err == VK_SUCCESS);
}

void dgVulkanShaderInfo::Destroy (dgVulkanContext& context)
{
	vkDestroyPipelineLayout(context.m_device, context.m_initBody.m_pipelineLayout, &context.m_allocator);
	vkDestroyDescriptorSetLayout(context.m_device, context.m_initBody.m_layout, &context.m_allocator);
	vkDestroyPipeline(context.m_device, context.m_initBody.m_pipeLine, &context.m_allocator);
	vkDestroyShaderModule(context.m_device, context.m_initBody.m_module, &context.m_allocator);
}

VkShaderModule dgVulkanShaderInfo::CreateShaderModule(dgVulkanContext& context, const char* const shaderName) const
{
	char fullPath[1024];
	uint32_t shaderByteCode[1024 * 32];

	sprintf(fullPath, "%s%s.spv", dgWorldBase::m_libPath, shaderName);
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

	VkShaderModuleCreateInfo shaderModuleCreateInfo;
	Clear(&shaderModuleCreateInfo);
	shaderModuleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	shaderModuleCreateInfo.codeSize = count * sizeof(uint32_t);
	shaderModuleCreateInfo.pCode = (uint32_t*)&shaderByteCode[0];

	VkShaderModule module;
	VkResult err = VK_SUCCESS;

	err = vkCreateShaderModule(context.m_device, &shaderModuleCreateInfo, &context.m_allocator, &module);
	dgAssert(err == VK_SUCCESS);

	return module;
}


