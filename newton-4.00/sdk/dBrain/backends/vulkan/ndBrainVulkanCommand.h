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
#ifndef __ND_BRAIN_GPU_VULKAN_COMMAND_H__
#define __ND_BRAIN_GPU_VULKAN_COMMAND_H__

class ndBrainGpuContext;
class ndBrainGpuFloatBuffer;

#if 1
class ndBrainVulkanCommand : public ndClassAlloc
{
	public:
	virtual ~ndBrainVulkanCommand();

	protected:
	ndBrainVulkanCommand(ndBrainGpuContext* const context);
	void Assembly(VkShaderModule shader, ndInt32 workGroups, ndInt32 paramCount, ndBrainGpuBuffer** params);

	ndBrainGpuContext* m_context;
	VkPipeline m_pipeline;
	VkDescriptorSet m_descriptorSet;
	VkCommandBuffer m_commandBuffer;
	VkPipelineLayout m_pipelineLayout;
	VkDescriptorSetLayout m_descriptorSetLayout;

	friend class ndBrainGpuContext;
};
#endif

//class ndBrainVulkanCommandTest : public ndBrainVulkanCommand
//{
//	public:
//	ndBrainVulkanCommandTest(ndBrainGpuContext* const context,
//		ndBrainGpuUniformBuffer& parammeters,
//		ndBrainGpuFloatBuffer& input, ndBrainGpuFloatBuffer& output);
//};
#endif