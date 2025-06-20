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
#ifndef __ND_BRAIN_GPU_COMMAND_H__
#define __ND_BRAIN_GPU_COMMAND_H__

#include "ndBrainStdafx.h"
#include "ndBrainGpuContext.h"

class ndBrainGpuBuffer;
class ndBrainGpuFloatBuffer;
class ndBrainGpuUniformBuffer;

class ndBrainGpuCommand : public ndContainersFreeListAlloc<ndBrainGpuCommand>
{
	public:
	virtual ~ndBrainGpuCommand();

	protected:
	ndBrainGpuCommand(ndBrainGpuContext* const context);
	void Assembly(const ndSharedPtr<ndBrainGpuShader>& shader, ndInt32 workGroups, ndInt32 paramCount, ndBrainGpuBuffer** params);

	ndBrainGpuContext* m_context;
	VkPipeline m_pipeline;
	VkDescriptorSet m_descriptorSet;
	VkCommandBuffer m_commandBuffer;
	VkPipelineLayout m_pipelineLayout;
	VkDescriptorSetLayout m_descriptorSetLayout;

	friend class ndBrainGpuContext;
};
#endif