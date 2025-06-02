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

class ndBrainGpuContext;
class ndBrainGpuBuffer;
class ndBrainGpuFloatBuffer;
class ndBrainGpuUniformBuffer;

#if !defined (D_USE_VULKAN_SDK)

class ndBrainGpuCommand : public ndClassAlloc
{
	public:
	ndBrainGpuCommand(ndBrainGpuContext* const context, size_t id)
		:m_context(context)
		,m_id(id)
	{
	}
	virtual ~ndBrainGpuCommand(){}

	size_t GetId() const { return m_id; }
	void Assembly(void*, ndInt32, ndInt32, ndBrainGpuBuffer**) {}

	protected:
	ndBrainGpuContext* m_context;
	size_t m_id;
};

#else

class ndBrainGpuCommand : public ndContainersFreeListAlloc<ndBrainGpuCommand>
{
	public:
	virtual ~ndBrainGpuCommand();

	size_t GetId() const { return m_id; }
	protected:
	ndBrainGpuCommand(ndBrainGpuContext* const context, size_t id);
	void Assembly(void* const shader, ndInt32 workGroups, ndInt32 paramCount, ndBrainGpuBuffer** params);

	ndBrainGpuContext* m_context;
	VkPipeline m_pipeline;
	VkDescriptorSet m_descriptorSet;
	VkCommandBuffer m_commandBuffer;
	VkPipelineLayout m_pipelineLayout;
	VkDescriptorSetLayout m_descriptorSetLayout;
	size_t m_id;

	friend class ndBrainGpuContext;
};

#endif
#endif