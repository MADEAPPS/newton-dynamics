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
class ndBrainGpuFloatBuffer;

#if !defined (D_USE_VULKAN_SDK)
class ndBrainGpuCommand : public ndClassAlloc
{
	public:
	ndBrainGpuCommand(ndBrainGpuContext* const context):m_context(context){}
	virtual ~ndBrainGpuCommand(){}
	protected:
	ndBrainGpuContext* m_context;
};

class ndBrainGpuCommandTest0 : public ndBrainGpuCommand
{
	public:
		ndBrainGpuCommandTest0(ndBrainGpuContext* const context, ndBrainGpuUniformBuffer&, ndBrainGpuFloatBuffer&, ndBrainGpuFloatBuffer&)
		:ndBrainGpuCommand(context)
	{
	}
};

class ndBrainGpuCommandTest1 : public ndBrainGpuCommand
{
	public:
	ndBrainGpuCommandTest1(ndBrainGpuContext* const context, ndBrainGpuFloatBuffer&)
		:ndBrainGpuCommand(context)
	{
	}
};

#else

class ndBrainGpuCommand : public ndClassAlloc
{
	public:
	virtual ~ndBrainGpuCommand();

	protected:
	ndBrainGpuCommand(ndBrainGpuContext* const context);

	ndBrainGpuContext* m_context;
	VkPipeline m_pipeline;
	VkDescriptorSet m_descriptorSet;
	VkCommandBuffer m_commandBuffer;
	VkPipelineLayout m_pipelineLayout;
	VkDescriptorSetLayout m_descriptorSetLayout;

	friend class ndBrainGpuContext;
};

class ndBrainGpuCommandTest0 : public ndBrainGpuCommand
{
	public:
	ndBrainGpuCommandTest0(ndBrainGpuContext* const context, 
		ndBrainGpuUniformBuffer& parammeters,
		ndBrainGpuFloatBuffer& input, ndBrainGpuFloatBuffer& output);
};

class ndBrainGpuCommandTest1 : public ndBrainGpuCommand
{
	public:
	ndBrainGpuCommandTest1(ndBrainGpuContext* const context, ndBrainGpuFloatBuffer& buffer);
};

#endif


#endif