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
#include "ndBrainGpuUniformBuffer.h"

ndBrainGpuCommand::ndBrainGpuCommand(ndBrainGpuContext* const context, const ndBrainLayer::ndCommandShareInfo& info)
	:ndClassAlloc()
	,m_context(context)
	,m_shader()
	,m_info(info)
	,m_layer(nullptr)
	,m_workGroupSize(0)
	,m_numberOfWorkGrouds(0)
{
}

ndBrainGpuCommand::~ndBrainGpuCommand()
{
}

void ndBrainGpuCommand::Assembly(const ndSharedPtr<ndBrainGpuShader>& shader, ndInt32 workGroupSize, ndInt32 buffersCount, ndBrainGpuBuffer** buffer)
{
	m_shader = shader;
	m_numberOfWorkGrouds = size_t(workGroupSize);

	m_parameters.SetCount(0);
	for (ndInt32 i = 0; i < buffersCount; ++i)
	{
		m_parameters.PushBack(buffer[i]);
	}
}