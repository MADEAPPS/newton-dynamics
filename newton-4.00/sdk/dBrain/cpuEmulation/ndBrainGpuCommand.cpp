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
#include "ndBrainKernel.h"
#include "ndBrainContext.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainUniformBuffer.h"

#if 0
ndBrainGpuCommand::ndBrainGpuCommand(ndBrainContext* const context, const ndCommandSharedInfo& info)
	:ndContainersFreeListAlloc<ndBrainGpuCommand>()
	,m_context(context)
	,m_shader()
	,m_info(info)
	,m_layer(nullptr)
	,m_workGroupSize(0)
	,m_numberOfWorkGroups(0)
{
}

ndBrainGpuCommand::~ndBrainGpuCommand()
{
}

void ndBrainGpuCommand::Assembly(const ndSharedPtr<ndBrainKernel>& shader, ndInt32 workGroupSize, ndInt32 buffersCount, ndBrainBuffer** buffer)
{
	m_shader = shader;
	m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
	m_numberOfWorkGroups = size_t(workGroupSize);

	m_parameters.SetCount(0);
	for (ndInt32 i = 0; i < buffersCount; ++i)
	{
		m_parameters.PushBack(buffer[i]);
	}
}

#endif

ndBrainGpuCommand::ndBrainGpuCommand(const ndBrainBufferCommandDesc& desc)
	:ndBrainBufferCommand(desc)
{

}

ndBrainGpuCommand::~ndBrainGpuCommand()
{
}

void ndBrainGpuCommand::Execute(ndInt32)
{
	ndAssert(0);
}
