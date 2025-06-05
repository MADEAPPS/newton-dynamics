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

ndBrainGpuCommand::ndBrainGpuCommand(ndBrainGpuContext* const context)
	:ndClassAlloc()
	,m_context(context)
	,m_workGroupSize(0)
	,m_numberOfWorkGroups(0)
{
}

ndBrainGpuCommand::~ndBrainGpuCommand()
{
}

void ndBrainGpuCommand::Assembly(const ndSharedPtr<ndBrainGpuShader>& shader, ndInt32 workGroupSize, ndInt32 buffersCount, ndBrainGpuBuffer** buffer)
{
	m_shader = shader;
	m_workGroupSize = size_t(workGroupSize);
	m_numberOfWorkGroups = size_t(workGroupSize);
	for (ndInt32 i = 0; i < buffersCount; ++i)
	{
		ndBrainGpuBuffer* const argBuffer = buffer[i];
		cl_int error = m_shader->setArg(cl_uint(i), argBuffer->m_buffer);
		ndAssert(error == CL_SUCCESS);
	}
}

