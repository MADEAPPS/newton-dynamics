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

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
	,ndBrainThreadPool()
{
	ndInt32 numOfThreads = (ndBrainThreadPool::GetMaxThreads() + 1) / 2;
//numOfThreads = 1;
	SetThreadCount(numOfThreads);
	CreateKerners();
}

bool ndBrainGpuContext::HasGpuSupport()
{ 
	return true; 
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

ndBrainContext::ndContextType ndBrainGpuContext::GetType() const
{
	return ndBrainContext::m_gpu;
}

ndBrainGpuContext* ndBrainGpuContext::GetAsGpuContext()
{
	return this;
}

void ndBrainGpuContext::SyncQueue()
{
	// do nothing. as cpu alway wait afte each kernel submition
	//ndAssert(0);
}

void ndBrainGpuContext::AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>& command)
{
	ndAtomic<ndInt32> iterator(0);
	auto ExecuteCommand = ndMakeObject::ndFunction([this, &iterator, command](ndInt32, ndInt32)
	{
		ndInt32 miniBatchSize = ndInt32(command->m_miniBatchSize);
		ndBrainGpuContext::ndBrainGpuShader& shader = **command->m_shader;
		//ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)shader.m_parameters[0];
		//ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)shader.m_parameters[1];
		//ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)shader.m_parameters[2];
		//
		////shader.m_workGroupSize = 256;
		//UniformBufferObject* const parameters = &buffer0->m_data;
		//ndBrainFloat* const arg1 = &buffer1->m_buffer[0];
		//ndBrainFloat* const arg2 = &buffer2->m_buffer[0];

		ndInt32 workGroupdSize = 256;
		for (ndInt32 i = iterator++; i < miniBatchSize; i = iterator++)
		{
			shader.Execute(i, workGroupdSize);
		}
	});
	iterator = 0;
	ParallelExecute(ExecuteCommand);
}