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
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"
#include "ndBrainUniformBuffer.h"

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
	,ndBrainThreadPool()
{
	ndInt32 numOfThreads = (ndBrainThreadPool::GetMaxThreads() + 1) / 2;
//numOfThreads = 1;
	SetThreadCount(numOfThreads);
	CreateKerners();
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

ndBrainGpuContext* ndBrainGpuContext::GetAsGpuContext()
{
	return this;
}

//void ndBrainGpuContext::AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>& command)
//{
//	ndAssert(0);
//	//ndAtomic<ndInt32> iterator(0);
//	//ndBrainGpuShader& shader = **command->m_shader;
//	//shader.m_parameters.SetCount(0);
//	//for (ndInt32 i = 0; i < ndInt32 (command->m_parameters.GetCount()); ++i)
//	//{
//	//	shader.m_parameters.PushBack(command->m_parameters[i]);
//	//}
//	//
//	//auto ExecuteCommand = ndMakeObject::ndFunction([this, &iterator, &command](ndInt32, ndInt32)
//	//{
//	//	ndInt32 workGroupdSize = ndInt32(command->m_workGroupSize);
//	//	ndInt32 numberOfWorkGrouds = ndInt32(command->m_numberOfWorkGroups);
//	//	
//	//	ndBrainGpuShader& kernel = **command->m_shader;
//	//	for (ndInt32 i = iterator++; i < numberOfWorkGrouds; i = iterator++)
//	//	{
//	//		kernel.Execute(i, workGroupdSize);
//	//	}
//	//});
//	//iterator = 0;
//	//ParallelExecute(ExecuteCommand);
//}

void ndBrainGpuContext::BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector)
{
	ndAssert(0);
}

void ndBrainGpuContext::CopyBufferIndirect(const ndBrainUniformBuffer& parameterBuffer, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndAssert(0);
}

void ndBrainGpuContext::MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	ndAssert(0);
}

void ndBrainGpuContext::MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const srcMemory) const
{
	ndAssert(0);
}

void ndBrainGpuContext::SubmitBufferCommand(ndBrainBufferCommand* const command)
{
	ndAssert(0);

	//ndAtomic<ndInt32> iterator(0);
//ndBrainGpuShader& shader = **command->m_shader;
//shader.m_parameters.SetCount(0);
//for (ndInt32 i = 0; i < ndInt32 (command->m_parameters.GetCount()); ++i)
//{
//	shader.m_parameters.PushBack(command->m_parameters[i]);
//}
//
//auto ExecuteCommand = ndMakeObject::ndFunction([this, &iterator, &command](ndInt32, ndInt32)
//{
//	ndInt32 workGroupdSize = ndInt32(command->m_workGroupSize);
//	ndInt32 numberOfWorkGrouds = ndInt32(command->m_numberOfWorkGroups);
//	
//	ndBrainGpuShader& kernel = **command->m_shader;
//	for (ndInt32 i = iterator++; i < numberOfWorkGrouds; i = iterator++)
//	{
//		kernel.Execute(i, workGroupdSize);
//	}
//});
//iterator = 0;
//ParallelExecute(ExecuteCommand);
}

void ndBrainGpuContext::SyncBufferCommandQueue()
{
	// do nothing. cpu kernels always wait for completion.
	ndAssert(0);
}
