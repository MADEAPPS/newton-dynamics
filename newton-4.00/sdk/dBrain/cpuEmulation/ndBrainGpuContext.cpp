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
#include "ndBrainVector.h"
#include "ndBrainKernel.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainBufferCommand.h"

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
	,ndBrainThreadPool()
{
	ndInt32 numOfThreads = (ndBrainThreadPool::GetMaxThreads() + 1) / 2;
#ifdef _DEBUG
numOfThreads = 1;
#endif
	SetThreadCount(numOfThreads);
	CreateKerners();
	CreateCopyCommands();
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

ndBrainGpuContext* ndBrainGpuContext::GetAsGpuContext()
{
	return this;
}

void ndBrainGpuContext::BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector)
{
	size_t sizeInBytes = ndMin(size_t(dst.SizeInBytes()), size_t(srcVector.GetCount() * sizeof(ndReal)));
	MemoryToDevice(dst, 0, sizeInBytes, &srcVector[0]);
}

void ndBrainGpuContext::BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector)
{
	ndAssert(0);
	size_t sizeInBytes = ndMin(size_t(src.SizeInBytes()), size_t(dstVector.GetCount() * sizeof(ndReal)));
	MemoryFromDevice(src, 0, sizeInBytes, &dstVector[0]);
}

void ndBrainGpuContext::CopyBuffer(const ndCopyBufferCommandInfo& parameters, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	m_copyBufferParams->MemoryToDevice(0, sizeof(ndCopyBufferCommandInfo), &parameters);

	ndBrainBufferCommandDesc& descriptor = m_copyBufferCommand->GetDescriptor();
	descriptor.SetCount(0);
	descriptor.PushBack(*m_copyBufferParams);
	descriptor.PushBack((ndBrainBuffer*)&dstData);
	descriptor.PushBack((ndBrainBuffer*)&srcData);

	descriptor.m_miniBatchSize = numberOfWorkGrups;
	descriptor.m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
	SubmitBufferCommand(*m_copyBufferCommand);
}

void ndBrainGpuContext::CopyBufferIndirect(const ndCopyBufferCommandInfo& parameters, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	m_copyBufferParams->MemoryToDevice(0, sizeof(ndCopyBufferCommandInfo), &parameters);

	ndBrainBufferCommandDesc& descriptor = m_copyBufferIndirectCommand->GetDescriptor();
	descriptor.SetCount(0);
	descriptor.PushBack(*m_copyBufferParams);
	descriptor.PushBack((ndBrainBuffer*)&dstData);
	descriptor.PushBack((ndBrainBuffer*)&srcData);
	descriptor.PushBack((ndBrainBuffer*)&indexBuffer);

	descriptor.m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
	descriptor.m_miniBatchSize = ndInt32(indexBuffer.SizeInBytes() / sizeof(ndUnsigned32));
	SubmitBufferCommand(*m_copyBufferIndirectCommand);
}

void ndBrainGpuContext::CreateCopyCommands()
{
	ndBrainBufferCommandDesc copyDescriptor(0);
	copyDescriptor.m_context = this;
	copyDescriptor.m_kernel = m_brainCopyBuffer;
	m_copyBufferCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyDescriptor));

	ndBrainBufferCommandDesc copyIndirectDescriptor(0);
	copyIndirectDescriptor.m_context = this;
	copyIndirectDescriptor.m_kernel = m_brainCopyBufferIndirect;
	m_copyBufferIndirectCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyIndirectDescriptor));

	ndCopyBufferCommandInfo copyBuffer;
	m_copyBufferParams = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(this, sizeof(ndCopyBufferCommandInfo), &copyBuffer, true));
}

void ndBrainGpuContext::MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer = deviceBuffer.m_gpuBuffer;
	ndInt8* const src = &gpuBuffer->m_memory[0];
	ndAssert(src);
	ndMemCpy((ndInt8*)outputMemory, &src[offsetInBytes], ndInt64(sizeInBytes));
}

void ndBrainGpuContext::MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory) const
{
	ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer = deviceBuffer.m_gpuBuffer;
	ndInt8* const dst = &gpuBuffer->m_memory[0];
	ndAssert(dst);
	ndMemCpy(&dst[offsetInBytes], (ndInt8*)inputMemory, ndInt64(sizeInBytes));
}

void ndBrainGpuContext::SyncBufferCommandQueue()
{
	// do nothing. cpu kernels always wait for completion.
}

void ndBrainGpuContext::SubmitBufferCommand(ndBrainBufferCommand* const command)
{
	ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	ndBrainKernel& shader = **desc.m_kernel;

	shader.m_parameters.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32 (desc.GetCount()); ++i)
	{
		shader.m_parameters.PushBack(desc[i]);
	}

	ndAtomic<ndInt32> iterator(0);
	auto ExecuteCommand = ndMakeObject::ndFunction([this, &iterator, &command](ndInt32, ndInt32)
	{
		ndBrainBufferCommandDesc& desc = command->GetDescriptor();
		ndInt32 workGroupdSize = ndInt32(desc.m_workGroupSize);
		ndInt32 numberOfWorkGrouds = ndInt32(desc.m_miniBatchSize);

		ndBrainKernel& shader = **desc.m_kernel;
		for (ndInt32 i = iterator++; i < numberOfWorkGrouds; i = iterator++)
		{
			shader.Execute(i, workGroupdSize);
		}
	});
	ParallelExecute(ExecuteCommand);
}

