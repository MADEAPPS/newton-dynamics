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

void ndBrainGpuContext::CopyBufferIndirect(const ndBrainUniformBuffer& parameterBuffer, const ndBrainIntegerBuffer& indexData, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer = parameterBuffer.m_gpuBuffer;
	const ndCopyBufferCommandInfo& data = *((ndCopyBufferCommandInfo*)&gpuBuffer->m_memory[0]);
	ndInt32 stride = ndInt32(data.m_strideInByte);
	ndInt32 srcStride = ndInt32(data.m_srcStrideInByte);
	ndInt32 srcOffset = ndInt32(data.m_srcOffsetInByte);
	ndInt32 dstStride = ndInt32(data.m_dstStrideInByte);
	ndInt32 dstOffset = ndInt32(data.m_dstOffsetInByte);

	ndAssert(stride <= srcStride);
	ndAssert(stride <= dstStride);

	ndSharedPtr<ndBrainGpuBuffer>& dstBuffer = dstData.m_gpuBuffer;
	const ndSharedPtr<ndBrainGpuBuffer>& srcBuffer = srcData.m_gpuBuffer;
	const ndSharedPtr<ndBrainGpuBuffer>& indexBuffer = indexData.m_gpuBuffer;

	ndInt8* const dst = &dstBuffer->m_memory[0];
	const ndInt8* const src = &srcBuffer->m_memory[0];
	const ndUnsigned32* const indexPtr = (ndUnsigned32*)&indexBuffer->m_memory[0];
	ndAssert(dst);
	ndAssert(src);
	ndAssert(indexPtr);

	ndInt32 count = ndInt32(indexData.SizeInBytes() / sizeof(ndUnsigned32));
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndUnsigned32 index = indexPtr[i];
		ndMemCpy(&dst[i * dstStride + dstOffset], &src[index * srcStride + srcOffset], stride);
	}
}

void ndBrainGpuContext::CopyBuffer(const ndBrainUniformBuffer& parameterBuffer, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	//const ndFixSizeArray<ndUnsigned32, 256>& bufferData = **parameterBuffer.m_data;
	//const ndCopyBufferCommandInfo& data = *((ndCopyBufferCommandInfo*)&bufferData[0]);
	const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer = parameterBuffer.m_gpuBuffer;
	const ndCopyBufferCommandInfo& data = *((ndCopyBufferCommandInfo*)&gpuBuffer->m_memory[0]);

	ndInt32 stride = ndInt32(data.m_strideInByte);
	ndInt32 srcStride = ndInt32(data.m_srcStrideInByte);
	ndInt32 srcOffset = ndInt32(data.m_srcOffsetInByte);
	ndInt32 dstStride = ndInt32(data.m_dstStrideInByte);
	ndInt32 dstOffset = ndInt32(data.m_dstOffsetInByte);

	ndAssert(stride <= srcStride);
	ndAssert(stride <= dstStride);

	ndUnsigned8* const dst = (ndUnsigned8*)dstData.GetCpuPtr();
	const ndUnsigned8* const src = (ndUnsigned8*)srcData.GetCpuPtr();
	ndAssert(dst);
	ndAssert(src);

	for (ndInt32 i = 0; i < numberOfWorkGrups; ++i)
	{
		ndMemCpy(&dst[i * dstStride + dstOffset], &src[i * srcStride + srcOffset], stride);
	}
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

