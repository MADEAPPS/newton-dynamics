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
#include "ndBrainCpuContext.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainBufferCommand.h"

ndBrainCpuContext::ndBrainCpuContext()
	:ndBrainContext()
	,ndBrainThreadPool()
{
	ndInt32 numOfThreads = (ndBrainThreadPool::GetMaxThreads() + 1) / 2;
#ifdef _DEBUG
//numOfThreads = 1;
#endif

	SetThreadCount(numOfThreads);
}

ndBrainCpuContext::~ndBrainCpuContext()
{
}

ndBrainCpuContext* ndBrainCpuContext::GetAsCpuContext()
{
	return this;
}

void ndBrainCpuContext::SyncBufferCommandQueue()
{
	// do nothing
}

void ndBrainCpuContext::CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndAssert(dstData.SizeInBytes() == srcData.SizeInBytes());
	ndAssert((dstData.SizeInBytes() & (sizeof(ndInt32) - 1)) == 0);

	ndUnsigned32* const dst = (ndUnsigned32*)dstData.GetCpuPtr();
	const ndUnsigned32* const src = (ndUnsigned32*)srcData.GetCpuPtr();
	ndMemCpy(dst, src, ndInt64(dstData.SizeInBytes() / sizeof (ndInt32)));
}

void ndBrainCpuContext::CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 numberOfWorkGroups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndInt32 stride = ndInt32(descriptor.m_strideInByte);
	ndInt32 srcStride = ndInt32(descriptor.m_srcStrideInByte);
	ndInt32 srcOffset = ndInt32(descriptor.m_srcOffsetInByte);
	ndInt32 dstStride = ndInt32(descriptor.m_dstStrideInByte);
	ndInt32 dstOffset = ndInt32(descriptor.m_dstOffsetInByte);

	ndAssert(stride <= srcStride);
	ndAssert(stride <= dstStride);

	ndUnsigned8* const dst = (ndUnsigned8*)dstData.GetCpuPtr();
	const ndUnsigned8* const src = (ndUnsigned8*)srcData.GetCpuPtr();
	ndAssert(dst);
	ndAssert(src);

	for (ndInt32 i = 0; i < numberOfWorkGroups; ++i)
	{
		ndMemCpy(&dst[i * dstStride + dstOffset], &src[i * srcStride + srcOffset], stride);
	}
}

void ndBrainCpuContext::CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndInt32 stride = ndInt32(descriptor.m_strideInByte);
	ndInt32 srcStride = ndInt32(descriptor.m_srcStrideInByte);
	ndInt32 srcOffset = ndInt32(descriptor.m_srcOffsetInByte);
	ndInt32 dstStride = ndInt32(descriptor.m_dstStrideInByte);
	ndInt32 dstOffset = ndInt32(descriptor.m_dstOffsetInByte);

	ndAssert(stride <= srcStride);
	ndAssert(stride <= dstStride);

	ndUnsigned8* const dst = (ndUnsigned8*)dstData.GetCpuPtr();
	const ndUnsigned8* const src = (ndUnsigned8*)srcData.GetCpuPtr();
	const ndUnsigned32* const indexPtr = (ndUnsigned32*)indexBuffer.GetCpuPtr();
	ndAssert(dst);
	ndAssert(src);
	ndAssert(indexPtr);

	ndInt32 count = ndInt32(indexBuffer.SizeInBytes() / sizeof(ndUnsigned32));
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndUnsigned32 index = indexPtr[i];
		ndMemCpy(&dst[i * dstStride + dstOffset], &src[index * srcStride + srcOffset], stride);
	}
}

void ndBrainCpuContext::MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	const ndInt8* const src = (ndInt8*)deviceBuffer.GetCpuPtr();
	ndAssert(src);
	ndMemCpy((ndInt8*)outputMemory, &src[offsetInBytes], ndInt64(sizeInBytes));
}

void ndBrainCpuContext::MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory) const
{
	const ndInt8* const src = (ndInt8*)inputMemory;
	ndInt8* const dst = (ndInt8*)deviceBuffer.GetCpuPtr();
	ndAssert(dst);
	ndAssert(src);
	ndMemCpy(&dst[offsetInBytes], src, ndInt64(sizeInBytes));
}

void ndBrainCpuContext::SubmitBufferCommand(ndBrainBufferCommand* const command)
{
	ndAtomic<ndInt32> iterator(0);
	auto ExecuteCommand = ndMakeObject::ndFunction([this, &iterator, command](ndInt32, ndInt32)
	{
		const ndBrainBufferCommandDesc& descriptor = command->GetDescriptor();
		ndBrainBufferCommandCpu* const cpuCommand = (ndBrainBufferCommandCpu*)command;

		for (ndInt32 i = iterator++; i < descriptor.m_miniBatchSize; i = iterator++)
		{
			cpuCommand->Execute(i);
		}
	});
	ParallelExecute(ExecuteCommand);
}

void ndBrainCpuContext::BrainVectorToDevice(ndBrainFloatBuffer& buffer, const ndBrainVector& srcVector)
{
	size_t sizeInBytes = ndMin(size_t(buffer.SizeInBytes()), size_t(srcVector.GetCount() * sizeof(ndReal)));
	MemoryToDevice(buffer, 0, sizeInBytes, &srcVector[0]);
}

void ndBrainCpuContext::BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector)
{
	ndAssert(0);
	size_t sizeInBytes = ndMin(size_t(src.SizeInBytes()), size_t(dstVector.GetCount() * sizeof(ndReal)));
	MemoryFromDevice(src, 0, sizeInBytes, &dstVector[0]);
}

void ndBrainCpuContext::Set(ndBrainFloatBuffer& dstData, ndBrainFloat value)
{
	ndBrainVector& dst = **dstData.m_buffer;
	dst.Set(value);
}

void ndBrainCpuContext::Set(ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData)
{
	ndBrainVector& dst = **dstData.m_buffer;
	const ndBrainVector& src = **srcData.m_buffer;
	dst.Set(src);
}

void ndBrainCpuContext::Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.Scale(scale);
}

void ndBrainCpuContext::Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Min(src);
}

void ndBrainCpuContext::Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Add(src);
}

void ndBrainCpuContext::Sub(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Sub(src);
}

void ndBrainCpuContext::Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Mul(src);
}

void ndBrainCpuContext::ScaleAdd(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat scale)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.ScaleAdd(src, scale);
}

void ndBrainCpuContext::GaussianSample(ndBrainFloatBuffer& meanBuffer, const ndBrainFloatBuffer& sigmaBuffer, const ndBrainFloatBuffer& uniformRandomBuffer)
{
	ndAssert(meanBuffer.SizeInBytes() == sigmaBuffer.SizeInBytes());
	ndAssert(meanBuffer.SizeInBytes() == uniformRandomBuffer.SizeInBytes());

	ndInt32 elements = ndInt32(meanBuffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector mean((ndBrainFloat*)meanBuffer.GetCpuPtr(), elements);
	const ndBrainMemVector sigma((ndBrainFloat*)sigmaBuffer.GetCpuPtr(), elements);
	const ndBrainMemVector uniformRand((ndBrainFloat*)uniformRandomBuffer.GetCpuPtr(), elements);
	mean.CalculateMeanAndDeviation(sigma, uniformRand);
}

void ndBrainCpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat blend)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Blend(src, blend);
}

void ndBrainCpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, const ndBrainFloatBuffer& blendBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	const ndBrainVector& blend = **blendBuffer.m_buffer;
	dst.Blend(src, blend);
}

void ndBrainCpuContext::LessEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.LessEqual(test);
}

void ndBrainCpuContext::LessEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.LessEqual(src);
}

void ndBrainCpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.GreaterEqual(test);
}

void ndBrainCpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.GreaterEqual(src);
}

void ndBrainCpuContext::Select(ndBrainFloatBuffer& buffer, ndBrainFloatBuffer& maskBuffer, ndBrainFloat a, ndBrainFloat b)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& mask = **maskBuffer.m_buffer;
	dst.Blend(mask, a, b);
}

void ndBrainCpuContext::BroadcastScaler(ndBrainFloatBuffer& buffer, ndInt32 bufferStrideInFloats, const ndBrainFloatBuffer& srcScalar)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcScalar.m_buffer;

	for (ndInt32 i = 0; i < src.GetCount(); ++i)
	{
		ndBrainMemVector subVector(&dst[i * bufferStrideInFloats], bufferStrideInFloats);
		subVector.Set(src[i]);
	}
}