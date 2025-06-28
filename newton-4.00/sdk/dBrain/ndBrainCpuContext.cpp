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

ndBrainCpuContext::ndBrainCpuContext()
	:ndBrainContext()
	,ndBrainThreadPool()
{
	ndInt32 numOfThreads = (ndBrainThreadPool::GetMaxThreads() + 1) / 2;
	SetThreadCount(numOfThreads);
}

ndBrainCpuContext::~ndBrainCpuContext()
{
}

ndBrainContext::ndContextType ndBrainCpuContext::GetType() const
{
	return ndBrainContext::m_cpu;
}

ndBrainCpuContext* ndBrainCpuContext::GetAsCpuContext()
{
	return this;
}

void ndBrainCpuContext::BrainVectorToDevice(ndBrainFloatBuffer& buffer, const ndBrainVector& srcVector)
{
	ndInt64 size = ndMin(srcVector.GetCount(), buffer.m_buffer->GetCount());
	ndBrainMemVector src(&srcVector[0], size);
	ndBrainMemVector dst(&(**buffer.m_buffer)[0], size);
	dst.Set(src);
}

void ndBrainCpuContext::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData)
{
	ndAssert(0);
	//const ndBrainCpuIntegerBuffer& indirectArray = *((ndBrainCpuIntegerBuffer*)&indexBuffer);
	//const ndBrainCpuUniformBuffer& uniforms = *(ndBrainCpuUniformBuffer*)&parameterBuffer;
	//const ndCopyBufferCommandInfo& data = *((ndCopyBufferCommandInfo*)&uniforms.m_data[0]);
	//
	//ndInt32 stride = ndInt32(data.m_strideInByte / sizeof(ndReal));
	//ndInt32 srcStride = ndInt32(data.m_srcStrideInByte / sizeof(ndReal));
	//ndInt32 srcOffset = ndInt32(data.m_srcOffsetInByte / sizeof(ndReal));
	//ndInt32 dstStride = ndInt32(data.m_dstStrideInByte / sizeof(ndReal));
	//ndInt32 dstOffset = ndInt32(data.m_dstOffsetInByte / sizeof(ndReal));
	//ndInt32 count = ndInt32(indirectArray.SizeInBytes() / sizeof(ndUnsigned32));
	//
	//ndAssert(stride <= srcStride);
	//ndAssert(stride <= dstStride);
	//
	//ndBrainVector& dstVector = **dstData.m_buffer;
	//const ndBrainVector& srcVector = **srcData.m_buffer;
	//
	//for (ndInt32 i = 0; i < count; ++i)
	//{
	//	ndUnsigned32 index = indirectArray.m_indexArray[i];
	//	const ndBrainMemVector src(&srcVector[index * srcStride + srcOffset], stride);
	//	ndBrainMemVector dst(&dstVector[i * dstStride + dstOffset], stride);
	//	dst.Set(src);
	//}
}

void ndBrainCpuContext::MemoryFromDevice(const ndBrainFloatBuffer& buffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	size_t offset = offsetInBytes / sizeof(ndReal);
	//ndBrainMemVector dst((ndBrainFloat*)outputData, ndInt32(sizeInBytes / sizeof(ndReal)));
	//const ndBrainMemVector src(&m_buffer[ndInt64(offset)], ndInt32(sizeInBytes / sizeof(ndReal)));
	//dst.Set(src);

}