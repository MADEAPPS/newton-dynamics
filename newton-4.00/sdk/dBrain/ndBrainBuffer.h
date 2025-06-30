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
#ifndef __ND_BRAIN_BUFFER_H__
#define __ND_BRAIN_BUFFER_H__

#include "ndBrainStdafx.h"

class ndBrainVector;
class ndBrainMatrix;
class ndBrainContext;
class ndBrainGpuBuffer;

class ndCopyBufferCommandInfo
{
	public:
	ndCopyBufferCommandInfo()
		:m_strideInByte(0)
		,m_srcStrideInByte(0)
		,m_srcOffsetInByte(0)
		,m_dstStrideInByte(0)
		,m_dstOffsetInByte(0)
	{
	}

	ndInt32 m_strideInByte;
	ndInt32 m_srcStrideInByte;
	ndInt32 m_srcOffsetInByte;
	ndInt32 m_dstStrideInByte;
	ndInt32 m_dstOffsetInByte;
};

class ndBrainBuffer : public ndContainersFreeListAlloc<ndBrainBuffer>
{
	protected:
	ndBrainBuffer(ndBrainContext* const context, ndInt64 sizeInByte, bool memoryMapped = false);

	public:
	virtual ~ndBrainBuffer();
	size_t SizeInBytes() const;

	virtual void* GetCpuPtr() = 0;
	virtual void* GetCpuPtr() const = 0;
	ndBrainGpuBuffer* GetGpuBuffer();

	protected:
	ndBrainContext* m_context;
	ndSharedPtr<ndBrainGpuBuffer> m_gpuBuffer;
	size_t m_sizeInBytes;
	bool m_isMemoryMapped;
	friend class ndBrainGpuContext;
};

#endif