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
#ifndef __ND_BRAIN_GPU_BUFFER_H__
#define __ND_BRAIN_GPU_BUFFER_H__

#include "ndBrainStdafx.h"
#include "ndBrainBuffer.h"

class ndBrainVector;
class ndBrainMatrix;
class ndBrainGpuBuffer;
class ndBrainGpuContext;

class ndBrainGpuBuffer : public ndBrainBuffer
{
	protected:
	ndBrainGpuBuffer(ndBrainContext* const context, ndInt64 sizeInByte, ndStorageBufferType bufferTypeFlags);

	public:
	virtual ~ndBrainGpuBuffer();

	protected:
	cl::Buffer m_buffer;
	size_t m_sizeInBytes;
	friend class ndBrainGpuContext;
	friend class ndBrainGpuCommand;
};

#endif