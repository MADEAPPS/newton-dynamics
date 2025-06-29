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
#ifndef __ND_BRAIN_CONTEXT_H__
#define __ND_BRAIN_CONTEXT_H__

#include "ndBrainStdafx.h"

class ndBrainBuffer;
class ndBrainVector;
class ndBrainContext;
class ndBrainCpuContext;
class ndBrainFloatBuffer;
class ndBrainUniformBuffer;
class ndBrainIntegerBuffer;
class ndBrainBufferCommand;

#define ND_DEFAULT_WORKGROUP_SIZE	256

class ndBrainContext : public ndClassAlloc
{
	public:
	ndBrainContext();
	virtual ~ndBrainContext();

	virtual ndBrainContext* GetAsGpuContext();
	virtual ndBrainCpuContext* GetAsCpuContext();

	static bool HasGpuSupport() { ndAssert(0); return true;}

	virtual void SyncBufferCommandQueue() = 0;
	virtual void SubmitBufferCommand(ndBrainBufferCommand* const command) = 0;

	virtual void BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector) = 0;

	virtual void MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const srcMemory) const = 0;
	virtual void MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const dstMemory) const = 0;

	virtual void CopyBufferIndirect(const ndBrainUniformBuffer& parameterBuffer, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) = 0;
};

#endif