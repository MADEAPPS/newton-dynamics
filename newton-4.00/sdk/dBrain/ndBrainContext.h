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
class ndBrainGpuContext;
class ndBrainFloatBuffer;
class ndBrainUniformBuffer;
class ndBrainIntegerBuffer;
class ndBrainBufferCommand;
class ndCopyBufferCommandInfo;

#define ND_DEFAULT_WORKGROUP_SIZE	256

class ndBrainContext : public ndClassAlloc
{
	public:
	ndBrainContext();
	virtual ~ndBrainContext();

	virtual ndBrainGpuContext* GetAsGpuContext();
	virtual ndBrainCpuContext* GetAsCpuContext();

	static bool HasGpuSupport() { ndAssert(0); return true;}

	virtual void SyncBufferCommandQueue() = 0;
	virtual void SubmitBufferCommand(ndBrainBufferCommand* const command) = 0;

	virtual void MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const srcMemory) const = 0;
	virtual void MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const dstMemory) const = 0;

	virtual void CopyBuffer(const ndCopyBufferCommandInfo& parameterBuffer, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) = 0;
	virtual void CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) = 0;

	virtual void BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector) = 0;
	virtual void BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector) = 0;

	virtual void Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale) = 0;
	virtual void Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) = 0;
	virtual void Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) = 0;
	virtual void Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) = 0;
};

#endif