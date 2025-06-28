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

#define ND_DEFAULT_WORKGROUP_SIZE	256

class ndBrainContext : public ndClassAlloc
{
	public:
	enum ndContextType
	{
		m_cpu,
		m_gpu,
	};
	ndBrainContext();
	virtual ~ndBrainContext();

	virtual ndContextType GetType() const = 0;
	virtual ndBrainCpuContext* GetAsCpuContext();
	virtual ndBrainContext* GetAsGpuContext();

	static bool HasGpuSupport() { ndAssert(0); return true;}

	virtual void BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector) = 0;

	virtual void MemoryFromDevice(const ndBrainFloatBuffer& buffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const = 0;
	virtual void CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData) = 0;
};

#endif