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
#ifndef __ND_BRAIN_CPU_CONTEXT_H__
#define __ND_BRAIN_CPU_CONTEXT_H__

#include "ndBrainStdafx.h"
#include "ndBrainContext.h"
#include "ndBrainThreadPool.h"

class ndBrainCpuContext : public ndBrainContext, public ndBrainThreadPool
{
	public:
	ndBrainCpuContext();
	virtual ~ndBrainCpuContext();

	virtual void SyncBufferCommandQueue() override;
	virtual ndBrainCpuContext* GetAsCpuContext() override;

	virtual void SubmitBufferCommand(ndBrainBufferCommand* const command) override;

	virtual void MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const srcMemory) const override;
	virtual void MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const override;

	virtual void CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;
	virtual void CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;
	virtual void CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;

	virtual void BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector) override;
	virtual void BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector) override;

	virtual void Set(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Max(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Sub(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void LessEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void GreaterEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, const ndBrainFloatBuffer& blend) override;

	virtual void Set(ndBrainFloatBuffer& buffer, ndBrainFloat value) override;
	virtual void Min(ndBrainFloatBuffer& buffer, ndBrainFloat value) override;
	virtual void Max(ndBrainFloatBuffer& buffer, ndBrainFloat value) override;
	virtual void Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale) override;
	virtual void LessEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void GreaterEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat blend) override;
	virtual void Select(ndBrainFloatBuffer& buffer, ndBrainFloatBuffer& mask, ndBrainFloat a, ndBrainFloat b) override;
	virtual void ScaleAdd(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat scale) override;

	virtual void BroadcastScaler(ndBrainFloatBuffer& buffer, ndInt32 bufferStrideInFloats, const ndBrainFloatBuffer& srcScalar) override;
	virtual void StandardNormalDistribution(ndBrainFloatBuffer& uniformRandomVariable) override;
};

#endif