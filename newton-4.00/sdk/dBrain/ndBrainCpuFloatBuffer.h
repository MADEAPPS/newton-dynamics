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
#ifndef __ND_BRAIN_CPU_FLOAT_BUFFER_H__
#define __ND_BRAIN_CPU_FLOAT_BUFFER_H__

#include "ndBrainBuffer.h"

class ndBrainCpuFloatBuffer : public ndBrainBuffer
{
	public:
	ndBrainCpuFloatBuffer(ndBrainContext* const context, ndInt64 size);
	ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input);
	ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix);

	virtual void BrainVectorToDevice(const ndBrainVector& vector) override;
	virtual void BrainVectorFromDevice(ndBrainVector& vector) const override;

	virtual void BrainMatrixToDevice(const ndBrainMatrix* const matrix) override;

	virtual void MemoryToDevive(size_t sizeInBytes, const void* const inputData) override;
	virtual void MemoryFromDevive(size_t sizeInBytes, void* const outputMemory) const override;

	virtual void CopyBuffer(const ndBrainBuffer& source, size_t sourceOffsetInBytes, size_t dstOffsetInBytes, size_t sizeInBytes) override;

	protected:
	virtual void LoadData(size_t sizeInBytes, const void* const sourceData) override;
	virtual void UnloadData(size_t sizeInBytes, void* const outputData) const override;

	ndBrainVector m_buffer;

	friend class ndBrainTrainerCpu;
	friend class ndBrainTrainerCpuInference;
};

#endif