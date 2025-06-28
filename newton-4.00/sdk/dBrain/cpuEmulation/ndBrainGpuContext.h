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
#ifndef __ND_BRAIN_GPU_CONTEXT_H__
#define __ND_BRAIN_GPU_CONTEXT_H__

#include "ndBrainStdafx.h"
#include "ndBrainContext.h"
#include "ndBrainThreadPool.h"

class ndBrainBuffer;
class ndBrainGpuCommand;
class ndBrainGpuFloatBuffer;

class ndBrainGpuShader : public ndClassAlloc
{
	public:
	ndBrainGpuShader(ndBrainGpuContext* const context)
		:ndClassAlloc()
		,m_context(context)
	{
	}

	virtual ~ndBrainGpuShader()
	{
	}

	virtual void Execute(ndInt32 groupId, ndInt32 workGroupSize) = 0;

	ndBrainGpuContext* m_context;
	ndFixSizeArray<ndBrainBuffer*, 8> m_parameters;
};

class ndBrainGpuContext : public ndBrainContext, public ndBrainThreadPool
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();
	virtual ndContextType GetType() const override;

	ndInt32 GetSubGroupSize() const { return 0; }

	void SyncQueue();
	void AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>&);

	virtual void BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector) override;
	virtual void MemoryFromDevice(const ndBrainFloatBuffer& buffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const override;
	virtual void CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData) override;
	

	static bool HasGpuSupport();

	ndBrainGpuContext* GetAsGpuContext() override;

	private:
	void CreateKerners();

	public:
	// feed foward shaders
	ndSharedPtr<ndBrainGpuShader> m_brainCopyInput;
	ndSharedPtr<ndBrainGpuShader> m_brainCopyOutput;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerReluActivation;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerTanhActivation;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerSoftmaxActivation;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerDropOutActivation;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerMatrixMatrixMultiply;

	// back propagate shaders
	ndSharedPtr<ndBrainGpuShader> m_brainCopyInputGradients;
	ndSharedPtr<ndBrainGpuShader> m_brainCopyOutputGradients;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerReluBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerTanhBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerDropOutBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerMatrixVectorBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerCathegoricalSoftmaxBackPropagate;

	// add all the partial gradinets
	ndSharedPtr<ndBrainGpuShader> m_brainAverageGradients;
	ndSharedPtr<ndBrainGpuShader> m_brainAccumulateGradients;
	ndSharedPtr<ndBrainGpuShader> m_brainAccumulateGradientsAndAverage;

	// optimizer shaders
	ndSharedPtr<ndBrainGpuShader> m_brainAdamMomentumUpdate;
	ndSharedPtr<ndBrainGpuShader> m_brainAdamRidgeOptimizerUpdate;
	ndSharedPtr<ndBrainGpuShader> m_brainAdamLassoOptimizerUpdate;
};
#endif