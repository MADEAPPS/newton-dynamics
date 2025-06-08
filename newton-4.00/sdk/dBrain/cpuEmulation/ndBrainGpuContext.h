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

class ndBrainGpuBuffer;
class ndBrainGpuCommand;
class ndBrainGpuFloatBuffer;

typedef struct
{
	ndUnsigned32 m_inputSize;
	ndUnsigned32 m_outputSize;
	ndUnsigned32 m_parametersBatchSize;
	ndUnsigned32 m_parametersStartOffset;
	ndUnsigned32 m_inputOutputSize;
	ndUnsigned32 m_inputOutputStartOffset;
	ndUnsigned32 m_unused[4];
} UniformBufferObject;

#define ND_KERNELS_WORKGROUP_SIZE	256

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
	ndFixSizeArray<ndBrainGpuBuffer*, 8> m_parameters;
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

	static bool HasGpuSupport();

	ndBrainGpuContext* GetAsGpuContext() override;

	private:
	void CreateKerners();

	public:
	// feed foward shaders
	ndSharedPtr<ndBrainGpuShader> m_ndBrainCopyInput;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainCopyOutput;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerLinear;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerReluActivation;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerTanhActivation;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerSoftmaxActivation;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerLinearDropOutActivation;

	// back prpagate shaders
	ndSharedPtr<ndBrainGpuShader> m_ndBrainCopyInputGradients;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainCopyOutputGradients;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerReluBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerTanhBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerLinearBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerLinearDropOutBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerCathegoricalSoftmaxBackPropagate;
};
#endif