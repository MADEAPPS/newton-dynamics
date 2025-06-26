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

class ndBrainGpuCommand;
class ndBrainGpuFloatBuffer;
class ndBrainGpuUniformBuffer;
class ndBrainGpuIntegerBuffer;

typedef cl::Kernel ndBrainGpuShader;

#define ND_KERNELS_WORKGROUP_SIZE	256

class ndBrainGpuContext : public ndBrainContext
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();
	virtual ndContextType GetType() const override;

	void SyncQueue();
	void AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>& command); 

	ndInt32 GetSubGroupSize() const { return 0; }
	static bool HasGpuSupport();

	ndBrainGpuContext* GetAsGpuContext() override;

	private:
	void CreateKerners();
	void CreateCopyIndirectCommand();
	size_t GetDeviceScore(cl::Device& device);

	void CopyBuffer(ndBrainGpuUniformBuffer& parameterBuffer, ndInt32 workGroups, ndBrainGpuFloatBuffer& dstBuffer, ndBrainGpuFloatBuffer& srcBuffer);
	void CopyBufferIndirect(ndBrainGpuUniformBuffer& parameterBuffer, ndBrainGpuIntegerBuffer& indexBuffer, ndBrainGpuFloatBuffer& dstBuffer, ndBrainGpuFloatBuffer& srcBuffer);

	ndSharedPtr<ndBrainGpuShader> CreateKerner(const cl::Program& program, const char* const functionMame) const;
	static void CL_CALLBACK clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data);

	public:
	// feed forward shaders
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
	ndSharedPtr<ndBrainGpuShader> m_brainLayerMatrixVectorBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerDropOutBackPropagate;
	ndSharedPtr<ndBrainGpuShader> m_brainLayerCathegoricalSoftmaxBackPropagate;

	// add all the partial gradients
	ndSharedPtr<ndBrainGpuShader> m_brainAverageGradients;
	ndSharedPtr<ndBrainGpuShader> m_brainAccumulateGradients;
	ndSharedPtr<ndBrainGpuShader> m_brainAccumulateGradientsAndAverage;

	// optimizer shaders
	ndSharedPtr<ndBrainGpuShader> m_brainAdamMomentumUpdate;
	ndSharedPtr<ndBrainGpuShader> m_brainAdamRidgeOptimizerUpdate;
	ndSharedPtr<ndBrainGpuShader> m_brainAdamLassoOptimizerUpdate;

	// other shaders
	ndSharedPtr<ndBrainGpuShader> m_brainCopyBuffer;
	ndSharedPtr<ndBrainGpuShader> m_brainCopyBufferIndirect;
	
	private:
	ndSharedPtr<cl::Device> m_device;
	ndSharedPtr<cl::Context> m_context;
	ndSharedPtr<cl::CommandQueue> m_queue;
	cl::Buffer m_emptyBuffer;

	ndSharedPtr<ndBrainGpuCommand> m_copyBufferCommand;
	ndSharedPtr<ndBrainGpuCommand> m_copyBufferIndirectCommand;

	static const char* m_feedForwardKernels_1;
	static const char* m_feedForwardKernels_2;
	static const char* m_feedForwardKernels_3;

	static const char* m_backPropagateKernels_1;
	static const char* m_backPropagateKernels_2;
	
	static const char* m_matrixMultiply;
	static const char* m_optimizerKernels;
	static const char* m_commonKernelsInclude;

	static const char* m_otherShaderFunctions;

	friend class ndBrainGpuBuffer;
	friend class ndBrainGpuCommand;
	friend class ndBrainGpuFloatBuffer;
	friend class ndBrainGpuUniformBuffer;
};
#endif