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

typedef cl::Kernel <ndBrainKernel>;

#define ND_KERNELS_WORKGROUP_SIZE	256

class ndBrainGpuContext : public ndBrainContext
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();
	virtual ndContextType GetType() const override;

	void SyncQueue();
	void AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>& command); 

	static bool HasGpuSupport();
	ndBrainGpuContext* GetAsGpuContext() override;

	private:
	void CreateKerners();
	void CreateCopyCommands();
	bool SupportsMappedMemory() const;
	size_t GetDeviceScore(cl::Device& device);

	void CopyBuffer(ndBrainGpuUniformBuffer& parameterBuffer, ndInt32 workGroups, ndBrainGpuFloatBuffer& dstBuffer, ndBrainGpuFloatBuffer& srcBuffer);
	void CopyBufferIndirect(ndBrainGpuUniformBuffer& parameterBuffer, ndBrainGpuIntegerBuffer& indexBuffer, ndBrainGpuFloatBuffer& dstBuffer, ndBrainGpuFloatBuffer& srcBuffer);

	ndSharedPtr<ndBrainKernel> CreateKerner(const cl::Program& program, const char* const functionMame) const;
	static void CL_CALLBACK clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data);

	public:
	// feed forward shaders
	ndSharedPtr<ndBrainKernel> m_brainCopyInput;
	ndSharedPtr<ndBrainKernel> m_brainCopyOutput;
	ndSharedPtr<ndBrainKernel> m_brainLayerReluActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerTanhActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerSoftmaxActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerDropOutActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixMatrixMultiply;

	// back propagate shaders
	ndSharedPtr<ndBrainKernel> m_brainCopyInputGradients;
	ndSharedPtr<ndBrainKernel> m_brainCopyOutputGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerReluBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerTanhBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixVectorBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerDropOutBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerCathegoricalSoftmaxBackPropagate;

	// add all the partial gradients
	ndSharedPtr<ndBrainKernel> m_brainAverageGradients;
	ndSharedPtr<ndBrainKernel> m_brainAccumulateGradients;
	ndSharedPtr<ndBrainKernel> m_brainAccumulateGradientsAndAverage;

	// optimizer shaders
	ndSharedPtr<ndBrainKernel> m_brainAdamMomentumUpdate;
	ndSharedPtr<ndBrainKernel> m_brainAdamRidgeOptimizerUpdate;
	ndSharedPtr<ndBrainKernel> m_brainAdamLassoOptimizerUpdate;

	// other shaders
	ndSharedPtr<ndBrainKernel> m_brainCopyBuffer;
	ndSharedPtr<ndBrainKernel> m_brainCopyBufferIndirect;
	
	private:
	ndSharedPtr<cl::Device> m_device;
	ndSharedPtr<cl::Context> m_context;
	ndSharedPtr<cl::CommandQueue> m_queue;
	cl::Buffer m_emptyBuffer;
	bool m_supportsMappedMemory;

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