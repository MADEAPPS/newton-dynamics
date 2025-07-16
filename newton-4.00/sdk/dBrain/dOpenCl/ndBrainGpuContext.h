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
#include "ndBrainKernel.h"
#include "ndBrainContext.h"

class ndBrainBuffer;
class ndBrainKernel;
class ndBrainGpuCommand;
class ndBrainFloatBuffer;
class ndBrainIntegerBuffer;
class ndBrainUniformBuffer;

class ndBrainGpuContext : public ndBrainContext
{
	public:
	class OpenclKernel : public ndBrainKernel
	{
		public:
		OpenclKernel(ndBrainContext* const context, const ndSharedPtr<cl::Kernel>& kenel)
			:ndBrainKernel(context)
			,m_shader(kenel)
		{
		}

		void Execute(ndInt32, ndInt32) override
		{
			ndAssert(0);
		}

		ndSharedPtr<cl::Kernel> m_shader;
	};

	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();

	virtual void SyncBufferCommandQueue() override;
	virtual ndBrainGpuContext* GetAsGpuContext() override;

	virtual void SubmitBufferCommand(ndBrainBufferCommand* const command) override;

	virtual void BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector) override;
	virtual void BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector) override;

	virtual void MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const srcMemory) const override;
	virtual void MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const override;

	virtual void CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;
	virtual void CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;
	virtual void CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;

	virtual void Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale) override;
	virtual void Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Sub(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;

	private:
	void CreateKerners();
	void CreateCopyCommands();
	bool SupportsMappedMemory() const;
	size_t GetDeviceScore(cl::Device& device);
	ndSharedPtr<ndBrainKernel> CreateKerner(const cl::Program& program, const char* const functionMame) const;
	static void CL_CALLBACK clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data);

	void SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer);

	ndSharedPtr<cl::Device> m_device;
	ndSharedPtr<cl::Context> m_context;
	ndSharedPtr<cl::CommandQueue> m_queue;
	cl::Buffer m_emptyBuffer;

	bool m_supportMappedMemory;

	public:
	// feed foward shaders
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
	ndSharedPtr<ndBrainKernel> m_brainLayerDropOutBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerCathegoricalSoftmaxBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateInputGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateWeightGradients;

	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateBiasGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateAddBiasGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateClearBiasGradients;

	// optimizer shaders
	ndSharedPtr<ndBrainKernel> m_brainAdamMomentumUpdate;
	ndSharedPtr<ndBrainKernel> m_brainAdamRidgeOptimizerUpdate;
	ndSharedPtr<ndBrainKernel> m_brainAdamLassoOptimizerUpdate;

	// shader strided buffers
	ndSharedPtr<ndBrainKernel> m_brainCopyStridedBuffer;
	ndSharedPtr<ndBrainKernel> m_brainCopyStridedBufferIndirect;
	ndSharedPtr<ndBrainUniformBuffer> m_copyStridedBufferParams;
	ndSharedPtr<ndBrainGpuCommand> m_copyStridedBufferCommand;
	ndSharedPtr<ndBrainGpuCommand> m_copyStridedBufferIndirectCommand;

	// math operations kernels
	ndSharedPtr<ndBrainKernel> m_mathBufferAssigment;

	static const char* m_mathOpsCommand;
	static const char* m_matrixMultiply;
	static const char* m_optimizerKernels;
	static const char* m_commonKernelsInclude;
	static const char* m_otherShaderFunctions;
	static const char* m_feedForwardKernels_1;
	static const char* m_feedForwardKernels_2;
	static const char* m_feedForwardKernels_3;
	static const char* m_backPropagateKernels_1;
	static const char* m_backPropagateKernels_2;
	static const char* m_matrixWeightsAndBiasGradients;

	friend class ndBrainGpuBuffer;
};

#endif