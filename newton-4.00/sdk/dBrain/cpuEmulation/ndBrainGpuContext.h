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
class ndBrainKernel;
class ndBrainGpuCommand;
class ndBrainFloatBuffer;
class ndBrainIntegerBuffer;
class ndBrainUniformBuffer;

class ndBrainGpuContext : public ndBrainContext, public ndBrainThreadPool
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();

	virtual void SyncBufferCommandQueue() override;
	virtual ndBrainGpuContext* GetAsGpuContext() override;

	virtual void SubmitBufferCommand(ndBrainBufferCommand* const command) override;

	virtual void MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const srcMemory) const override;
	virtual void MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const override;

	virtual void CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;
	virtual void CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;
	virtual void CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData) override;

	virtual void BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector) override;
	virtual void BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector) override;

	virtual void Exp(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Set(ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData) override;
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
	virtual void Less(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale) override;
	virtual void Greater(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void LessEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void GreaterEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;

	virtual void Reciprocal(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer) override;
	virtual void Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat blend) override;
	virtual void Select(ndBrainFloatBuffer& buffer, ndBrainFloatBuffer& mask, ndBrainFloat a, ndBrainFloat b) override;
	virtual void ScaleAdd(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat scale) override;

	virtual void SetOrdinal(ndBrainFloatBuffer& buffer) override;
	virtual void ReductionSum(ndBrainFloatBuffer& buffer) override;
	virtual void StandardNormalDistribution(ndBrainFloatBuffer& uniformRandomVariable) override;
	virtual void BroadcastScaler(ndBrainFloatBuffer& buffer, ndInt32 bufferStrideInFloats, const ndBrainFloatBuffer& srcScalar) override;
	virtual void CalculateEntropyRegularization(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer, ndBrainFloat regularization) override;
	virtual void CalculateEntropyRegularizationGradient(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer, ndBrainFloat regularization, ndInt32 inputSize) override;

	virtual void Sqrt(ndBrainFloatBuffer&, ndInt32 clipSize) override;
	virtual void InvSqrt(ndBrainFloatBuffer&, ndInt32 clipSize) override;
	virtual void ReductionSum(ndBrainFloatBuffer& buffer, ndInt32 clipSize) override;

	// learnRate commands
	virtual void ApplyLeanRateCommands(ndBrainBufferCommand* const command, ndBrainFloat learRate) override;
	virtual void SetLearnRateCommandBuffers(ndBrainOptimizerAdam& optimizer, ndInt32 minibatchSize, ndBrainFloatBuffer& weightsAndBiasBuffer, ndBrainFloatBuffer& weightsAndBiasGradientBuffer) override;

	private:
	void CreateKerners();
	void CreateCopyCommands();

	ndSharedPtr<ndBrainUniformBuffer> m_copyBufferParams;
	ndSharedPtr<ndBrainGpuCommand> m_copyBufferCommand;
	ndSharedPtr<ndBrainGpuCommand> m_copyBufferIndirectCommand;

	public:
	// feed foward shaders
	ndSharedPtr<ndBrainKernel> m_brainCopyInput;
	ndSharedPtr<ndBrainKernel> m_brainCopyOutput;
	ndSharedPtr<ndBrainKernel> m_brainLayerReluActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerTanhActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerLinearActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerSoftmaxActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerDropOutActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerLeakyReluActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerPolicyGradientActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixMatrixMultiply;

	// back propagate shaders
	ndSharedPtr<ndBrainKernel> m_brainCopyInputGradients;
	ndSharedPtr<ndBrainKernel> m_brainCopyOutputGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerLinearPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerReluBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerTanhBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerDropOutBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerLeakyReluBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerPolicyGradientBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerCathegoricalSoftmaxBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateBiasGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateInputGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateWeightGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateAddBiasGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixBackPropagateClearBiasGradients;

	// optimizer shaders
	ndSharedPtr<ndBrainKernel> m_brainAdamMomentumUpdate;
	ndSharedPtr<ndBrainKernel> m_brainAdamRidgeOptimizerUpdate;
	ndSharedPtr<ndBrainKernel> m_brainAdamLassoOptimizerUpdate;

	// other shader
	ndSharedPtr<ndBrainKernel> m_brainCopyBuffer;
	ndSharedPtr<ndBrainKernel> m_brainCopyBufferIndirect;
};
#endif