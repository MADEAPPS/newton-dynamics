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

	virtual bool IsValid() const;
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
	virtual void Less(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale) override;
	virtual void Greater(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void LessEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void GreaterEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test) override;
	virtual void Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat blend) override;
	virtual void Select(ndBrainFloatBuffer& buffer, ndBrainFloatBuffer& mask, ndBrainFloat a, ndBrainFloat b) override;
	virtual void ScaleAdd(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat scale) override;

	virtual void StandardNormalDistribution(ndBrainFloatBuffer& uniformRandomVariable) override;
	virtual void BroadcastScaler(ndBrainFloatBuffer& buffer, ndInt32 bufferStrideInFloats, const ndBrainFloatBuffer& srcScalar) override;
	virtual void CalculateEntropyRegularization(ndBrainFloatBuffer&, const ndBrainFloatBuffer&, const ndBrainFloatBuffer&, ndBrainFloat) override;
	virtual void CalculateEntropyRegularizationGradient(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer, ndBrainFloat regularization, ndInt32 inputSize) override;

	private:
	void CreateQueue();
	void CreateKerners();
	void CreateCopyCommands();
	size_t GetDeviceScore(cl::Device& device);
	ndSharedPtr<ndBrainKernel> CreateKerner(const cl::Program& program, const char* const functionMame) const;
	static void CL_CALLBACK clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data);

	void SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer);
	void SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, float scale);
	void SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer);
	void SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer, float scale);
	void SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer0, const ndBrainBuffer* const srcBuffer1);

	ndSharedPtr<cl::Device> m_device;
	ndSharedPtr<cl::Context> m_context;
	ndSharedPtr<cl::CommandQueue> m_queue;
	cl::Buffer m_emptyBuffer;
	size_t m_maxBufferSize;
	bool m_isValid;

	public:
	// feed forward shaders
	ndSharedPtr<ndBrainKernel> m_brainCopyInput;
	ndSharedPtr<ndBrainKernel> m_brainCopyOutput;
	ndSharedPtr<ndBrainKernel> m_brainLayerReluActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerTanhActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerLinearActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerSoftmaxActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerDropOutActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerLeakyReluActivation;
	ndSharedPtr<ndBrainKernel> m_brainLayerMatrixMatrixMultiply;
	ndSharedPtr<ndBrainKernel> m_brainLayerOffPolicyActivation;

	// back propagate shaders
	ndSharedPtr<ndBrainKernel> m_brainCopyInputGradients;
	ndSharedPtr<ndBrainKernel> m_brainCopyOutputGradients;
	ndSharedPtr<ndBrainKernel> m_brainLayerLinearPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerReluBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerTanhBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerDropOutBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerLeakyReluBackPropagate;
	ndSharedPtr<ndBrainKernel> m_brainLayerOffPolicyBackPropagate;
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

	// shader stride buffers
	ndSharedPtr<ndBrainKernel> m_brainCopyStridedBuffer;
	ndSharedPtr<ndBrainKernel> m_brainCopyStridedBufferIndirect;
	ndSharedPtr<ndBrainUniformBuffer> m_copyStridedBufferParams;
	ndSharedPtr<ndBrainGpuCommand> m_copyStridedBufferCommand;
	ndSharedPtr<ndBrainGpuCommand> m_copyStridedBufferIndirectCommand;

	// arithmetic operations kernels
	ndSharedPtr<ndBrainKernel> m_brainSet;
	ndSharedPtr<ndBrainKernel> m_brainAdd;
	ndSharedPtr<ndBrainKernel> m_brainSub;
	ndSharedPtr<ndBrainKernel> m_brainMul;
	ndSharedPtr<ndBrainKernel> m_brainMin;
	ndSharedPtr<ndBrainKernel> m_brainMax;
	ndSharedPtr<ndBrainKernel> m_brainScale;
	ndSharedPtr<ndBrainKernel> m_brainSelect;
	ndSharedPtr<ndBrainKernel> m_brainScaleAdd;
	ndSharedPtr<ndBrainKernel> m_brainAssigment;
	ndSharedPtr<ndBrainKernel> m_brainLessEqual;
	ndSharedPtr<ndBrainKernel> m_brainMinScalar;
	ndSharedPtr<ndBrainKernel> m_brainMaxScalar;
	ndSharedPtr<ndBrainKernel> m_brainBlendScale;
	ndSharedPtr<ndBrainKernel> m_brainLessScalar;
	ndSharedPtr<ndBrainKernel> m_brainBlendVector;
	ndSharedPtr<ndBrainKernel> m_brainGreaterEqual;
	ndSharedPtr<ndBrainKernel> m_brainGreaterScalar;
	ndSharedPtr<ndBrainKernel> m_brainLessEqualScalar;
	ndSharedPtr<ndBrainKernel> m_brainBroadcastScalar;
	ndSharedPtr<ndBrainKernel> m_brainGreaterEqualScalar;
	ndSharedPtr<ndBrainKernel> m_brainNormalDistribution;
	ndSharedPtr<ndBrainKernel> m_brainEntropyRegularization;
	ndSharedPtr<ndBrainKernel> m_brainEntropyRegularizationGradient;

	static const char* m_mathOpsCommand;
	static const char* m_matrixMultiply;
	static const char* m_optimizerKernels;
	static const char* m_commonKernelsInclude;
	static const char* m_probabilitiesKernels;
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