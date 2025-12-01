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

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainKernel.h"
#include "ndBrainCpuContext.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainBufferCommand.h"
#include "ndBrainTrainerInference.h"

class ndBrainAdamUpdateParametersRidge : public ndBrainBufferCommandCpu
{
	public:
	ndBrainAdamUpdateParametersRidge(const ndBrainBufferCommandDesc& desc)
		:ndBrainBufferCommandCpu(desc)
		,m_learnRate(ndBrainFloat(1.0e-4f))
	{
	}

	//#pragma optimize( "", off )
	virtual void Execute(ndInt32 groupId) override
	{
		ndInt32 workGroupSize = m_desc.m_workGroupSize;

		const ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)m_desc[0]->GetCpuPtr();
		ndBrainFloat* const weightAndBiasBuffer = (ndBrainFloat*)m_desc[1]->GetCpuPtr();
		ndBrainFloat* const weightAndBiasGradientBuffer = (ndBrainFloat*)m_desc[2]->GetCpuPtr();
		ndBrainFloat* const vdw = (ndBrainFloat*)m_desc[3]->GetCpuPtr();
		ndBrainFloat* const vdw2 = (ndBrainFloat*)m_desc[4]->GetCpuPtr();

		ndBrainFloat descendRate = -m_learnRate;
		ndBrainFloat regularizer = -parameters->m_decayRegularizer;

		ndInt32 start = groupId * workGroupSize;
		ndBrainFloat miniBatchWeight = parameters->m_minibathScale;
		for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
		{
			ndBrainFloat veloc = vdw[start + itemId];
			ndBrainFloat accel = vdw2[start + itemId];
			ndBrainFloat posit = miniBatchWeight * weightAndBiasGradientBuffer[start + itemId];

			// calculate moving average
			ndBrainFloat a = veloc * parameters->m_alpha + posit * (ndBrainFloat(1.0f) - parameters->m_alpha);

			// calculate RMS
			ndBrainFloat b = accel * parameters->m_beta + posit * posit * (ndBrainFloat(1.0f) - parameters->m_beta);

			// save veloc and accel for net update
			vdw[start + itemId] = a;
			vdw2[start + itemId] = b;

			// apply correction in initalization, until bias becomes 1.0
			ndBrainFloat vdwCorrected = a * parameters->m_invAlpha;
			ndBrainFloat vdw2Corrected = b * parameters->m_invBeta;

			ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected)) + parameters->m_epsilon);
			ndBrainFloat gradient = vdwCorrected * bias_den;

			ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
			gradient += weight * regularizer;
			weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
		}
	}

	ndBrainFloat m_learnRate;
};

class ndBrainAdamMomentumUpdate : public ndBrainBufferCommandCpu
{
	public:
	ndBrainAdamMomentumUpdate(const ndBrainBufferCommandDesc& desc)
		:ndBrainBufferCommandCpu(desc)
	{
	}

	//#pragma optimize( "", off )
	virtual void Execute(ndInt32) override
	{
		ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)m_desc[0]->GetCpuPtr();

		parameters->m_betaAcc *= parameters->m_beta;
		parameters->m_alphaAcc *= parameters->m_alpha;
		if (parameters->m_betaAcc < ndBrainFloat(1.0e-7f))
		{
			parameters->m_betaAcc = ndBrainFloat(0.0f);
		}
		if (parameters->m_alphaAcc < ndBrainFloat(1.0e-7f))
		{
			parameters->m_alphaAcc = ndBrainFloat(0.0f);
		}

		parameters->m_invBeta = (ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - parameters->m_betaAcc));
		parameters->m_invAlpha = (ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - parameters->m_alphaAcc));
	}
};

ndBrainCpuContext::ndBrainCpuContext()
	:ndBrainContext()
	,ndBrainThreadPool()
{
	ndInt32 numOfThreads = ndBrainThreadPool::GetMaxThreads();
#ifdef _DEBUG
numOfThreads = 1;
#endif

	SetThreadCount(numOfThreads);
}

ndBrainCpuContext::~ndBrainCpuContext()
{
}

ndBrainCpuContext* ndBrainCpuContext::GetAsCpuContext()
{
	return this;
}

void ndBrainCpuContext::SyncBufferCommandQueue()
{
	// do nothing
}

void ndBrainCpuContext::CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndAssert(dstData.SizeInBytes() == srcData.SizeInBytes());
	ndAssert((dstData.SizeInBytes() & (sizeof(ndInt32) - 1)) == 0);

	ndUnsigned32* const dst = (ndUnsigned32*)dstData.GetCpuPtr();
	const ndUnsigned32* const src = (ndUnsigned32*)srcData.GetCpuPtr();
	ndMemCpy(dst, src, ndInt64(dstData.SizeInBytes() / sizeof (ndInt32)));
}

void ndBrainCpuContext::CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 numberOfWorkGroups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndInt32 stride = ndInt32(descriptor.m_strideInByte);
	ndInt32 srcStride = ndInt32(descriptor.m_srcStrideInByte);
	ndInt32 srcOffset = ndInt32(descriptor.m_srcOffsetInByte);
	ndInt32 dstStride = ndInt32(descriptor.m_dstStrideInByte);
	ndInt32 dstOffset = ndInt32(descriptor.m_dstOffsetInByte);

	ndAssert(stride <= srcStride);
	ndAssert(stride <= dstStride);

	ndUnsigned8* const dst = (ndUnsigned8*)dstData.GetCpuPtr();
	const ndUnsigned8* const src = (ndUnsigned8*)srcData.GetCpuPtr();
	ndAssert(dst);
	ndAssert(src);

	for (ndInt32 i = 0; i < numberOfWorkGroups; ++i)
	{
		ndMemCpy(&dst[i * dstStride + dstOffset], &src[i * srcStride + srcOffset], stride);
	}
}

void ndBrainCpuContext::CopyBufferIndirect(const ndCopyBufferCommandInfo& descriptor, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndInt32 stride = ndInt32(descriptor.m_strideInByte);
	ndInt32 srcStride = ndInt32(descriptor.m_srcStrideInByte);
	ndInt32 srcOffset = ndInt32(descriptor.m_srcOffsetInByte);
	ndInt32 dstStride = ndInt32(descriptor.m_dstStrideInByte);
	ndInt32 dstOffset = ndInt32(descriptor.m_dstOffsetInByte);

	ndAssert(stride <= srcStride);
	ndAssert(stride <= dstStride);

	ndUnsigned8* const dst = (ndUnsigned8*)dstData.GetCpuPtr();
	const ndUnsigned8* const src = (ndUnsigned8*)srcData.GetCpuPtr();
	const ndUnsigned32* const indexPtr = (ndUnsigned32*)indexBuffer.GetCpuPtr();
	ndAssert(dst);
	ndAssert(src);
	ndAssert(indexPtr);

	ndInt32 count = ndInt32(indexBuffer.SizeInBytes() / sizeof(ndUnsigned32));
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndUnsigned32 index = indexPtr[i];
		ndMemCpy(&dst[i * dstStride + dstOffset], &src[index * srcStride + srcOffset], stride);
	}
}

void ndBrainCpuContext::MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	const ndInt8* const src = (ndInt8*)deviceBuffer.GetCpuPtr();
	ndAssert(src);
	ndMemCpy((ndInt8*)outputMemory, &src[offsetInBytes], ndInt64(sizeInBytes));
}

void ndBrainCpuContext::MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory) const
{
	const ndInt8* const src = (ndInt8*)inputMemory;
	ndInt8* const dst = (ndInt8*)deviceBuffer.GetCpuPtr();
	ndAssert(dst);
	ndAssert(src);
	ndMemCpy(&dst[offsetInBytes], src, ndInt64(sizeInBytes));
}

void ndBrainCpuContext::SubmitBufferCommand(ndBrainBufferCommand* const command)
{
	// adaptive thread dispacher, in general 50% faster
	auto ExecuteCommand = ndMakeObject::ndFunction([this, command](ndInt32 groupId, ndInt32)
	{
		ndBrainBufferCommandCpu* const cpuCommand = (ndBrainBufferCommandCpu*)command;
		cpuCommand->Execute(groupId);
	});

	const ndBrainBufferCommandDesc& descriptor = command->GetDescriptor();
	ParallelExecute(ExecuteCommand, descriptor.m_miniBatchSize);
}

void ndBrainCpuContext::BrainVectorToDevice(ndBrainFloatBuffer& buffer, const ndBrainVector& srcVector)
{
	size_t sizeInBytes = ndMin(size_t(buffer.SizeInBytes()), size_t(srcVector.GetCount() * sizeof(ndReal)));
	MemoryToDevice(buffer, 0, sizeInBytes, &srcVector[0]);
}

void ndBrainCpuContext::BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector)
{
	ndAssert(0);
	size_t sizeInBytes = ndMin(size_t(src.SizeInBytes()), size_t(dstVector.GetCount() * sizeof(ndReal)));
	MemoryFromDevice(src, 0, sizeInBytes, &dstVector[0]);
}

void ndBrainCpuContext::Set(ndBrainFloatBuffer& dstData, ndBrainFloat value)
{
	ndBrainVector& dst = **dstData.m_buffer;
	dst.Set(value);
}

void ndBrainCpuContext::Set(ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData)
{
	ndBrainVector& dst = **dstData.m_buffer;
	const ndBrainVector& src = **srcData.m_buffer;
	dst.Set(src);
}

void ndBrainCpuContext::Exp(ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData)
{
	ndBrainVector& dst = **dstData.m_buffer;
	const ndBrainVector& src = **srcData.m_buffer;
	dst.Exp(src);
}

void ndBrainCpuContext::Reciprocal(ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData)
{
	ndBrainVector& dst = **dstData.m_buffer;
	dst.Reciprocal(**srcData.m_buffer);
}

void ndBrainCpuContext::SetOrdinal(ndBrainFloatBuffer& dstData)
{
	ndBrainVector& dst = **dstData.m_buffer;
	dst.SetOrdinal();
}

void ndBrainCpuContext::ReductionSum(ndBrainFloatBuffer& dstData)
{
	ndBrainVector& dst = **dstData.m_buffer;
	dst.ReductionSum();
}

void ndBrainCpuContext::Sqrt(ndBrainFloatBuffer& dstData, ndInt32 clipSize)
{
	ndBrainVector& dst = **dstData.m_buffer;
	ndAssert(clipSize <= dst.GetCount());
	ndBrainMemVector clipDst(&dst[0], clipSize);
	clipDst.Sqrt();
}

void ndBrainCpuContext::InvSqrt(ndBrainFloatBuffer& dstData, ndInt32 clipSize)
{
	ndBrainVector& dst = **dstData.m_buffer;
	ndAssert(clipSize <= dst.GetCount());
	ndBrainMemVector clipDst(&dst[0], clipSize);
	clipDst.InvSqrt();
}

void ndBrainCpuContext::ReductionSum(ndBrainFloatBuffer& dstData, ndInt32 clipSize)
{
	ndBrainVector& dst = **dstData.m_buffer;
	ndAssert(clipSize <= dst.GetCount());
	ndBrainMemVector clipDst(&dst[0], clipSize);
	clipDst.ReductionSum();
}

void ndBrainCpuContext::Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.Scale(scale);
}

void ndBrainCpuContext::Min(ndBrainFloatBuffer& buffer, ndBrainFloat value)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.Min(value);
}

void ndBrainCpuContext::Max(ndBrainFloatBuffer& buffer, ndBrainFloat value)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.Max(value);
}

void ndBrainCpuContext::Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Min(src);
}

void ndBrainCpuContext::Max(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Max(src);
}

void ndBrainCpuContext::Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Add(src);
}

void ndBrainCpuContext::Sub(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Sub(src);
}

void ndBrainCpuContext::Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Mul(src);
}

void ndBrainCpuContext::ScaleAdd(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat scale)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.ScaleAdd(src, scale);
}

void ndBrainCpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat blend)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.Blend(src, blend);
}

void ndBrainCpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, const ndBrainFloatBuffer& blendBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	const ndBrainVector& blend = **blendBuffer.m_buffer;
	dst.Blend(src, blend);
}

void ndBrainCpuContext::Less(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.Less(test);
}

void ndBrainCpuContext::Greater(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.Greater(test);
}

void ndBrainCpuContext::LessEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.LessEqual(test);
}

void ndBrainCpuContext::LessEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.LessEqual(src);
}

void ndBrainCpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndBrainVector& dst = **buffer.m_buffer;
	dst.GreaterEqual(test);
}

void ndBrainCpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcBuffer.m_buffer;
	dst.GreaterEqual(src);
}

void ndBrainCpuContext::Select(ndBrainFloatBuffer& buffer, ndBrainFloatBuffer& maskBuffer, ndBrainFloat a, ndBrainFloat b)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& mask = **maskBuffer.m_buffer;
	dst.Blend(mask, a, b);
}

void ndBrainCpuContext::BroadcastScaler(ndBrainFloatBuffer& buffer, ndInt32 bufferStrideInFloats, const ndBrainFloatBuffer& srcScalar)
{
	ndBrainVector& dst = **buffer.m_buffer;
	const ndBrainVector& src = **srcScalar.m_buffer;

	for (ndInt32 i = 0; i < src.GetCount(); ++i)
	{
		ndBrainMemVector subVector(&dst[i * bufferStrideInFloats], bufferStrideInFloats);
		subVector.Set(src[i]);
	}
}

void ndBrainCpuContext::StandardNormalDistribution(ndBrainFloatBuffer& uniformRandomVariable)
{
	ndInt32 elements = ndInt32(uniformRandomVariable.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)uniformRandomVariable.GetCpuPtr(), elements);
	dst.StandardNormalDistribution();
}

void ndBrainCpuContext::CalculateEntropyRegularization(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer, ndBrainFloat regularization)
{
	const ndInt32 stride = ndInt32(sigmaBuffer.SizeInBytes() / buffer.SizeInBytes());
	ndAssert(sampleBuffer.SizeInBytes() == sigmaBuffer.SizeInBytes());
	ndAssert(stride * buffer.SizeInBytes() == sampleBuffer.SizeInBytes());

	const ndInt32 numberOfGroups = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));

	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), numberOfGroups);
	const ndBrainMemVector sample((ndBrainFloat*)sampleBuffer.GetCpuPtr(), stride * numberOfGroups);
	const ndBrainMemVector sigmas((ndBrainFloat*)sigmaBuffer.GetCpuPtr(), stride * numberOfGroups);

	for (ndInt32 i = 0; i < numberOfGroups; ++i)
	{
		const ndBrainMemVector meanSample(&sample[i * stride], stride);
		const ndBrainMemVector variance(&sigmas[i * stride], stride);
		dst[i] = meanSample.CalculateEntropyRegularization(variance, regularization);
	}
}

void ndBrainCpuContext::CalculateEntropyRegularizationGradient(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer, ndBrainFloat regularization, ndInt32 inputSize)
{
	ndAssert(sampleBuffer.SizeInBytes() == sigmaBuffer.SizeInBytes());
	ndAssert(2 * sampleBuffer.SizeInBytes() == buffer.SizeInBytes());
	const ndInt32 numberOfGroups = ndInt32(sampleBuffer.SizeInBytes() / sizeof(ndBrainFloat)) / inputSize;

	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), ndInt64(buffer.SizeInBytes() / sizeof (ndReal)));
	const ndBrainMemVector sigmas((ndBrainFloat*)sigmaBuffer.GetCpuPtr(), ndInt64(sigmaBuffer.SizeInBytes() / sizeof(ndReal)));
	const ndBrainMemVector sample((ndBrainFloat*)sampleBuffer.GetCpuPtr(), ndInt64(sampleBuffer.SizeInBytes() / sizeof(ndReal)));

	for (ndInt32 i = 0; i < numberOfGroups; ++i)
	{
		ndBrainMemVector gradient(&dst[2 * i * inputSize], 2 * inputSize);
		const ndBrainMemVector meanSample(&sample[i * inputSize], inputSize);
		const ndBrainMemVector variance(&sigmas[i * inputSize], inputSize);
		gradient.CalculateEntropyRegularizationGradient(meanSample, variance, regularization);
	}
}

void ndBrainCpuContext::SetLearnRateCommandBuffers(
	ndBrainOptimizerAdam& optimizer, ndInt32 minibatchSize, 
	ndBrainFloatBuffer& weightsAndBiasBuffer, ndBrainFloatBuffer& weightsAndBiasGradientBuffer)
{
	ndBrainOptimizerAdam::ndCommandSharedInfo optimizerData(optimizer.m_parameters);
	optimizerData.m_minibathScale = ndBrainFloat(1.0f) / ndBrainFloat(minibatchSize);
	ndSharedPtr<ndBrainUniformBuffer> adamUniformbuffer(new ndBrainUniformBuffer(this, sizeof(ndBrainOptimizerAdam::ndCommandSharedInfo), &optimizerData));
	{
		// add the Adam optimizer kernel here
		ndInt32 sizeInFloats = ndInt32(weightsAndBiasBuffer.SizeInBytes() / sizeof(ndReal));
		ndBrainBufferCommandDesc descriptor(ndInt32(sizeInFloats) / ND_DEFAULT_WORKGROUP_SIZE);
		descriptor.m_context = this;
		descriptor.m_owner = nullptr;
		descriptor.m_id = m_adamOptimizerUpdate;
		descriptor.m_uniformBuffer = adamUniformbuffer;
		descriptor.m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
		
		descriptor.PushBack(*adamUniformbuffer);
		descriptor.PushBack(&weightsAndBiasBuffer);
		descriptor.PushBack(&weightsAndBiasGradientBuffer);
		descriptor.PushBack(*optimizer.m_vdw);
		descriptor.PushBack(*optimizer.m_vdw2);
		
		if (optimizer.GetRegularizerType() != m_lasso)
		{
			optimizer.m_commands.Append(ndSharedPtr<ndBrainBufferCommand>(new ndBrainAdamUpdateParametersRidge(descriptor)));
		}
		else
		{
			ndAssert(0);
			//m_adamOptimizerCommand = ndSharedPtr<ndBrainBufferCommand>(new ndBrainAdamUpdateParametersRidge(descriptor));
			//commands.PushBack(new ndBrainAdamUpdateParametersRidge(descriptor));
			optimizer.m_commands.Append(ndSharedPtr<ndBrainBufferCommand>(new ndBrainAdamUpdateParametersRidge(descriptor)));
		}
	}
	
	{
		// add Adam momentum bias update
		ndBrainBufferCommandDesc descriptor(1);
		descriptor.m_context = this;
		descriptor.m_owner = nullptr;
		descriptor.m_id = m_adamOptimizerMomentum;
		descriptor.m_uniformBuffer = adamUniformbuffer;
		descriptor.PushBack(*adamUniformbuffer);
		optimizer.m_commands.Append(ndSharedPtr<ndBrainBufferCommand>(new ndBrainAdamMomentumUpdate(descriptor)));
	}
}

void ndBrainCpuContext::ApplyLeanRateCommands(ndBrainBufferCommand* const command, ndBrainFloat learnRate)
{
	ndBrainAdamUpdateParametersRidge* const gradientUpdateCommand = (ndBrainAdamUpdateParametersRidge*)command;
	ndAssert(gradientUpdateCommand->GetDescriptor().m_id == m_adamOptimizerUpdate);
	gradientUpdateCommand->m_learnRate = learnRate;
	SubmitBufferCommand(command);
}