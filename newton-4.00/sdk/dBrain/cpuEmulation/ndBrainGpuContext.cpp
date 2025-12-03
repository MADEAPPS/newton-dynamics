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
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainBufferCommand.h"
#include "ndBrainTrainerInference.h"

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
	,ndBrainThreadPool()
{
	ndInt32 numOfThreads = ndBrainThreadPool::GetMaxThreads();
#ifdef _DEBUG
numOfThreads = 1;
#endif

	SetThreadCount(numOfThreads);
	CreateKerners();
	CreateCopyCommands();
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

ndBrainGpuContext* ndBrainGpuContext::GetAsGpuContext()
{
	return this;
}

void ndBrainGpuContext::CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	ndSharedPtr<ndBrainGpuBuffer>& dstBuffer = dstData.m_gpuBuffer;
	const ndSharedPtr<ndBrainGpuBuffer>& srcBuffer = srcData.m_gpuBuffer;
	ndAssert(dstData.SizeInBytes() == srcData.SizeInBytes());
	ndInt32* const dst = (ndInt32*)&dstBuffer->m_memory[0];
	ndInt32* const src = (ndInt32*)&srcBuffer->m_memory[0];
	ndAssert(src);
	ndAssert(dst);
	ndAssert((dstData.SizeInBytes() & (sizeof (ndInt32) - 1)) == 0);
	ndMemCpy(dst, src, ndInt64(dstData.SizeInBytes() / sizeof (ndInt32)));
}

void ndBrainGpuContext::CopyBuffer(const ndCopyBufferCommandInfo& parameters, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	m_copyBufferParams->MemoryToDevice(0, sizeof(ndCopyBufferCommandInfo), &parameters);

	ndBrainBufferCommandDesc& descriptor = m_copyBufferCommand->GetDescriptor();
	descriptor.SetCount(0);
	descriptor.PushBack(*m_copyBufferParams);
	descriptor.PushBack((ndBrainBuffer*)&dstData);
	descriptor.PushBack((ndBrainBuffer*)&srcData);

	descriptor.m_miniBatchSize = numberOfWorkGrups;
	descriptor.m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
	SubmitBufferCommand(*m_copyBufferCommand);
}

void ndBrainGpuContext::CopyBufferIndirect(const ndCopyBufferCommandInfo& parameters, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	m_copyBufferParams->MemoryToDevice(0, sizeof(ndCopyBufferCommandInfo), &parameters);

	ndBrainBufferCommandDesc& descriptor = m_copyBufferIndirectCommand->GetDescriptor();
	descriptor.SetCount(0);
	descriptor.PushBack(*m_copyBufferParams);
	descriptor.PushBack((ndBrainBuffer*)&dstData);
	descriptor.PushBack((ndBrainBuffer*)&srcData);
	descriptor.PushBack((ndBrainBuffer*)&indexBuffer);

	descriptor.m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
	descriptor.m_miniBatchSize = ndInt32(indexBuffer.SizeInBytes() / sizeof(ndUnsigned32));
	SubmitBufferCommand(*m_copyBufferIndirectCommand);
}

void ndBrainGpuContext::CreateCopyCommands()
{
	ndBrainBufferCommandDesc copyDescriptor(0);
	copyDescriptor.m_context = this;
	copyDescriptor.m_kernel = m_brainCopyBuffer;
	m_copyBufferCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyDescriptor));

	ndBrainBufferCommandDesc copyIndirectDescriptor(0);
	copyIndirectDescriptor.m_context = this;
	copyIndirectDescriptor.m_kernel = m_brainCopyBufferIndirect;
	m_copyBufferIndirectCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyIndirectDescriptor));

	ndCopyBufferCommandInfo copyBuffer;
	m_copyBufferParams = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(this, sizeof(ndCopyBufferCommandInfo), &copyBuffer));
}

void ndBrainGpuContext::MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer = deviceBuffer.m_gpuBuffer;
	ndInt8* const src = &gpuBuffer->m_memory[0];
	ndAssert(src);
	ndMemCpy((ndInt8*)outputMemory, &src[offsetInBytes], ndInt64(sizeInBytes));
}

void ndBrainGpuContext::MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory) const
{
	ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer = deviceBuffer.m_gpuBuffer;
	ndInt8* const dst = &gpuBuffer->m_memory[0];
	ndAssert(dst);
	ndMemCpy(&dst[offsetInBytes], (ndInt8*)inputMemory, ndInt64(sizeInBytes));
}

void ndBrainGpuContext::SyncBufferCommandQueue()
{
	// do nothing. cpu kernels always wait for completion.
}

void ndBrainGpuContext::SubmitBufferCommand(ndBrainBufferCommand* const command)
{
	ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	ndBrainKernel& shader = **desc.m_kernel;

	shader.m_parameters.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32 (desc.GetCount()); ++i)
	{
		shader.m_parameters.PushBack(desc[i]);
	}

	auto ExecuteCommand = ndMakeObject::ndFunction([this, &command](ndInt32 groupId, ndInt32)
	{
		ndBrainBufferCommandDesc& desc = command->GetDescriptor();
		ndBrainKernel& shader = **desc.m_kernel;
		ndInt32 workGroupdSize = ndInt32(desc.m_workGroupSize);
		shader.Execute(groupId, workGroupdSize);
	});
	ParallelExecute(ExecuteCommand, desc.m_miniBatchSize);
}

void ndBrainGpuContext::BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector)
{
	size_t sizeInBytes = ndMin(size_t(dst.SizeInBytes()), size_t(srcVector.GetCount() * sizeof(ndReal)));
	MemoryToDevice(dst, 0, sizeInBytes, &srcVector[0]);
}

void ndBrainGpuContext::BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector)
{
	size_t sizeInBytes = ndMin(size_t(src.SizeInBytes()), size_t(dstVector.GetCount() * sizeof(ndReal)));
	MemoryFromDevice(src, 0, sizeInBytes, &dstVector[0]);
}

void ndBrainGpuContext::Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.Scale(scale);
}

void ndBrainGpuContext::Set(ndBrainFloatBuffer& buffer, ndBrainFloat value)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.Set(value);
}

void ndBrainGpuContext::Set(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.Set(src);
}

void ndBrainGpuContext::Exp(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.Exp(src);
}

void ndBrainGpuContext::Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);

	dst.Min(src);
}

void ndBrainGpuContext::Max(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);

	dst.Max(src);
}

void ndBrainGpuContext::Min(ndBrainFloatBuffer& buffer, ndBrainFloat value)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.Min(value);
}

void ndBrainGpuContext::Less(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.Less(test);
}

void ndBrainGpuContext::Greater(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.Greater(test);
}

void ndBrainGpuContext::Max(ndBrainFloatBuffer& buffer, ndBrainFloat value)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.Max(value);
}

void ndBrainGpuContext::Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.Add(src);
}

void ndBrainGpuContext::Sub(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.Sub(src);
}

void ndBrainGpuContext::Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst ((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.Mul(src);
}

void ndBrainGpuContext::ScaleAdd(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat scale)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.ScaleAdd(src, scale);
}

void ndBrainGpuContext::StandardNormalDistribution(ndBrainFloatBuffer& uniformRandomVariable)
{
	ndInt32 elements = ndInt32(uniformRandomVariable.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)uniformRandomVariable.GetCpuPtr(), elements);
	dst.StandardNormalDistribution();
}

void ndBrainGpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat blend)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.Blend(src, blend);
}

void ndBrainGpuContext::Reciprocal(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.Reciprocal(src);
}

void ndBrainGpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, const ndBrainFloatBuffer& blendBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	const ndBrainMemVector blend((ndBrainFloat*)blendBuffer.GetCpuPtr(), elements);
	dst.Blend(src, blend);
}

void ndBrainGpuContext::LessEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.LessEqual(src);
}

void ndBrainGpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(buffer.SizeInBytes() == srcBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector src((ndBrainFloat*)srcBuffer.GetCpuPtr(), elements);
	dst.GreaterEqual(src);
}

void ndBrainGpuContext::LessEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.LessEqual(test);
}

void ndBrainGpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	dst.GreaterEqual(test);
}

void ndBrainGpuContext::Select(ndBrainFloatBuffer& buffer, ndBrainFloatBuffer& maskBuffer, ndBrainFloat a, ndBrainFloat b)
{
	ndAssert(buffer.SizeInBytes() == maskBuffer.SizeInBytes());
	ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector mask((ndBrainFloat*)maskBuffer.GetCpuPtr(), elements);
	dst.Blend(mask, a, b);
}

void ndBrainGpuContext::BroadcastScaler(ndBrainFloatBuffer& buffer, ndInt32 bufferStrideInFloats, const ndBrainFloatBuffer& srcScalar)
{
	ndAssert(buffer.SizeInBytes() == srcScalar.SizeInBytes() * bufferStrideInFloats);
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat)));
	const ndBrainMemVector src((ndBrainFloat*)srcScalar.GetCpuPtr(), ndInt32(srcScalar.SizeInBytes() / sizeof(ndBrainFloat)));

	for (ndInt32 i = 0; i < src.GetCount(); ++i)
	{
		ndBrainMemVector subVector(&dst[i * bufferStrideInFloats], bufferStrideInFloats);
		subVector.Set(src[i]);
	}
}

void ndBrainGpuContext::SetOrdinal(ndBrainFloatBuffer& dstData)
{
	ndAssert(0);
	//ndBrainVector& dst = **dstData.m_buffer;
	//dst.SetOrdinal();
}

void ndBrainGpuContext::ReductionSum(ndBrainFloatBuffer& dstData)
{
	ndAssert(0);
	//ndBrainVector& dst = **dstData.m_buffer;
	//dst.ReductionSum();
}

void ndBrainGpuContext::Sqrt(ndBrainFloatBuffer&, ndInt32)
{
	ndAssert(0);
}

void ndBrainGpuContext::InvSqrt(ndBrainFloatBuffer&, ndInt32 clipSize)
{
	ndAssert(0);
}

void ndBrainGpuContext::ReductionSum(ndBrainFloatBuffer& buffer, ndInt32 clipSize)
{
	ndAssert(0);
}

void ndBrainGpuContext::CalculateLikelihood(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer)
{
	ndAssert(0);
}

void ndBrainGpuContext::CalculateEntropyRegularization(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer, ndBrainFloat regularization)
{ 
	const ndInt32 stride = ndInt32(sigmaBuffer.SizeInBytes() / buffer.SizeInBytes());
	ndAssert(sampleBuffer.SizeInBytes() == sigmaBuffer.SizeInBytes());
	ndAssert(stride * buffer.SizeInBytes() == sampleBuffer.SizeInBytes());

	const ndInt32 elements = ndInt32(buffer.SizeInBytes() / sizeof(ndBrainFloat));

	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), elements);
	const ndBrainMemVector sample((ndBrainFloat*)sampleBuffer.GetCpuPtr(), stride * elements);
	const ndBrainMemVector sigmas((ndBrainFloat*)sigmaBuffer.GetCpuPtr(), stride * elements);
	for (ndInt32 i = 0; i < elements; ++i)
	{
		const ndBrainMemVector sampleMean(&sample[i * stride], stride);
		const ndBrainMemVector varianceMean(&sigmas[i * stride], stride);
		dst[i] = sampleMean.CalculateEntropyRegularization(varianceMean, regularization);
	}
}

void ndBrainGpuContext::CalculateEntropyRegularizationGradient(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& sampleBuffer, const ndBrainFloatBuffer& sigmaBuffer, ndBrainFloat regularization, ndInt32 inputSize)
{
	ndAssert(sampleBuffer.SizeInBytes() == sigmaBuffer.SizeInBytes());
	ndAssert(2 * sampleBuffer.SizeInBytes() == buffer.SizeInBytes());
	const ndInt32 numberOfGroups = ndInt32(sampleBuffer.SizeInBytes() / sizeof(ndBrainFloat)) / inputSize;
	
	ndBrainMemVector dst((ndBrainFloat*)buffer.GetCpuPtr(), ndInt64(buffer.SizeInBytes() / sizeof(ndReal)));
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

void ndBrainGpuContext::SetLearnRateCommandBuffers(ndBrainOptimizerAdam& optimizer, ndInt32 minibatchSize, ndBrainFloatBuffer& weightsAndBiasBuffer, ndBrainFloatBuffer& weightsAndBiasGradientBuffer)
{
	ndBrainOptimizerAdam::ndCommandSharedInfo optimizerData(optimizer.m_parameters);
	ndSharedPtr<ndBrainUniformBuffer> adamUniformbuffer(new ndBrainUniformBuffer(this, sizeof(ndBrainOptimizerAdam::ndCommandSharedInfo), &optimizerData));
	{
		// add the Adam optimizer kernel here
		optimizerData.m_minibathScale = ndBrainFloat(1.0f) / ndBrainFloat(minibatchSize);
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
			descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainAdamRidgeOptimizerUpdate;
		}
		else
		{
			descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainAdamLassoOptimizerUpdate;
		}
		optimizer.m_commands.Append(ndSharedPtr<ndBrainBufferCommand>(new ndBrainGpuCommand(descriptor)));
	}
	
	{
		// add Adam momentum update
		ndBrainBufferCommandDesc descriptor(1);
		descriptor.m_context = this;
		descriptor.m_owner = nullptr;
		descriptor.m_id = m_adamOptimizerMomentum;
		descriptor.m_uniformBuffer = adamUniformbuffer;
		descriptor.PushBack(*adamUniformbuffer);
	
		descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainAdamMomentumUpdate;
		optimizer.m_commands.Append(ndSharedPtr<ndBrainBufferCommand>(new ndBrainGpuCommand(descriptor)));
	}
}

void ndBrainGpuContext::ApplyLeanRateCommands(ndBrainBufferCommand* const command, ndBrainFloat learnRate)
{
	ndBrainFloat learnRatePtr = learnRate;
	ndBrainBuffer* const buffer = (ndBrainBuffer*)&learnRatePtr;
	ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	desc.PushBack(buffer);
	SubmitBufferCommand(command);
	desc.Pop();
}