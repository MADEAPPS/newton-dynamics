/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainTrainer.h"
#include "ndBrainContext.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainSimdFloat8.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainLayerActivationTanh.h"

ndBrainLayerActivationTanh::ndBrainLayerActivationTanh(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationTanh::ndBrainLayerActivationTanh(const ndBrainLayerActivationTanh& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationTanh::Clone() const
{
	return new ndBrainLayerActivationTanh(*this);
}

const char* ndBrainLayerActivationTanh::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_TANGH_NAME;
}

ndBrainLayer* ndBrainLayerActivationTanh::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationTanh* const layer = new ndBrainLayerActivationTanh(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationTanh::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());

	const ndBrainSimdFloat8 max(30.0f);
	const ndBrainSimdFloat8 min(-30.0f);
	ndBrainFloat* const dst = &output[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 value(x.Clamp(min, max));
		value.Tanh().Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	{
		ndBrainFloat value = ndClamp(src[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		dst[i] = ndBrainFloat(ndTanh(value));
	}
	output.FlushToZero();
}

void ndBrainLayerActivationTanh::InputDerivative(const ndBrainVector&, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	inputDerivative.Set(ndBrainFloat(1.0f));
	inputDerivative.MulSub(output, output);
	inputDerivative.Mul(outputDerivative);
	inputDerivative.FlushToZero();
}

bool ndBrainLayerActivationTanh::HasGpuSupport() const
{
	return true;
}

void ndBrainLayerActivationTanh::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	
	ndInt32 offset = miniBatchIndex * info.m_inputOutputSize + info.m_inputOutputStartOffset;
	ndAssert(offset >= 0);

	const ndBrainMemVector input(&inputOutputBuffer[offset], inputSize);
	ndBrainMemVector output(&inputOutputBuffer[offset + inputSize], outputSize);
	
	const ndBrainSimdFloat8 max(30.0f);
	const ndBrainSimdFloat8 min(-30.0f);
	ndBrainFloat* const dst = &output[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 value(x.Clamp(min, max));
		value.Tanh().Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	{
		ndBrainFloat value = ndClamp(src[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		dst[i] = ndBrainFloat(ndTanh(value));
	}
	output.FlushToZero();
}

void ndBrainLayerActivationTanh::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	
	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	
	ndInt32 offset = miniBatchIndex * info.m_inputOutputSize + info.m_inputOutputStartOffset;
	ndAssert(offset >= 0);
	const ndBrainMemVector output(&inputOutputBuffer[offset + inputSize], outputSize);
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[offset + inputSize], outputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[offset], inputSize);
	
	inputDerivative.Set(ndBrainFloat(1.0f));
	inputDerivative.MulSub(output, output);
	inputDerivative.Mul(outputDerivative);
	inputDerivative.FlushToZero();
}

ndBrainBufferCommand* ndBrainLayerActivationTanh::CreateGpuFeedForwardCommand(
	ndBrainTrainerInference* const owner,
	const ndCommandSharedInfo& info,
	ndBrainContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	ndBrainBufferCommandDesc descriptor(miniBatchSize);
	descriptor.m_id = size_t(this);
	descriptor.m_context = context;
	descriptor.m_owner = owner;
	descriptor.m_info = info;
	descriptor.m_uniformBuffer = uniformBuffer;
	descriptor.PushBack((ndBrainUniformBuffer*)*uniformBuffer);
	descriptor.PushBack(inputOutputData);
	descriptor.PushBack(weightsAndBias);

	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommand* const command = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
		return command;
	}
	else
	{
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerTanhActivation;
		//ndBrainBufferCommand* const command = new ndBrainTrainerGpuCommand(owner,
		//	info, size_t(this), context, context->m_brainLayerTanhActivation, miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias);
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		return command;
	}
}

ndBrainBufferCommand* ndBrainLayerActivationTanh::CreateGpuBackPropagateCommand(
	ndBrainTrainerInference* const owner,
	const ndCommandSharedInfo& info,
	ndBrainContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias,
	ndBrainFloatBuffer* const inputOutputGradients,
	ndBrainFloatBuffer* const weightsAndBiasGradients) const
{
	ndBrainBufferCommandDesc descriptor(miniBatchSize);
	descriptor.m_id = size_t(this);
	descriptor.m_context = context;
	descriptor.m_owner = owner;
	descriptor.m_info = info;
	descriptor.m_uniformBuffer = uniformBuffer;

	descriptor.PushBack((ndBrainUniformBuffer*)*uniformBuffer);
	descriptor.PushBack(inputOutputData);
	descriptor.PushBack(weightsAndBias);
	descriptor.PushBack(inputOutputGradients);
	descriptor.PushBack(weightsAndBiasGradients);

	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
		return command;
	}
	else
	{
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerTanhBackPropagate;
		//ndBrainBufferCommand* const command = new ndBrainTrainerGpuCommand(
		//	owner, info, size_t(this), context, context->m_brainLayerTanhBackPropagate,
		//	miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias, inputOutputGradients, weightsAndBiasGradients);
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		return command;
	}
}
