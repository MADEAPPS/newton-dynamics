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
#include "ndBrainSaveLoad.h"
#include "ndBrainSimdFloat8.h"
#include "ndBrainTrainerCpu.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainTrainerGpuInference.h"

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

ndBrainLayerFeedForwardCpuCommand* ndBrainLayerActivationTanh::GetLayerCpuFeedForwardCommand()
{
	ndBrainLayerFeedForwardCpuCommand* const command = new ndBrainLayerFeedForwardCpuCommand(this);
	return command;
}

ndBrainLayerBackPropagateCpuCommand* ndBrainLayerActivationTanh::GetLayerCpuBackPropagateCommand()
{
	ndBrainLayerBackPropagateCpuCommand* const command = new ndBrainLayerBackPropagateCpuCommand(this);
	return command;
}

void ndBrainLayerActivationTanh::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndCommandShareInfo* const info = &command->m_info;
	const ndBrainTrainerCpuInference* const trainer = command->m_owner;
	
	ndInt32 inputSize = info->m_inputSize;
	ndInt32 outputSize = info->m_outputSize;
	
	ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector input(&trainer->m_inputOutputBuffer[offset], inputSize);
	ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);
	
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
	const ndCommandShareInfo* const info = &command->m_info;
	const ndBrainTrainerCpu* const trainer = (ndBrainTrainerCpu*)command->m_owner;

	ndInt32 inputSize = info->m_inputSize;
	ndInt32 outputSize = info->m_outputSize;
	
	ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);
	
	ndInt32 dstOffset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector outputDerivative(&trainer->m_inputOuputGradientsBuffer[dstOffset + inputSize], outputSize);
	ndBrainMemVector inputDerivative(&trainer->m_inputOuputGradientsBuffer[dstOffset], inputSize);
	
	inputDerivative.Set(ndBrainFloat(1.0f));
	inputDerivative.MulSub(output, output);
	inputDerivative.Mul(outputDerivative);
	inputDerivative.FlushToZero();
}

ndBrainTrainerGpuCommand* ndBrainLayerActivationTanh::CreateGpuFeedForwardCommand(
	ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info,
	ndBrainGpuContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer,
	ndBrainGpuFloatBuffer* const inputOutputData,
	ndBrainGpuFloatBuffer* const weightsAndBias) const
{
	ndBrainTrainerGpuCommand* const command = new ndBrainTrainerGpuCommand(owner,
		info, size_t(this), context, context->m_brainLayerTanhActivation, miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias);
	return command;
}

ndBrainTrainerGpuCommand* ndBrainLayerActivationTanh::CreateGpuBackPropagateCommand(
	ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info,
	ndBrainGpuContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer,
	ndBrainGpuFloatBuffer* const inputOutputData,
	ndBrainGpuFloatBuffer* const weightsAndBias,
	ndBrainGpuFloatBuffer* const inputOutputGradients,
	ndBrainGpuFloatBuffer* const weightsAndBiasGradients) const
{
	ndBrainTrainerGpuCommand* const command = new ndBrainTrainerGpuCommand(
		owner, info, size_t(this), context, context->m_brainLayerTanhBackPropagate,
		miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias, inputOutputGradients, weightsAndBiasGradients);
	return command;
}

