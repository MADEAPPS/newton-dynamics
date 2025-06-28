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
#include "ndBrainTrainer.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainSimdFloat8.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerActivationRelu.h"

ndBrainLayerActivationRelu::ndBrainLayerActivationRelu(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationRelu::ndBrainLayerActivationRelu(const ndBrainLayerActivationRelu& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationRelu::Clone() const
{
	return new ndBrainLayerActivationRelu(*this);
}

const char* ndBrainLayerActivationRelu::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_RELU_NAME;
}

ndBrainLayer* ndBrainLayerActivationRelu::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationRelu* const layer = new ndBrainLayerActivationRelu(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationRelu::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	const ndBrainSimdFloat8 zero(0.0f);
	ndBrainFloat* const dst = &output[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 value(x.Max(zero));
		value.Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	{
		output[i] = ndMax (input[i], ndBrainFloat (0.0f));
	}
}

void ndBrainLayerActivationRelu::InputDerivative(const ndBrainVector& input, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(input.GetCount() == outputDerivative.GetCount());
	ndAssert(input.GetCount() == inputDerivative.GetCount());

	const ndBrainSimdFloat8 one(1.0f);
	const ndBrainSimdFloat8 zero(0.0f);
	ndBrainFloat* const dst = &inputDerivative[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 test(x >= zero);
		const ndBrainSimdFloat8 value(test & one);
		value.Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	{
		inputDerivative[i] = (input[i] >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
	}

	inputDerivative.Mul(outputDerivative);
}

bool ndBrainLayerActivationRelu::HasGpuSupport() const
{
	return true;
}

ndBrainLayerFeedForwardCpuCommand* ndBrainLayerActivationRelu::GetLayerCpuFeedForwardCommand()
{
	ndAssert(0);
	return nullptr;

	//ndBrainLayerFeedForwardCpuCommand* const command = new ndBrainLayerFeedForwardCpuCommand(this);
	//return command;
}

ndBrainLayerBackPropagateCpuCommand* ndBrainLayerActivationRelu::GetLayerCpuBackPropagateCommand()
{
	ndAssert(0);
	return nullptr;

	//ndBrainLayerBackPropagateCpuCommand* const command = new ndBrainLayerBackPropagateCpuCommand(this);
	//return command;
}

void ndBrainLayerActivationRelu::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	ndAssert(0);

	//const ndCommandShareInfo* const info = &command->m_info;
	//const ndBrainTrainerCpuInference* const trainer = command->m_owner;
	//
	//ndInt32 inputSize = info->m_inputSize;
	//ndInt32 outputSize = info->m_outputSize;
	//
	//ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	//const ndBrainMemVector input(&trainer->m_inputOutputBuffer[offset], inputSize);
	//ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);
	//
	//const ndBrainSimdFloat8 zero(0.0f);
	//ndBrainFloat* const dst = &output[0];
	//const ndBrainFloat* const src = &input[0];
	//const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	//for (ndInt32 i = 0; i < roundCount; i += 8)
	//{
	//	const ndBrainSimdFloat8 x(&src[i]);
	//	const ndBrainSimdFloat8 value(x.Max(zero));
	//	value.Store(&dst[i]);
	//}
	//for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	//{
	//	output[i] = ndMax(input[i], ndBrainFloat(0.0f));
	//}
}

void ndBrainLayerActivationRelu::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	ndAssert(0);

	//const ndCommandShareInfo* const info = &command->m_info;
	//const ndBrainTrainerCpu* const trainer = (ndBrainTrainerCpu*)command->m_owner;
	//
	//ndInt32 inputSize = info->m_inputSize;
	//ndInt32 outputSize = info->m_outputSize;
	//
	//ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	//const ndBrainMemVector input(&trainer->m_inputOutputBuffer[offset], inputSize);
	//const ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);
	//
	//ndInt32 dstOffset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	//const ndBrainMemVector outputDerivative(&trainer->m_inputOutputGradientsBuffer[dstOffset + inputSize], outputSize);
	//ndBrainMemVector inputDerivative(&trainer->m_inputOutputGradientsBuffer[dstOffset], inputSize);
	//
	//const ndBrainSimdFloat8 one(1.0f);
	//const ndBrainSimdFloat8 zero(0.0f);
	//ndBrainFloat* const dst = &inputDerivative[0];
	//const ndBrainFloat* const src = &input[0];
	//const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	//for (ndInt32 i = 0; i < roundCount; i += 8)
	//{
	//	const ndBrainSimdFloat8 x(&src[i]);
	//	const ndBrainSimdFloat8 test(x >= zero);
	//	const ndBrainSimdFloat8 value(test & one);
	//	value.Store(&dst[i]);
	//}
	//for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	//{
	//	inputDerivative[i] = (input[i] >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
	//}
	//inputDerivative.Mul(outputDerivative);
}

ndBrainTrainerGpuCommand* ndBrainLayerActivationRelu::CreateGpuFeedForwardCommand(
	ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info,
	ndBrainGpuContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	ndAssert(0);
	return nullptr;

	//ndBrainTrainerGpuCommand* const command = new ndBrainTrainerGpuCommand(owner,
	//	info, size_t(this), context, context->m_brainLayerReluActivation, miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias);
	//return command;
}

ndBrainTrainerGpuCommand* ndBrainLayerActivationRelu::CreateGpuBackPropagateCommand(
	ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info,
	ndBrainGpuContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias,
	ndBrainFloatBuffer* const inputOutputGradients,
	ndBrainFloatBuffer* const weightsAndBiasGradients) const
{
	ndAssert(0);
	return nullptr;

	//ndBrainTrainerGpuCommand* const command = new ndBrainTrainerGpuCommand(
	//	owner, info, size_t(this), context, context->m_brainLayerReluBackPropagate,
	//	miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias, inputOutputGradients, weightsAndBiasGradients);
	//return command;
}
