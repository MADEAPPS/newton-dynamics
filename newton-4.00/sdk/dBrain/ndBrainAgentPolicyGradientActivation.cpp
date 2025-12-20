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
#include "ndBrainLayer.h"
#include "ndBrainKernel.h"
#include "ndBrainTrainer.h"
#include "ndBrainContext.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainCpuContext.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainBufferCommand.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainAgentPolicyGradientActivation.h"


ndBrainAgentPolicyGradientActivation::ndBrainAgentPolicyGradientActivation(ndInt32 neurons, ndBrainFloat minVariance, ndBrainFloat maxVariance)
	:ndBrainLayerActivation(neurons)
	,m_varianceBuffer(nullptr)
{
	ndAssert(minVariance > 0.0f);
	ndAssert(maxVariance > minVariance);

	m_sigmaBias = (maxVariance + minVariance) * ndBrainFloat(0.5f);
	m_sigmaSlope = (maxVariance - minVariance) * ndBrainFloat(0.5f);

	//ndBrainFloat xxx0 = m_sigmaBias + m_sigmaSlope * -1.0f;
	//ndBrainFloat xxx1 = m_sigmaBias + m_sigmaSlope * 1.0f;
	//ndBrainFloat xxx2 = m_sigmaBias + m_sigmaSlope * 1.0f;
}

ndBrainAgentPolicyGradientActivation::ndBrainAgentPolicyGradientActivation(const ndBrainAgentPolicyGradientActivation& src)
	:ndBrainLayerActivation(src)
	,m_sigmaBias(src.m_sigmaBias)
	,m_sigmaSlope(src.m_sigmaSlope)
	,m_varianceBuffer(nullptr)
{
}

bool ndBrainAgentPolicyGradientActivation::HasGpuSupport() const
{
	return true;
}

ndBrainLayer* ndBrainAgentPolicyGradientActivation::Clone() const
{
	return new ndBrainAgentPolicyGradientActivation(*this);
}

ndBrainLayer* ndBrainAgentPolicyGradientActivation::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();

	loadSave->ReadString(buffer);
	ndBrainFloat minSigma2 = ndBrainFloat(loadSave->ReadFloat());

	loadSave->ReadString(buffer);
	ndBrainFloat maxSigma2 = ndBrainFloat(loadSave->ReadFloat());

	ndBrainAgentPolicyGradientActivation* const layer = new ndBrainAgentPolicyGradientActivation(inputs, ndBrainFloat(ndSqrt(minSigma2)), ndBrainFloat(ndSqrt(maxSigma2)));
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainAgentPolicyGradientActivation::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerActivation::Save(loadSave);

	ndBrainFloat minSigma = m_sigmaBias - m_sigmaSlope;
	ndBrainFloat maxSigma = m_sigmaBias + m_sigmaSlope;

	char buffer[1024];
	snprintf(buffer, sizeof(buffer), "\tminSigma2 %f\n", minSigma * minSigma);
	loadSave->WriteData(buffer);

	snprintf(buffer, sizeof(buffer), "\tmaxSigma2 %f\n", maxSigma * maxSigma);
	loadSave->WriteData(buffer);
}

const char* ndBrainAgentPolicyGradientActivation::GetLabelId() const
{
	return ND_POLICY_GRADIENT_ACTIVATION_NAME;
}

void ndBrainAgentPolicyGradientActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndInt32 size = ndInt32(input.GetCount());
	for (ndInt32 i = 0; i < size; ++i)
	{
		ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		ndBrainFloat out0 = ndBrainFloat(ndTanh(value));
		ndBrainFloat out1 = m_sigmaBias + out0 * m_sigmaSlope;
		output[i] = (i < size / 2) ? out0 : out1;
	}
}

void ndBrainAgentPolicyGradientActivation::InputDerivative(const ndBrainVector&, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	//ndAssert(input.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	const ndInt32 size = ndInt32(output.GetCount());

	ndBrainFloat invSlope = ndBrainFloat(1.0) / m_sigmaSlope;
	for (ndInt32 i = 0; i < size; ++i)
	{
		ndBrainFloat out = output[i];
		ndBrainFloat meanGrad = ndBrainFloat(1.0f) - out * out;

		ndBrainFloat x1 = (out - m_sigmaBias) * invSlope;
		ndBrainFloat sigmaGrad = m_sigmaSlope * (ndBrainFloat(1.0f) - x1 * x1);

		ndBrainFloat blend = (i < size / 2) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		ndBrainFloat gradiend = meanGrad * blend + sigmaGrad * (ndBrainFloat(1.0f) - blend);

		inputDerivative[i] = gradiend * outputDerivative[i];
	}
}

void ndBrainAgentPolicyGradientActivation::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);

	const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
	ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);

	const ndInt32 size = ndInt32(input.GetCount());
	for (ndInt32 i = 0; i < size; ++i)
	{
		ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		ndBrainFloat out0 = ndBrainFloat(ndTanh(value));
		ndBrainFloat out1 = m_sigmaBias + m_sigmaSlope * out0;
		output[i] = (i < size / 2) ? out0 : out1;
	}
}

void ndBrainAgentPolicyGradientActivation::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);
	ndAssert(inputSize == info.m_outputSize);

	//const ndBrainMemVector input(&inputOutputBuffer[srcBase], inputSize);
	const ndBrainMemVector output(&inputOutputBuffer[dstBase], inputSize);
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], inputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);

	ndAssert(inputDerivative.GetCount() == output.GetCount());
	ndAssert(inputDerivative.GetCount() == outputDerivative.GetCount());

	const ndInt32 size = ndInt32(output.GetCount());
	ndBrainFloat invSlope = ndBrainFloat(1.0) / m_sigmaSlope;
	for (ndInt32 i = 0; i < size; ++i)
	{
		ndBrainFloat out = output[i];
		ndBrainFloat meanGrad = ndBrainFloat(1.0f) - out * out;

		ndBrainFloat x1 = (out - m_sigmaBias) * invSlope;
		ndBrainFloat sigmaGrad = m_sigmaSlope * (ndBrainFloat(1.0f) - x1 * x1);

		ndBrainFloat blend = (i < size / 2) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		ndBrainFloat gradiend = meanGrad * blend + sigmaGrad * (ndBrainFloat(1.0f) - blend);

		inputDerivative[i] = gradiend * outputDerivative[i];
	}
}

ndCommandArray ndBrainAgentPolicyGradientActivation::CreateGpuFeedForwardCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
		owner, context, info, miniBatchSize, 0,
		inputOutputData, weightsAndBias));

	ndBrainBufferCommand* command = nullptr;
	if (context->GetAsCpuContext())
	{
		command = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
	}
	else
	{
		struct VarianceParams
		{
			ndReal m_bias;
			ndReal m_slope;
		};
		VarianceParams variance;
		variance.m_bias = m_sigmaBias;
		variance.m_slope = m_sigmaSlope;
		m_varianceBuffer = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(context, sizeof(VarianceParams), &variance));
		descriptor.PushBack(*m_varianceBuffer);

		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerPolicyGradientActivation;
		command = new ndBrainGpuCommand(descriptor);
	}
	ndCommandArray commandArray(0);
	commandArray.PushBack(command);
	return commandArray;
}

ndCommandArray ndBrainAgentPolicyGradientActivation::CreateGpuBackPropagateCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias,
	ndBrainFloatBuffer* const inputOutputGradients,
	ndBrainFloatBuffer* const weightsAndBiasGradients) const
{
	ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
		owner, context, info, miniBatchSize, 0,
		inputOutputData, weightsAndBias,
		inputOutputGradients, weightsAndBiasGradients));

	ndCommandArray commands(0);

	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
		commands.PushBack(command);
	}
	else
	{
		descriptor.PushBack(*m_varianceBuffer);
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerPolicyGradientBackPropagate;
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		commands.PushBack(command);
	}
	return commands;
}
