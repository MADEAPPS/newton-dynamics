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
#include "ndBrainFloatBuffer.h"
#include "ndBrainLayerLinearWithDropOut.h"

ndBrainLayerLinearWithDropOut::ndBrainLayerLinearWithDropOut(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
	,m_dropOut()
{
	m_dropOut.SetCount(neurons);
	m_dropOut.Set(ndFloat32(1.0f));
}

ndBrainLayerLinearWithDropOut::ndBrainLayerLinearWithDropOut(const ndBrainLayerLinearWithDropOut& src)
	:ndBrainLayerActivation(src)
	,m_dropOut()
{
	m_dropOut.SetCount(m_neurons);
	m_dropOut.Set(ndFloat32(1.0f));
}

ndBrainLayer* ndBrainLayerLinearWithDropOut::Clone() const
{
	return new ndBrainLayerLinearWithDropOut(*this);
}

const char* ndBrainLayerLinearWithDropOut::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_LINEAR_DROPOUT;
}

ndBrainLayer* ndBrainLayerLinearWithDropOut::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerLinearWithDropOut* const layer = new ndBrainLayerLinearWithDropOut(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerLinearWithDropOut::ApplyDropOut(ndFloat32 rate)
{
	ndInt32 activeCount = 0;
	for (ndInt32 i = ndInt32(m_dropOut.GetCount() - 1); i >= 0; --i)
	{
		ndInt32 active = (ndRand() > rate);
		m_dropOut[i] = active ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		activeCount += active;
	}
	
	ndAssert(activeCount > 0);
	ndFloat32 dropoutScale = ndBrainFloat(m_dropOut.GetCount()) / ndBrainFloat(activeCount);
	m_dropOut.Scale(dropoutScale);
}

void ndBrainLayerLinearWithDropOut::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	output.Set(input);
	output.Mul(m_dropOut);
}

void ndBrainLayerLinearWithDropOut::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	inputDerivative.Set(m_dropOut);
	inputDerivative.Mul(outputDerivative);
}

bool ndBrainLayerLinearWithDropOut::HasGpuSupport() const
{
	return true;
}

void ndBrainLayerLinearWithDropOut::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
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
	
	output.Set(input);
	output.Mul(m_dropOut);
}

void ndBrainLayerLinearWithDropOut::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	//const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);
	ndAssert(inputSize == info.m_outputSize);

	//const ndBrainMemVector output(&inputOutputBuffer[dstBase], inputSize);
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], inputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);
	
	inputDerivative.Set(m_dropOut);
	inputDerivative.Mul(outputDerivative);
}

ndCommandArray ndBrainLayerLinearWithDropOut::CreateGpuFeedForwardCommand(
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
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerDropOutActivation;
		command = new ndBrainGpuCommand(descriptor);
	}
	ndCommandArray commandArray(0);
	commandArray.PushBack(command);
	return commandArray;
}

ndCommandArray ndBrainLayerLinearWithDropOut::CreateGpuBackPropagateCommand(
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

	ndCommandArray comnands(0);

	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
		comnands.PushBack(command);
	}
	else
	{
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerDropOutBackPropagate;
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		comnands.PushBack(command);
	}
	return comnands;
}
