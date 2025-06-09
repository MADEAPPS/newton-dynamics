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
#include "ndBrainTrainerCpu.h"
#include "ndBrainLayerLinearWithDropOut.h"
#include "ndBrainTrainerGpuInference.h"

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
	//inputDerivative.Set(1.0f);
	inputDerivative.Set(m_dropOut);
	inputDerivative.Mul(outputDerivative);
}

bool ndBrainLayerLinearWithDropOut::HasGpuSupport() const
{
	return true;
}

ndBrainLayer::ndCommandShareInfo ndBrainLayerLinearWithDropOut::GetCommandSharedInfo()
{
	ndCommandShareInfo info(this);
	info.m_inputSize = GetInputSize();
	info.m_outputSize = GetOutputSize();
	return info;
}

ndBrainLayerFeedForwardCpuCommand* ndBrainLayerLinearWithDropOut::GetLayerCpuFeedForwardCommand()
{
	ndBrainLayerFeedForwardCpuCommand* const command = new ndBrainLayerFeedForwardCpuCommand(this);
	return command;
}

ndBrainLayerBackPropagateCpuCommand* ndBrainLayerLinearWithDropOut::GetLayerCpuBackPropagateCommand()
{
	ndBrainLayerBackPropagateCpuCommand* const command = new ndBrainLayerBackPropagateCpuCommand(this);
	return command;
}

void ndBrainLayerLinearWithDropOut::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndCommandShareInfo* const info = &command->m_info;
	const ndBrainTrainerCpuInference* const trainer = command->m_owner;
	
	ndInt32 inputSize = info->m_inputSize;
	ndInt32 outputSize = info->m_outputSize;
	
	ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector input(&trainer->m_inputOutputBuffer[offset], inputSize);
	ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);
	
	output.Set(input);
	output.Mul(m_dropOut);
}

void ndBrainLayerLinearWithDropOut::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
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
	
	inputDerivative.Set(m_dropOut);
	inputDerivative.Mul(outputDerivative);
}

ndBrainTrainerGpuCommand* ndBrainLayerLinearWithDropOut::CreateGpuFeedForwardCommand(
	ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info,
	ndBrainGpuContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
	ndBrainGpuBuffer* const inputOutputData,
	ndBrainGpuBuffer* const weightsAndBias) const
{
	ndBrainTrainerGpuCommand* const command = new ndBrainTrainerGpuCommand(owner,
		info, size_t(this), context, context->m_ndBrainLayerLinearDropOutActivation, miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias);
	return command;
}

ndBrainTrainerGpuCommand* ndBrainLayerLinearWithDropOut::CreateGpuBackPropagateCommand(
	ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info,
	ndBrainGpuContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
	ndBrainGpuBuffer* const inputOutputData,
	ndBrainGpuBuffer* const weightsAndBias,
	ndBrainGpuBuffer* const inputOutputGradients,
	ndBrainGpuBuffer* const weightsAndBiasGradients) const
{
	ndBrainTrainerGpuCommand* const command = new ndBrainTrainerGpuCommand(
		owner, info, size_t(this), context, context->m_ndBrainLayerLinearDropOutBackPropagate,
		miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias, inputOutputGradients, weightsAndBiasGradients);
	return command;
}

