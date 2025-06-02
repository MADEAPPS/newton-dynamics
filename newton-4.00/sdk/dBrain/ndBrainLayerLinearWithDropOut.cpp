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

#if 0
ndBrainLayerLinearWithDropOut::ndBrainLayerLinearWithDropOut(ndInt32 inputs, ndInt32 outputs, ndBrainFloat dropOutFactor)
	:ndBrainLayerLinear(inputs, outputs)
	,m_dropout()
	,m_dropoutFactor(dropOutFactor)
	,m_dropoutScale(ndBrainFloat(1.0f))
	,m_droutOutEnable(true)
{
	ndAssert(dropOutFactor >= ndBrainFloat(0.5f));
	ndAssert(dropOutFactor <= ndBrainFloat(1.0f));
	m_dropout.SetCount(outputs);
	UpdateDropOut();
}

ndBrainLayerLinearWithDropOut::ndBrainLayerLinearWithDropOut(const ndBrainLayerLinearWithDropOut& src)
	:ndBrainLayerLinear(src)
	,m_dropout(src.m_dropout)
	,m_dropoutFactor(src.m_dropoutFactor)
	,m_dropoutScale(src.m_dropoutScale)
	,m_droutOutEnable(src.m_droutOutEnable)
{
}

ndBrainLayerLinearWithDropOut::~ndBrainLayerLinearWithDropOut()
{
}


ndBrainLayer* ndBrainLayerLinearWithDropOut::Clone() const
{
	return new ndBrainLayerLinearWithDropOut(*this);
}


ndBrainLayer* ndBrainLayerLinearWithDropOut::Load(const ndBrainLoad* const)
{
	ndAssert(0);
	return nullptr;
}

void ndBrainLayerLinearWithDropOut::EnableDropOut(bool state)
{
	m_droutOutEnable = state;
}

void ndBrainLayerLinearWithDropOut::UpdateDropOut()
{
	ndInt32 activeCount = 0;
	for (ndInt32 i = ndInt32(m_dropout.GetCount()-1); i >= 0; --i)
	{
		ndInt32 active = (ndRand() <= m_dropoutFactor);
		m_dropout[i] = active ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		activeCount += active;
	}

	ndAssert(activeCount > 0);
	m_dropoutScale = ndBrainFloat (m_dropout.GetCount()) / ndBrainFloat (activeCount);
	m_dropout.Scale(m_dropoutScale);
}

void ndBrainLayerLinearWithDropOut::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(output.GetCount() == m_dropout.GetCount());
	ndBrainLayerLinear::MakePrediction(input, output);
	if (m_droutOutEnable)
	{
		output.Mul(m_dropout);
	}
}

//void ndBrainLayerLinearWithDropOut::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
void ndBrainLayerLinearWithDropOut::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
	//m_weights.TransposeMul(outputDerivative, inputDerivative);
}

void ndBrainLayerLinearWithDropOut::CalculateParamGradients(
	const ndBrainVector& input, const ndBrainVector& output,
	const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const
{
	if (m_droutOutEnable)
	{
		const ndBrainFloat* const outMemory = &outputDerivative[0];
		ndBrainMemVector outDerivative(outMemory, outputDerivative.GetCount());
		outDerivative.Mul(m_dropout);
	}
	ndBrainLayerLinear::CalculateParamGradients(input, output, outputDerivative, inputGradient, gradientOut);
}
#endif

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

ndBrainLayer::ndCommandShareInfo ndBrainLayerLinearWithDropOut::GetCommandSharedInfo() const
{
	ndCommandShareInfo info(this);
	info.m_inputSize = GetInputSize();
	info.m_outputSize = GetOutputSize();
	return info;
}

ndBrainLayerFeedFowardCpuCommand* ndBrainLayerLinearWithDropOut::GetLayerCpuFeedForwardCommand() const
{
	ndBrainLayerFeedFowardCpuCommand* const command = new ndBrainLayerFeedFowardCpuCommand(this);
	return command;
}

ndBrainLayerBackPropagateCpuCommand* ndBrainLayerLinearWithDropOut::GetLayerCpuBackPropagateCommand() const
{
	ndBrainLayerBackPropagateCpuCommand* const command = new ndBrainLayerBackPropagateCpuCommand(this);
	return command;
}

//ndBrainLayer::ndLayerUniformDataGpu ndBrainLayerLinearWithDropOut::GetLayerUniformDataGpu(const ndBrainGpuContext* const context) const
//{
//	ndLayerUniformDataGpu data;
//	data.m_shader = context->m_ndBrainLayerLinearDropOutActivation;
//	data.m_inputSize = GetInputSize();
//	data.m_outputSize = GetOutputSize();
//	return data;
//}

void ndBrainLayerLinearWithDropOut::FeedForward(const ndBrainLayerFeedFowardCpuCommand* const command, ndInt32 miniBatchIndex) const
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