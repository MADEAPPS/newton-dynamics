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
#include "gpu/ndBrainGpuContext.h"
#include "ndBrainTrainerCpuInference.h"
#include "ndBrainLayerActivationSoftmax.h"

ndBrainLayerActivationSoftmax::ndBrainLayerActivationSoftmax(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationSoftmax::ndBrainLayerActivationSoftmax(const ndBrainLayerActivationSoftmax& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationSoftmax::Clone() const
{
	return new ndBrainLayerActivationSoftmax(*this);
}

const char* ndBrainLayerActivationSoftmax::GetLabelId() const
{
	return "ndBrainLayerActivationSoftmax";
}

ndBrainLayer* ndBrainLayerActivationSoftmax::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationSoftmax* const layer = new ndBrainLayerActivationSoftmax(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationSoftmax::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	ndBrainFloat max = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	{
		max = ndMax(input[i], max);
	}

	ndBrainFloat acc = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat in = ndMax((input[i] - max), ndBrainFloat(-30.0f));
		ndAssert(in <= ndBrainFloat(0.0f));
		ndBrainFloat prob = ndBrainFloat(ndExp(in));
		output[i] = prob;
		acc += prob;
	}

	ndAssert(acc > ndBrainFloat(0.0f));
	output.Scale(ndBrainFloat(1.0f) / acc);
	output.FlushToZero();
}

void ndBrainLayerActivationSoftmax::InputDerivative(const ndBrainVector&, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	// calculate the output derivative which is a the Jacobian matrix time the input
	//for (ndInt32 i = 0; i < output.GetCount(); ++i)
	//{
	//	ndFloat32 s = output[i];
	//	ndFloat32 acc = (s * (ndFloat32(1.0f) - s)) * outputDerivative[i];
	//	for (ndInt32 j = 0; j < output.GetCount(); ++j)
	//	{
	//		if (i != j)
	//		{
	//			acc -= s * output[j] * outputDerivative[j];
	//		}
	//	}
	//	inputDerivative[i] = ndBrainFloat(acc);
	//}

	// better way to calculate the output derivative which is a the Jacobian matrix time the input
	// y = (O * I - O * transp(O)) * InputDerivative

	ndAssert(0);
	ndBrainFloat* const tempBuffer = ndAlloca(ndBrainFloat, output.GetCount());
	ndBrainMemVector tmp(tempBuffer, output.GetCount());
	for (ndInt32 i = 0; i < output.GetCount(); ++i)
	{
		tmp.ScaleSet(output, output[i]);
		tmp[i] = output[i] - tmp[i];
		inputDerivative[i] = outputDerivative.Dot(tmp);
	}
	inputDerivative.FlushToZero();
}

bool ndBrainLayerActivationSoftmax::HasGpuSupport() const
{
	return true;
}

ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* ndBrainLayerActivationSoftmax::GetLayerCpuFeedForwardCommand() const
{
	ndBrainLayerFeedFowardCpuCommand* const command = new ndBrainLayerFeedFowardCpuCommand(this);
	command->m_inputSize = GetInputSize();
	command->m_outputSize = GetOutputSize();
	return command;
}

ndBrainLayer::ndBrainLayerBackPropagateCpuCommand* ndBrainLayerActivationSoftmax::GetLayerCpuBackPropagateCommand() const
{
	ndBrainLayerBackPropagateCpuCommand* const command = new ndBrainLayerBackPropagateCpuCommand(this);
	command->m_inputSize = GetInputSize();
	command->m_outputSize = GetOutputSize();
	return command;
}

ndBrainLayer::ndLayerUniformDataGpu ndBrainLayerActivationSoftmax::GetLayerUniformDataGpu(const ndBrainGpuContext* const context) const
{
	ndLayerUniformDataGpu data;
	data.m_shader = context->m_ndBrainLayerSoftmaxActivation;
	data.m_inputSize = GetInputSize();
	data.m_outputSize = GetOutputSize();

	return data;
}

void ndBrainLayerActivationSoftmax::FeedForward(const ndBrainLayerFeedFowardCpuCommand* const info, ndInt32 miniBatchIndex) const
{
	ndInt32 inputSize = info->m_inputSize;
	ndInt32 outputSize = info->m_outputSize;

	const ndBrainTrainerCpuInference* const trainer = info->m_owner;
	ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector input(&trainer->m_inputOutputBuffer[offset], inputSize);
	ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);

	ndBrainFloat max = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	{
		max = ndMax(input[i], max);
	}

	ndBrainFloat acc = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat in = ndMax((input[i] - max), ndBrainFloat(-30.0f));
		ndAssert(in <= ndBrainFloat(0.0f));
		ndBrainFloat prob = ndBrainFloat(ndExp(in));
		output[i] = prob;
		acc += prob;
	}

	ndAssert(acc > ndBrainFloat(0.0f));
	output.Scale(ndBrainFloat(1.0f) / acc);
	output.FlushToZero();
}

//void ndBrainLayerActivationSoftmax::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const info, ndInt32 miniBatchIndex) const
void ndBrainLayerActivationSoftmax::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const, ndInt32) const
{
	ndAssert(0);
}