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

void ndBrainLayerActivationSoftmax::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
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

//void ndBrainLayerActivationSoftmax::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
void ndBrainLayerActivationSoftmax::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const, ndInt32) const
{
	ndAssert(0);
}

ndCommandArray ndBrainLayerActivationSoftmax::CreateGpuFeedForwardCommand(
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
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerSoftmaxActivation;
		command = new ndBrainGpuCommand(descriptor);
	}

	ndCommandArray commandArray(0);
	commandArray.PushBack(command);
	return commandArray;
}
