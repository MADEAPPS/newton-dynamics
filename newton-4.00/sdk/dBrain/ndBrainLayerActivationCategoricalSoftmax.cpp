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
#include "ndBrainSaveLoad.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainLayerActivationCategoricalSoftmax.h"

ndBrainLayerActivationCategoricalSoftmax::ndBrainLayerActivationCategoricalSoftmax(ndInt32 neurons)
	:ndBrainLayerActivationSoftmax(neurons)
{
}

ndBrainLayerActivationCategoricalSoftmax::ndBrainLayerActivationCategoricalSoftmax(const ndBrainLayerActivationCategoricalSoftmax& src)
	:ndBrainLayerActivationSoftmax(src)
{
}

ndBrainLayer* ndBrainLayerActivationCategoricalSoftmax::Clone() const
{
	return new ndBrainLayerActivationCategoricalSoftmax(*this);
}


const char* ndBrainLayerActivationCategoricalSoftmax::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_CATEGORICAL_SOFTMAX;
}

ndBrainLayer* ndBrainLayerActivationCategoricalSoftmax::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationCategoricalSoftmax* const layer = new ndBrainLayerActivationCategoricalSoftmax(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationCategoricalSoftmax::InputDerivative(const ndBrainVector&, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	#ifdef _DEBUG
	// check that the outputDerivative is a one hat encoding 
	ndAssert(output.GetCount() == inputDerivative.GetCount());
	ndAssert(output.GetCount() == outputDerivative.GetCount());

	ndInt32 index = 0;
	for (ndInt32 i = 0; i < outputDerivative.GetCount(); ++i)
	{
		ndAssert((outputDerivative[i] == ndBrainFloat(0.0f)) || (outputDerivative[i] == ndBrainFloat(1.0f)));
		index += (outputDerivative[i] == ndBrainFloat(1.0f)) ? 1 : 0;
	}
	ndAssert(index == 1);
	#endif

	// basically it acts as a loss function
	inputDerivative.Set(output);
	inputDerivative.Sub(outputDerivative);

	inputDerivative.FlushToZero();
}

void ndBrainLayerActivationCategoricalSoftmax::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	
	ndInt32 inputSize = info.m_inputSize;
	//ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);
	ndAssert(inputSize == info.m_outputSize);

	const ndBrainMemVector output(&inputOutputBuffer[dstBase], inputSize);
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], inputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);

	inputDerivative.Set(output);
	inputDerivative.Sub(outputDerivative);
	inputDerivative.FlushToZero();
}

ndCommandArray ndBrainLayerActivationCategoricalSoftmax::CreateGpuBackPropagateCommand(
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
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerCathegoricalSoftmaxBackPropagate;
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		commands.PushBack(command);
	}
	return commands;
}

