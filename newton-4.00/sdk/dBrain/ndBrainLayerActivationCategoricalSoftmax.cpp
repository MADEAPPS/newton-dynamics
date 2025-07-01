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
	ndInt32 outputSize = info.m_outputSize;
	
	ndInt32 offset = miniBatchIndex * info.m_inputOutputSize + info.m_inputOutputStartOffset;
	ndAssert(offset >= 0);
	const ndBrainMemVector output(&inputOutputBuffer[offset + inputSize], outputSize);
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[offset + inputSize], outputSize);
	ndBrainMemVector inputDerivative (&inputOutputGradientsBuffer[offset], inputSize);

	inputDerivative.Set(output);
	inputDerivative.Sub(outputDerivative);
	inputDerivative.FlushToZero();
}

ndBrainBufferCommand* ndBrainLayerActivationCategoricalSoftmax::CreateGpuBackPropagateCommand(
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
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerCathegoricalSoftmaxBackPropagate;
		//ndBrainBufferCommand* const command = new ndBrainTrainerGpuCommand(
		//	owner, info, size_t(this), context, context->m_brainLayerCathegoricalSoftmaxBackPropagate, 
		//	miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias, inputOutputGradients, weightsAndBiasGradients);
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		return command;
	}
}

