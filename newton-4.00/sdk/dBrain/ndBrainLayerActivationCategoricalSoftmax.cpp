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
#include "ndBrainTrainerCpuInference.h"
#include "ndBrainTrainerGpuInference.h"
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
	return "ndBrainLayerActivationCategoricalSoftmax";
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
	// check that the ouputDerivative is a one hat encoding 
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
	const ndCommandShareInfo* const info = &command->m_info;
	const ndBrainTrainerCpu* const trainer = (ndBrainTrainerCpu*)command->m_owner;

	ndInt32 inputSize = info->m_inputSize;
	ndInt32 outputSize = info->m_outputSize;
	
	ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);
	
	ndInt32 dstOffset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector outputDerivative(&trainer->m_inputOuputGradientsBuffer[dstOffset + inputSize], outputSize);
	ndBrainMemVector inputDerivative (&trainer->m_inputOuputGradientsBuffer[dstOffset], inputSize);
	
	inputDerivative.Set(output);
	inputDerivative.Sub(outputDerivative);
	inputDerivative.FlushToZero();
}

ndBrainTrainerGpuCommand* ndBrainLayerActivationCategoricalSoftmax::CreateGpuBackPropagateCommand(
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
		owner, info, size_t(this), context, context->m_brainLayerCathegoricalSoftmaxBackPropagate, 
		miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias, inputOutputGradients, weightsAndBiasGradients);
	return command;
}

