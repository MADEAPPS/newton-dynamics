/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrain.h"
#include "ndDeepBrainInstance.h"
#include "ndDeepBrainTrainingOperator.h"

ndDeepBrainInstance::ndDeepBrainInstance(ndDeepBrain* const brain)
	:ndClassAlloc()
	,m_inputs()
	,m_outputs()
	,m_brain(brain)
{
}

ndDeepBrainInstance::~ndDeepBrainInstance()
{
}

ndDeepBrainVector& ndDeepBrainInstance::GetInputs()
{
	return m_inputs;
}

ndDeepBrainVector& ndDeepBrainInstance::GetOutputs()
{
	return m_outputs;
}

ndArray<ndDeepBrainLayer*>& ndDeepBrainInstance::GetLayers()
{
	return *m_brain;
}

const ndArray<ndDeepBrainLayer*>& ndDeepBrainInstance::GetLayers() const
{
	return *m_brain;
}

void ndDeepBrainInstance::SetInput(const ndDeepBrainVector& input)
{
	ndDeepBrainLayer* const inputLayer = (*m_brain)[0];
	ndInt32 size = inputLayer->GetInputSize();
	dAssert(size == input.GetCount());

	m_inputs.SetCount(size);
	m_inputs.CopyData(input);
}

void ndDeepBrainInstance::MakePrediction(const ndDeepBrainVector& input)
{
	dAssert(0);
	SetInput(input);
	ndArray<ndDeepBrainLayer*>& layers = (*m_brain);
	dAssert(layers.GetCount());

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		dAssert(m_inputs.GetCount() == layer->GetInputSize());

		ndInt32 neuronsCount = layer->GetOuputSize();
		m_outputs.SetCount(neuronsCount);
		layer->MakePrediction(m_inputs, m_outputs);
		m_inputs.Swap(m_outputs);
	}
	m_inputs.Swap(m_outputs);
}

void ndDeepBrainInstance::MakeTrainingPrediction(const ndDeepBrainVector &input, ndDeepBrainTrainingOperator& trainingOperator)
{
	SetInput(input);
	ndArray<ndDeepBrainLayer*>& layers = (*m_brain);
	dAssert(layers.GetCount());

	ndDeepBrainVector& output = trainingOperator.m_output;
	const ndDeepBrainPrefixScan& prefixSum = trainingOperator.m_ouputPrefixScan;

	ndDeepBrainMemVector layerInput(&output[prefixSum[0]], m_inputs.GetCount());
	layerInput.CopyData(input);

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		dAssert(m_inputs.GetCount() == layer->GetInputSize());
		
		ndInt32 outputCount = layer->GetOuputSize();
		m_outputs.SetCount(outputCount);
		layer->MakePrediction(m_inputs, m_outputs);
		
		ndDeepBrainMemVector layerOutput(&output[prefixSum[i + 1]], outputCount);
		layerOutput.CopyData(m_outputs);

		m_inputs.Swap(m_outputs);
	}
	m_inputs.Swap(m_outputs);
}

void ndDeepBrainInstance::BackPropagate(ndDeepBrainTrainingOperator& trainingOperator)
{
	ndArray<ndDeepBrainLayer*>& layers = (*m_brain);
	dAssert(layers.GetCount());

	const ndDeepBrainVector& cost = trainingOperator.m_cost;
	const ndDeepBrainVector& gradient = trainingOperator.m_gradient;
	const ndDeepBrainPrefixScan& gradientPrefixScan = trainingOperator.m_gradientPrefixScan;
	ndDeepBrainMemVector temVector(&gradient[gradientPrefixScan[layers.GetCount()]], cost.GetCount());
	temVector.CopyData(cost);

	const ndDeepBrainVector& output = trainingOperator.m_output;
	ndDeepBrainVector& outputDerivative = trainingOperator.m_outputDerivative;
	const ndDeepBrainPrefixScan& ouputPrefixScan = trainingOperator.m_ouputPrefixScan;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];

		ndInt32 outputSize = layer->GetOuputSize();
		outputDerivative.SetCount(outputSize);

		const ndDeepBrainMemVector layerOutput(&output[ouputPrefixScan[i + 1]], outputSize);
		layer->ActivationDerivative(layerOutput, outputDerivative);

		ndDeepBrainMemVector gradientVector(&gradient[gradientPrefixScan[i + 1]], outputSize);
		outputDerivative.Scale(outputDerivative, gradientVector[0]);
		
		//for (ndInt32 j = 0; j < outputSize; j++)
		//{
		//	outputDerivative.Scale(outputDerivative, gradientVector[j]);
		//
		//		//for (ndInt32 j = layer->GetInputSize() - 1; j >= 0; --j)
		//		//{
		//		//
		//		//}
		//}
	}
}