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
#include "ndDeepBrainLayer.h"
#include "ndDeepBrainTrainingOperator.h"

ndDeepBrainTrainingOperator::ndDeepBrainTrainingOperator(ndDeepBrain* const brain)
	:ndClassAlloc()
	,m_instance(brain)
	,m_cost()
	,m_output()
	//,m_gradient()
	,m_outputDerivative()
	,m_ouputPrefixScan()
	//,m_gradientPrefixScan()
{
}

ndDeepBrainTrainingOperator::~ndDeepBrainTrainingOperator()
{
}

void ndDeepBrainTrainingOperator::InitGaussianWeights(ndReal mean, ndReal variance)
{
	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		layers[i]->InitGaussianWeights(mean, variance);
	}
}

void ndDeepBrainTrainingOperator::PrefixScan()
{
	const ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	m_ouputPrefixScan.SetCount(layers.GetCount() + 1);
	m_ouputPrefixScan[0] = (layers[0]->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		ndInt32 size = (layer->GetOuputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_ouputPrefixScan[i + 1] = size;
	}
	
	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_ouputPrefixScan.GetCount(); ++i)
	{
		ndInt32 size = m_ouputPrefixScan[i];
		m_ouputPrefixScan[i] = sum;
		sum += size;
	}
	m_output.SetCount(sum);
	m_output.SetValue(0.0f);
	m_outputDerivative.SetCount(sum);
	m_outputDerivative.SetValue(0.0f);

	//m_gradientPrefixScan.SetCount(layers.GetCount() + 1);
	//m_gradientPrefixScan[layers.GetCount()] = (layers[layers.GetCount() - 1]->GetOuputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	//for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	//{
	//	ndDeepBrainLayer* const layer = layers[i];
	//	ndInt32 size = (layer->GetInputSize() + 1 + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	//	m_gradientPrefixScan[i] = size * layer->GetOuputSize();
	//}
	//
	//sum = 0;
	//for (ndInt32 i = 0; i < m_gradientPrefixScan.GetCount(); ++i)
	//{
	//	ndInt32 size = m_gradientPrefixScan[i];
	//	m_gradientPrefixScan[i] = sum;
	//	sum += size;
	//}
	//m_gradient.SetCount(sum);
	//m_gradient.SetValue(0.0f);
}

void ndDeepBrainTrainingOperator::SetInput(const ndDeepBrainVector& input)
{
	ndDeepBrainLayer* const inputLayer = m_instance.GetLayers()[0];
	ndInt32 size = inputLayer->GetInputSize();
	dAssert(size == input.GetCount());

	m_instance.GetInputs().SetCount(size);
	m_instance.GetInputs().CopyData(input);

	ndDeepBrainVector& output = m_output;
	const ndDeepBrainPrefixScan& prefixSum = m_ouputPrefixScan;
	ndDeepBrainMemVector layerInput(&output[prefixSum[0]], size);
	layerInput.CopyData(input);
}

void ndDeepBrainTrainingOperator::MakePrediction(const ndDeepBrainVector& trainingInput)
{
	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	dAssert(layers.GetCount());

	SetInput(trainingInput);
	ndDeepBrainVector& input = m_instance.GetInputs();
	ndDeepBrainVector& output = m_instance.GetOutputs();
	
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		dAssert(m_instance.GetInputs().GetCount() == layer->GetInputSize());
	
		input.SetCount(layer->GetInputSize());
		output.SetCount(layer->GetOuputSize());
		layer->MakePrediction(input, output);
	
		ndDeepBrainMemVector layerOutput(&m_output[m_ouputPrefixScan[i + 1]], layer->GetOuputSize());
		layerOutput.CopyData(output);
	
		input.Swap(output);
	}
	input.Swap(output);
}

//void ndDeepBrainInstance::BackPropagate(ndDeepBrainTrainingOperator& trainingOperator)
void ndDeepBrainTrainingOperator::BackPropagate()
{
	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	dAssert(layers.GetCount());

	////const ndDeepBrainVector& cost = trainingOperator.m_cost;
	////const ndDeepBrainVector& gradient = trainingOperator.m_gradient;
	////const ndDeepBrainPrefixScan& gradientPrefixScan = trainingOperator.m_gradientPrefixScan;
	////ndDeepBrainMemVector temVector(&gradient[gradientPrefixScan[layers.GetCount()]], cost.GetCount());
	////temVector.CopyData(cost);
	//
	//const ndDeepBrainVector& output = trainingOperator.m_output;
	//ndDeepBrainVector& outputDerivative = trainingOperator.m_outputDerivative;
	//const ndDeepBrainPrefixScan& ouputPrefixScan = trainingOperator.m_ouputPrefixScan;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
	
	//	ndInt32 outputSize = layer->GetOuputSize();
	//
	//	const ndDeepBrainMemVector layerOutput(&output[ouputPrefixScan[i + 1]], outputSize);
	//	const ndDeepBrainMemVector layerOutputDevivative(&outputDerivative[ouputPrefixScan[i + 1]], outputSize);
	//	layer->ActivationDerivative(layerOutput, outputDerivative);
	//
	//	//	ndDeepBrainMemVector gradientVectorIn(&gradient[gradientPrefixScan[i + 1]], outputSize);
	//	//	outputDerivative.Scale(outputDerivative, gradientVectorIn[0]);
	//	//	
	//	//	ndInt32 inputSize = layer->GetInputSize();
	//	//
	//	//	const ndDeepBrainMemVector layerInput(&output[ouputPrefixScan[i]], inputSize);
	//	//	ndDeepBrainMemVector gradientVectorOut(&gradient[gradientPrefixScan[i]], inputSize + 1);
	//	//	for (ndInt32 j = 0; j < inputSize; j++)
	//	//	{
	//	//		gradientVectorOut[j] = outputDerivative[0] * layerInput[j];
	//	//	}
	//	//	gradientVectorOut[inputSize] = outputDerivative[0];
	}
}