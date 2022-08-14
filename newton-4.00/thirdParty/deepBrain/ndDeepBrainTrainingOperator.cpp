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
	,m_output()
	,m_preOutput()
	,m_gradients()
	,m_costGradients()
	,m_outputDerivative()
	,m_ouputPrefixScan()
	,m_gradientsPrefixScan()
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
	m_ouputPrefixScan.SetCount(layers.GetCount());
	m_gradientsPrefixScan.SetCount(layers.GetCount());
	
	//m_ouputPrefixScan[0] = (layers[0]->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		ndInt32 size = (layer->GetOuputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_ouputPrefixScan[i] = size;
	
		ndInt32 weightStride = (layer->GetInputSize() + 1 + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_gradientsPrefixScan[i] = layer->GetOuputSize() * weightStride;
	}
	
	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_ouputPrefixScan.GetCount(); ++i)
	{
		ndInt32 size = m_ouputPrefixScan[i];
		m_ouputPrefixScan[i] = sum;
		sum += size;
	}
	
	m_output.SetCount(sum);
	m_preOutput.SetCount(sum);
	m_outputDerivative.SetCount(sum);
	
	m_output.SetValue(0.0f);
	m_preOutput.SetValue(0.0f);
	m_outputDerivative.SetValue(0.0f);
	
	sum = 0;
	for (ndInt32 i = 0; i < m_gradientsPrefixScan.GetCount(); ++i)
	{
		ndInt32 size = m_gradientsPrefixScan[i];
		m_gradientsPrefixScan[i] = sum;
		sum += size;
	}
	m_gradients.SetCount(sum);
	m_gradients.SetValue(0.0f);
}

//void ndDeepBrainTrainingOperator::SetInput(const ndDeepBrainVector& input)
//{
//	dAssert(0);
//	//ndDeepBrainLayer* const inputLayer = m_instance.GetLayers()[0];
//	//ndInt32 size = inputLayer->GetInputSize();
//	//dAssert(size == input.GetCount());
//	//
//	//m_instance.GetInputs().SetCount(size);
//	//m_instance.GetInputs().CopyData(input);
//	//
//	//ndDeepBrainVector& output = m_output;
//	//const ndDeepBrainPrefixScan& prefixSum = m_ouputPrefixScan;
//	//ndDeepBrainMemVector layerInput(&output[prefixSum[0]], size);
//	//layerInput.CopyData(input);
//}

void ndDeepBrainTrainingOperator::PredictInputLayer(const ndDeepBrainVector& trainingInput)
{
	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	dAssert(layers.GetCount());
	
	//SetInput(trainingInput);

	ndDeepBrainVector& input = m_instance.GetInputs();
	ndDeepBrainVector& output = m_instance.GetOutputs();
	
	ndDeepBrainLayer* const layer = layers[0];
	ndDeepBrainMemVector layerOutput(&m_output[m_ouputPrefixScan[0]], layer->GetOuputSize());
	ndDeepBrainMemVector layerPreOutput(&m_preOutput[m_ouputPrefixScan[0]], layer->GetOuputSize());
	ndDeepBrainMemVector layerOutputDerivative(&m_outputDerivative[m_ouputPrefixScan[0]], layer->GetOuputSize());

	output.SetCount(layer->GetOuputSize());
	layer->Mul(trainingInput, output);
	output.Add(output, layer->GetBias());
	layerPreOutput.CopyData(output);
	layer->ApplyActivation(output);
	layerOutput.CopyData(output);
	layer->ActivationDerivative(output, layerOutputDerivative);
	input.Swap(output);
}

void ndDeepBrainTrainingOperator::MakePrediction(const ndDeepBrainVector& trainingInput)
{
	PredictInputLayer(trainingInput);

	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	dAssert(layers.GetCount());
	
	ndDeepBrainVector& input = m_instance.GetInputs();
	ndDeepBrainVector& output = m_instance.GetOutputs();
	
	for (ndInt32 i = 1; i < layers.GetCount(); ++i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		dAssert(m_instance.GetInputs().GetCount() == layer->GetInputSize());
	
		input.SetCount(layer->GetInputSize());
		output.SetCount(layer->GetOuputSize());
	
		ndDeepBrainMemVector layerOutput(&m_output[m_ouputPrefixScan[i]], layer->GetOuputSize());
		ndDeepBrainMemVector layerPreOutput(&m_preOutput[m_ouputPrefixScan[i]], layer->GetOuputSize());
		ndDeepBrainMemVector layerOutputDerivative(&m_outputDerivative[m_ouputPrefixScan[i]], layer->GetOuputSize());
	
		layer->Mul(input, output);
		output.Add(output, layer->GetBias());
		layerPreOutput.CopyData(output);
		layer->ApplyActivation(output);
		layerOutput.CopyData(output);
		layer->ActivationDerivative(output, layerOutputDerivative);
	
		input.Swap(output);
	}
	input.Swap(output);

	input.SetCount(layers[0]->GetCount());
	input.CopyData(trainingInput);
}

void ndDeepBrainTrainingOperator::BackPropagateHiddenLayer()
{
	//ndDeepBrainLayer* const layer = m_instance.GetLayers()[m_instance.GetLayers().GetCount()];
	//ndDeepBrainLayer* const layer = m_instance.GetLayers()[m_instance.GetLayers().GetCount()];
	//dAssert(layers.GetCount());
	//
	//ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();

}

void ndDeepBrainTrainingOperator::BackPropagateOutputLayer(const ndDeepBrainVector& groundTruth)
{
	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	const ndInt32 layerIndex = layers.GetCount() - 1;
	
	ndDeepBrainLayer* const ouputLayer = layers[layerIndex];
	const ndInt32 inputCount = ouputLayer->GetInputSize();
	const ndInt32 outputCount = ouputLayer->GetOuputSize();
	const ndInt32 weightGradientStride = (ouputLayer->GetInputSize() + 1 + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	
	m_costGradients.SetCount(outputCount);
	
	const ndDeepBrainMemVector output(&m_output[m_ouputPrefixScan[layerIndex]], outputCount);
	m_costGradients.Sub(output, groundTruth);

	ndDeepBrainVector& input = m_instance.GetInputs();
	const ndReal* const ptr = layerIndex ? &m_output[m_ouputPrefixScan[layerIndex - 1]] : &input[0];
	const ndDeepBrainMemVector prevOutput(ptr, inputCount);
	const ndDeepBrainMemVector outputDerivative(&m_outputDerivative[m_ouputPrefixScan[layerIndex]], outputCount);
	
	ndInt32 gradientStart = m_gradientsPrefixScan[layerIndex];
	for (ndInt32 i = 0; i < outputCount; ++i)
	{
		ndDeepBrainMemVector weightGradient(&m_gradients[gradientStart], inputCount + 1);
		ndFloat32 derivative = m_costGradients[i] * outputDerivative[i];
		weightGradient[inputCount] = derivative;
		weightGradient.SetCount(inputCount);
		weightGradient.ScaleSet(prevOutput, derivative);
		gradientStart += weightGradientStride;
	}
}

void ndDeepBrainTrainingOperator::UpdateWeights(ndReal learnRate)
{
	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndDeepBrainLayer* const layer = layers[i];

		const ndInt32 weightGradientStride = (layer->GetInputSize() + 1 + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		ndInt32 gradientStart = m_gradientsPrefixScan[i];

		ndInt32 inputSize = layer->GetInputSize();
		ndDeepBrainVector& bias = layer->GetBias();
		
		for (ndInt32 j = 0; j < layer->GetOuputSize(); ++j)
		{
			ndDeepBrainVector& weightVector = (*layer)[j];
			ndDeepBrainMemVector weightDerivative(&m_gradients[gradientStart], inputSize);
			weightVector.MulScale(weightDerivative, learnRate);
			gradientStart += weightGradientStride;
			bias[j] += learnRate * weightDerivative[inputSize];
		}
	}
}

void ndDeepBrainTrainingOperator::BackPropagate(ndReal learnRate, const ndDeepBrainVector& groundTruth)
{
	BackPropagateOutputLayer(groundTruth);
	BackPropagateHiddenLayer();
	UpdateWeights(learnRate);

	//ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	//dAssert(layers.GetCount());
	//dAssert(0);
	//m_cost.SetCount(layers[layers.GetCount() - 1]->GetOuputSize());
	//m_cost.Sub(m_instance.GetOutputs(), groundTruth);

	//const ndDeepBrainVector& cost = trainingOperator.m_cost;
	//const ndDeepBrainVector& gradient = trainingOperator.m_gradient;
	//const ndDeepBrainPrefixScan& gradientPrefixScan = trainingOperator.m_gradientPrefixScan;
	//ndDeepBrainMemVector temVector(&gradient[gradientPrefixScan[layers.GetCount()]], cost.GetCount());
	//temVector.CopyData(cost);
	
	//const ndDeepBrainVector& output = m_output;
	//ndDeepBrainVector& outputDerivative = m_outputDerivative;
	//const ndDeepBrainPrefixScan& ouputPrefixScan = m_ouputPrefixScan;
	//for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	//{
	//	ndDeepBrainLayer* const layer = layers[i];
	//
	//	ndInt32 outputSize = layer->GetOuputSize();
	//
	//	const ndDeepBrainMemVector layerOutput(&output[ouputPrefixScan[i + 1]], outputSize);
	//	ndDeepBrainMemVector layerOutputDevivative(&outputDerivative[ouputPrefixScan[i + 1]], outputSize);
	//	layer->ActivationDerivative(layerOutput, layerOutputDevivative);
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
	//}
}