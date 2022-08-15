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
	,m_zDerivative()
	//,m_preOutput()
	//,m_gradients()
	//,m_costGradients()
	//,m_biasGradients()
	//,m_outputDerivative()
	//,m_ouputPrefixScan()
	//,m_gradientsPrefixScan()
{
}

ndDeepBrainTrainingOperator::~ndDeepBrainTrainingOperator()
{
}

void ndDeepBrainTrainingOperator::PrefixScan()
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	m_instance.CalculatePrefixScan();
	m_zDerivative.SetCount(m_instance.m_z.GetCount());
	m_zDerivative.Set(0.0f);
	m_output.SetCount(layers[layers.GetCount() - 1]->GetOuputSize());
}

void ndDeepBrainTrainingOperator::MakePrediction(const ndDeepBrainVector& input)
{
	m_instance.MakePrediction(input, m_output);
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());

	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		const ndDeepBrainMemVector z(&m_instance.m_z[m_instance.m_zPrefixScan[i + 1]], layer->GetOuputSize());
		ndDeepBrainMemVector zDerivative(&m_zDerivative[m_instance.m_zPrefixScan[i + 1]], layer->GetOuputSize());
		layer->ActivationDerivative(z, zDerivative);
	}
}

void ndDeepBrainTrainingOperator::BackPropagateOutputLayer(const ndDeepBrainVector& groundTruth)
{
	ndAssert(0);
	//ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	//const ndInt32 layerIndex = layers.GetCount() - 1;
	//
	//ndDeepBrainLayer* const ouputLayer = layers[layerIndex];
	//const ndInt32 inputCount = ouputLayer->GetInputSize();
	//const ndInt32 outputCount = ouputLayer->GetOuputSize();
	//const ndInt32 weightGradientStride = (ouputLayer->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	//
	//m_costGradients.SetCount(outputCount);
	//
	//const ndDeepBrainMemVector output(&m_output[m_ouputPrefixScan[layerIndex]], outputCount);
	//m_costGradients.Sub(output, groundTruth);
	//
	//ndDeepBrainVector& input = m_instance.GetInputs();
	//const ndReal* const ptr = layerIndex ? &m_output[m_ouputPrefixScan[layerIndex - 1]] : &input[0];
	//const ndDeepBrainMemVector prevOutput(ptr, inputCount);
	//const ndDeepBrainMemVector outputDerivative(&m_outputDerivative[m_ouputPrefixScan[layerIndex]], outputCount);
	//ndDeepBrainMemVector biasGradients(&m_biasGradients[m_ouputPrefixScan[layerIndex]], outputCount);
	//
	//ndInt32 gradientStart = m_gradientsPrefixScan[layerIndex];
	//for (ndInt32 i = 0; i < outputCount; ++i)
	//{
	//	ndDeepBrainMemVector weightGradient(&m_gradients[gradientStart], inputCount);
	//	ndFloat32 derivative = m_costGradients[i] * outputDerivative[i];
	//	gradientStart += weightGradientStride;
	//
	//	biasGradients[i] = derivative;
	//	weightGradient.ScaleSet(prevOutput, derivative);
	//}
}

void ndDeepBrainTrainingOperator::BackPropagateHiddenLayer()
{
	ndAssert(0);
	//ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	//
	//for (ndInt32 layerIndex = layers.GetCount() - 2; layerIndex >= 0; --layerIndex)
	//{
	//	ndDeepBrainLayer* const layer = layers[layerIndex];
	//	ndDeepBrainLayer* const prevLayer = layers[layerIndex + 1];
	//
	//	const ndInt32 prevLayerInputCount = prevLayer->GetInputSize();
	//	const ndDeepBrainMemVector biasGradients(&m_biasGradients[m_ouputPrefixScan[layerIndex + 1]], prevLayer->GetInputSize());
	//
	//	ndDeepBrainVector xxxx;
	//	xxxx.SetCount(prevLayerInputCount);
	//	for (ndInt32 i = 0; i < prevLayerInputCount; ++i)
	//	{
	//		const ndDeepBrainVector& prevWeights = (*prevLayer)[i];
	//		xxxx.Mul(prevWeights, biasGradients);
	//	}
	//	
	//	//for (ndInt32 i = 0; i < outputCount; ++i)
	//	{
	//		//ndDeepBrainMemVector weightGradient(&m_gradients[gradientStart], inputCount);
	//		//ndFloat32 derivative = m_costGradients[i] * outputDerivative[i];
	//		//gradientStart += weightGradientStride;
	//		//
	//		//biasGradients[i] = derivative;
	//		//weightGradient.ScaleSet(prevOutput, derivative);
	//	}
	//}
}

void ndDeepBrainTrainingOperator::UpdateWeights(ndReal learnRate)
{
	ndAssert(0);
	//ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	//for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	//{
	//	ndDeepBrainLayer* const layer = layers[i];
	//
	//	const ndInt32 weightGradientStride = (layer->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	//	ndInt32 gradientStart = m_gradientsPrefixScan[i];
	//
	//	const ndInt32 inputSize = layer->GetInputSize();
	//	const ndInt32 outputSize = layer->GetOuputSize();
	//
	//	ndDeepBrainVector& bias = layer->GetBias();
	//	const ndDeepBrainMemVector biasGradients(&m_biasGradients[m_ouputPrefixScan[i]], outputSize);
	//	bias.ScaleAdd(biasGradients, learnRate);
	//
	//	for (ndInt32 j = 0; j < outputSize; ++j)
	//	{
	//		ndDeepBrainVector& weightVector = (*layer)[j];
	//		const ndDeepBrainMemVector weightGradients(&m_gradients[gradientStart], inputSize);
	//		weightVector.ScaleAdd(weightGradients, learnRate);
	//	}
	//}
}

void ndDeepBrainTrainingOperator::BackPropagate(ndReal learnRate, const ndDeepBrainVector& groundTruth)
{
	//BackPropagateOutputLayer(groundTruth);
	//BackPropagateHiddenLayer();
	//UpdateWeights(learnRate);
}