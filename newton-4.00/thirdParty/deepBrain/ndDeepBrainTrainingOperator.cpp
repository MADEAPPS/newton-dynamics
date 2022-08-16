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
	,m_g()
	,m_zDerivative()
	,m_weightGradients()
	,m_weightGradientsPrefitScan()
{
}

ndDeepBrainTrainingOperator::~ndDeepBrainTrainingOperator()
{
}

void ndDeepBrainTrainingOperator::PrefixScan()
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	m_instance.CalculatePrefixScan();
	m_g.SetCount(m_instance.m_z.GetCount());
	m_zDerivative.SetCount(m_instance.m_z.GetCount());

	m_g.Set(0.0f);
	m_zDerivative.Set(0.0f);
	m_output.SetCount(layers[layers.GetCount() - 1]->GetOuputSize());

	m_weightGradientsPrefitScan.SetCount(layers.GetCount());
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		ndInt32 stride = (layer->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_weightGradientsPrefitScan[i] = stride * layer->GetOuputSize();
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_weightGradientsPrefitScan.GetCount(); ++i)
	{
		ndInt32 count = m_weightGradientsPrefitScan[i];
		m_weightGradientsPrefitScan[i] = sum;
		sum += count;
	}
	m_weightGradients.SetCount(sum);
	m_weightGradients.Set(0.0f);
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

void ndDeepBrainTrainingOperator::BackPropagateOutputLayer(const ndDeepBrainVector& groundTruth)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	const ndInt32 layerIndex = layers.GetCount() - 1;
	
	ndDeepBrainLayer* const ouputLayer = layers[layerIndex];
	const ndInt32 inputCount = ouputLayer->GetInputSize();
	const ndInt32 outputCount = ouputLayer->GetOuputSize();
	ndDeepBrainMemVector g(&m_g[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector z(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);

	g.Sub(z, groundTruth);
	g.Mul(g, zDerivative);

	const ndInt32 stride = (ouputLayer->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;

	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefitScan[layerIndex]];
	const ndDeepBrainMemVector z0(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex]], inputCount);
	for (ndInt32 i = 0; i < outputCount; ++i)
	{
		ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
		ndFloat32 gValue = g[i];
		weightGradientPtr += stride;
		weightGradient.ScaleSet(z0, gValue);
	}
}

void ndDeepBrainTrainingOperator::BackPropagateHiddenLayer(ndInt32 layerIndex)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());

	ndDeepBrainLayer* const ouputLayer = layers[layerIndex];
	const ndInt32 inputCount = ouputLayer->GetInputSize();
	const ndInt32 outputCount = ouputLayer->GetOuputSize();
	ndDeepBrainMemVector g(&m_g[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector z(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);

	//g.Sub(z, groundTruth);
	//g.Mul(g, zDerivative);
	//
	//const ndInt32 stride = (ouputLayer->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	//
	//ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefitScan[layerIndex]];
	//const ndDeepBrainMemVector z0(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex]], inputCount);
	//for (ndInt32 i = 0; i < outputCount; ++i)
	//{
	//	ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
	//	ndFloat32 gValue = g[i];
	//	weightGradientPtr += stride;
	//	weightGradient.ScaleSet(z0, gValue);
	//}
}

void ndDeepBrainTrainingOperator::BackPropagate(ndReal learnRate, const ndDeepBrainVector& groundTruth)
{
	BackPropagateOutputLayer(groundTruth);

	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = layers.GetCount() - 2; i >= 0; --i)
	{
		BackPropagateHiddenLayer(i);
	}
	
	//UpdateWeights(learnRate);
}