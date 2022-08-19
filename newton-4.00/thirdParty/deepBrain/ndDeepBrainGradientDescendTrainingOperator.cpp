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
#include "ndDeepBrainGradientDescendTrainingOperator.h"

ndDeepBrainGradientDescendTrainingOperator::ndDeepBrainGradientDescendTrainingOperator(ndDeepBrain* const brain)
	:ndDeepBrainTrainingOperator(brain)
	,m_output()
	,m_zDerivative()
	,m_biasGradients()
	,m_weightGradients()
	,m_weightGradientsPrefixScan()
	,m_weightsLayersTranspose()
{
	PrefixScan();

	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		const ndDeepBrainLayer& layer = *layers[i];
		m_weightsLayersTranspose.PushBack(new ndDeepBrainMatrix(layer.GetInputSize(), layer.GetOuputSize()));
		m_weightsLayersTranspose[i]->SetTranspose(layer);
	}
}

ndDeepBrainGradientDescendTrainingOperator::ndDeepBrainGradientDescendTrainingOperator(const ndDeepBrainGradientDescendTrainingOperator& src)
	:ndDeepBrainTrainingOperator(src)
	,m_output(src.m_output)
	,m_zDerivative(src.m_zDerivative)
	,m_biasGradients(src.m_biasGradients)
	,m_weightGradients(src.m_weightGradients)
	,m_weightGradientsPrefixScan(src.m_weightGradientsPrefixScan)
	,m_weightsLayersTranspose()
{
	for (ndInt32 i = 0; i < src.m_weightsLayersTranspose.GetCount(); i++)
	{
		m_weightsLayersTranspose.PushBack(new ndDeepBrainMatrix(*src.m_weightsLayersTranspose[i]));
	}
}

ndDeepBrainGradientDescendTrainingOperator::~ndDeepBrainGradientDescendTrainingOperator()
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = 0; i < layers.GetCount(); i++)
	{
		delete (m_weightsLayersTranspose[i]);
	}
}

void ndDeepBrainGradientDescendTrainingOperator::PrefixScan()
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	m_instance.CalculatePrefixScan();
	m_zDerivative.SetCount(m_instance.m_z.GetCount());
	m_biasGradients.SetCount(m_instance.m_z.GetCount());

	m_zDerivative.Set(0.0f);
	m_biasGradients.Set(0.0f);
	m_output.SetCount(layers[layers.GetCount() - 1]->GetOuputSize());

	m_weightGradientsPrefixScan.SetCount(layers.GetCount());
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		ndInt32 stride = (layer->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_weightGradientsPrefixScan[i] = stride * layer->GetOuputSize();
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_weightGradientsPrefixScan.GetCount(); ++i)
	{
		ndInt32 count = m_weightGradientsPrefixScan[i];
		m_weightGradientsPrefixScan[i] = sum;
		sum += count;
	}
	m_weightGradients.SetCount(sum);
	m_weightGradients.Set(0.0f);
}

void ndDeepBrainGradientDescendTrainingOperator::MakePrediction(const ndDeepBrainVector& input)
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

void ndDeepBrainGradientDescendTrainingOperator::BackPropagateOutputLayer(const ndDeepBrainVector& groundTruth)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	const ndInt32 layerIndex = layers.GetCount() - 1;

	ndDeepBrainLayer* const ouputLayer = layers[layerIndex];
	const ndInt32 inputCount = ouputLayer->GetInputSize();
	const ndInt32 outputCount = ouputLayer->GetOuputSize();
	ndDeepBrainMemVector biasGradients(&m_biasGradients[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector z(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);

	biasGradients.Sub(z, groundTruth);
	biasGradients.Mul(biasGradients, zDerivative);

	const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[layerIndex]];
	const ndDeepBrainMemVector z0(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex]], inputCount);
	for (ndInt32 i = 0; i < outputCount; ++i)
	{
		ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
		ndFloat32 gValue = biasGradients[i];
		weightGradient.ScaleSet(z0, gValue);
		weightGradientPtr += stride;
	}
}

void ndDeepBrainGradientDescendTrainingOperator::BackPropagateHiddenLayer(ndInt32 layerIndex)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	BackPropagateCalculateBiasGradient(layerIndex);

	ndDeepBrainLayer* const layer = layers[layerIndex];

	const ndDeepBrainMemVector biasGradients(&m_biasGradients[m_instance.m_zPrefixScan[layerIndex + 1]], layer->GetOuputSize());

	const ndInt32 inputCount = layer->GetInputSize();
	const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[layerIndex]];

	const ndDeepBrainMemVector z0(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex]], inputCount);
	for (ndInt32 i = 0; i < layer->GetOuputSize(); ++i)
	{
		ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
		ndFloat32 gValue = biasGradients[i];
		weightGradient.ScaleSet(z0, gValue);
		weightGradientPtr += stride;
	}
}

void ndDeepBrainGradientDescendTrainingOperator::BackPropagateCalculateBiasGradient(ndInt32 layerIndex)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	ndDeepBrainLayer* const layer = layers[layerIndex + 1];

	const ndDeepBrainMemVector biasGradients1(&m_biasGradients[m_instance.m_zPrefixScan[layerIndex + 2]], layer->GetOuputSize());
	ndDeepBrainMemVector biasGradients(&m_biasGradients[m_instance.m_zPrefixScan[layerIndex + 1]], layer->GetInputSize());
	const ndDeepBrainMatrix& matrix = *m_weightsLayersTranspose[layerIndex + 1];
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[m_instance.m_zPrefixScan[layerIndex + 1]], layer->GetInputSize());

	matrix.Mul(biasGradients1, biasGradients);
	biasGradients.Mul(biasGradients, zDerivative);
}

void ndDeepBrainGradientDescendTrainingOperator::UpdateWeights(ndReal learnRate)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		const ndInt32 inputSize = layer->GetInputSize();
		const ndInt32 outputSize = layer->GetOuputSize();

		const ndDeepBrainMemVector biasGradients(&m_biasGradients[m_instance.m_zPrefixScan[i + 1]], outputSize);
		layer->m_bias.ScaleAdd(biasGradients, -learnRate);

		const ndInt32 weightGradientStride = (inputSize + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[i]];

		ndDeepBrainMatrix& weightMatrix = *layer;
		for (ndInt32 j = 0; j < outputSize; ++j)
		{
			ndDeepBrainVector& weightVector = weightMatrix[j];
			const ndDeepBrainMemVector weightGradients(weightGradientPtr, inputSize);
			weightVector.ScaleAdd(weightGradients, -learnRate);
			weightGradientPtr += weightGradientStride;
		}
	}
}

void ndDeepBrainGradientDescendTrainingOperator::ApplyWeightTranspose()
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		ndDeepBrainMatrix& weightMatrix = *layer;
		m_weightsLayersTranspose[i]->SetTranspose(weightMatrix);
	}
}

void ndDeepBrainGradientDescendTrainingOperator::BackPropagate(const ndDeepBrainVector& groundTruth)
{
	BackPropagateOutputLayer(groundTruth);

	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = layers.GetCount() - 2; i >= 0; --i)
	{
		BackPropagateHiddenLayer(i);
	}
}

void ndDeepBrainGradientDescendTrainingOperator::Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	ndAssert(inputBatch.GetCount() == groundTruth.GetCount());
	ndAssert(m_output.GetCount() == groundTruth[0].GetCount());

	ndInt32 index = 0;
	ndInt32 batchCount = (inputBatch.GetCount() + m_miniBatchSize - 1) / m_miniBatchSize;
	for (ndInt32 i = 0; i < steps; ++i)
	{
		const ndInt32 batchStart = index * m_miniBatchSize;
		const ndInt32 batchSize = index != (batchCount - 1) ? m_miniBatchSize : inputBatch.GetCount() - batchStart;
		index = (index + 1) % batchCount;

		m_averageError = 0.0f;
		for (ndInt32 j = 0; j < batchSize; ++j)
		{
			const ndDeepBrainVector& input = inputBatch[batchStart + j];
			const ndDeepBrainVector& truth = groundTruth[batchStart + j];
			MakePrediction(input);
			BackPropagate(truth);
			UpdateWeights(learnRate);
			ndFloat32 error = CalculateMeanSquareError(truth);
			m_averageError += error;
		}
		ApplyWeightTranspose();
		m_averageError = ndSqrt(m_averageError / batchSize);
		ndExpandTraceMessage("%f\n", m_averageError);
	}
}