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
#include "ndBrainLayer.h"
#include "ndBrainTrainer.h"

ndBrainTrainer::ndBrainTrainer(ndBrain* const brain)
	:ndBrainTrainerBase(brain)
	,m_output()
	,m_zDerivative()
	,m_biasGradients()
	,m_weightGradients()
	,m_biasGradientsAcc()
	,m_biasGradient_u()
	,m_biasGradient_v()
	,m_weightGradient_u()
	,m_weightGradient_v()
	,m_weightGradientsPrefixScan()
	,m_weightsLayersTranspose()
	,m_regularizer(1.0e-6f)
	,m_bestCost(1.0e10f)
	,m_alpha(0.9f)
	,m_beta(0.999f)
	,m_epsilon(1.0e-8f)
	,m_alphaAcc(m_alpha)
	,m_betaAcc(m_beta)
{
	ndAssert(m_regularizer >= 0.0f);
	PrefixScan();

	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		const ndBrainLayer& layer = *layers[i];
		m_weightsLayersTranspose.PushBack(new ndBrainMatrix(layer.GetInputSize(), layer.GetOuputSize()));
		m_weightsLayersTranspose[i]->SetTranspose(layer);
	}
}

ndBrainTrainer::ndBrainTrainer(const ndBrainTrainer& src)
	:ndBrainTrainerBase(src)
	,m_output(src.m_output)
	,m_zDerivative(src.m_zDerivative)
	,m_biasGradients(src.m_biasGradients)
	,m_weightGradients(src.m_weightGradients)
	,m_biasGradientsAcc(src.m_biasGradientsAcc)
	,m_biasGradient_u(src.m_biasGradient_u)
	,m_biasGradient_v(src.m_biasGradient_v)
	,m_weightGradient_u(src.m_weightGradient_u)
	,m_weightGradient_v(src.m_weightGradient_v)
	,m_weightGradientsPrefixScan(src.m_weightGradientsPrefixScan)
	,m_weightsLayersTranspose()
	,m_regularizer(src.m_regularizer)
	,m_alpha(src.m_alpha)
	,m_beta(src.m_beta)
	,m_epsilon(src.m_epsilon)
	,m_alphaAcc(src.m_alphaAcc)
	,m_betaAcc(src.m_betaAcc)
{
	for (ndInt32 i = 0; i < src.m_weightsLayersTranspose.GetCount(); i++)
	{
		m_weightsLayersTranspose.PushBack(new ndBrainMatrix(*src.m_weightsLayersTranspose[i]));
	}
}

ndBrainTrainer::~ndBrainTrainer()
{
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = 0; i < layers.GetCount(); i++)
	{
		delete (m_weightsLayersTranspose[i]);
	}
}

void ndBrainTrainer::PrefixScan()
{
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	m_instance.CalculatePrefixScan();

	ndBrainVector& instance_z = m_instance.GetOutput();
	m_zDerivative.SetCount(instance_z.GetCount());
	m_biasGradients.SetCount(instance_z.GetCount());
	m_biasGradientsAcc.SetCount(instance_z.GetCount());
	m_biasGradient_u.SetCount(instance_z.GetCount());;
	m_biasGradient_v.SetCount(instance_z.GetCount());;

	m_zDerivative.Set(0.0f);
	m_biasGradients.Set(0.0f);
	m_biasGradientsAcc.Set(0.0f);
	m_biasGradient_u.Set(0.0f);
	m_biasGradient_v.Set(0.0f);
	m_output.SetCount(layers[layers.GetCount() - 1]->GetOuputSize());

	m_weightGradientsPrefixScan.SetCount(layers.GetCount());
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = layers[i];
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
	m_weightGradient_u.SetCount(sum);
	m_weightGradient_v.SetCount(sum);

	m_weightGradients.Set(0.0f);
	m_weightGradient_u.Set(0.0f);
	m_weightGradient_v.Set(0.0f);
}

void ndBrainTrainer::MakePrediction(const ndBrainVector& input)
{
	m_instance.MakePrediction(input, m_output);
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());

	ndBrainVector& instance_z = m_instance.GetOutput();
	const ndBrainPrefixScan& preFixScan = m_instance.GetPrefixScan();
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = layers[i];
		const ndDeepBrainMemVector z(&instance_z[preFixScan[i + 1]], layer->GetOuputSize());
		ndDeepBrainMemVector zDerivative(&m_zDerivative[preFixScan[i + 1]], layer->GetOuputSize());
		layer->ActivationDerivative(z, zDerivative);
	}
}

void ndBrainTrainer::BackPropagateOutputLayer(const ndBrainVector& groundTruth)
{
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	const ndInt32 layerIndex = layers.GetCount() - 1;

	ndBrainLayer* const ouputLayer = layers[layerIndex];
	const ndInt32 inputCount = ouputLayer->GetInputSize();
	const ndInt32 outputCount = ouputLayer->GetOuputSize();
	const ndBrainVector& instance_z = m_instance.GetOutput();
	const ndBrainPrefixScan& preFixScan = m_instance.GetPrefixScan();
	ndDeepBrainMemVector biasGradients(&m_biasGradients[preFixScan[layerIndex + 1]], outputCount);
	ndDeepBrainMemVector biasGradientsAcc(&m_biasGradientsAcc[preFixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector z(&instance_z[preFixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[preFixScan[layerIndex + 1]], outputCount);

	biasGradients.Sub(z, groundTruth);
	biasGradients.Mul(biasGradients, zDerivative);
	biasGradientsAcc.Add(biasGradientsAcc, biasGradients);

	const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[layerIndex]];
	const ndDeepBrainMemVector z0(&instance_z[preFixScan[layerIndex]], inputCount);
	for (ndInt32 i = 0; i < outputCount; ++i)
	{
		ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
		ndReal gValue = biasGradients[i];
		weightGradient.ScaleAdd(z0, gValue);
		weightGradientPtr += stride;
	}
}

void ndBrainTrainer::BackPropagateCalculateBiasGradient(ndInt32 layerIndex)
{
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	ndBrainLayer* const layer = layers[layerIndex + 1];
	const ndBrainPrefixScan& preFixScan = m_instance.GetPrefixScan();

	const ndDeepBrainMemVector biasGradients1(&m_biasGradients[preFixScan[layerIndex + 2]], layer->GetOuputSize());
	ndDeepBrainMemVector biasGradients(&m_biasGradients[preFixScan[layerIndex + 1]], layer->GetInputSize());
	ndDeepBrainMemVector biasGradientsAcc(&m_biasGradientsAcc[preFixScan[layerIndex + 1]], layer->GetInputSize());
	const ndBrainMatrix& matrix = *m_weightsLayersTranspose[layerIndex + 1];
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[preFixScan[layerIndex + 1]], layer->GetInputSize());

	matrix.Mul(biasGradients1, biasGradients);
	biasGradients.Mul(biasGradients, zDerivative);
	biasGradientsAcc.Add(biasGradientsAcc, biasGradients);
}

void ndBrainTrainer::BackPropagateHiddenLayer(ndInt32 layerIndex)
{
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	BackPropagateCalculateBiasGradient(layerIndex);

	ndBrainLayer* const layer = layers[layerIndex];
	const ndBrainVector& instance_z = m_instance.GetOutput();
	const ndBrainPrefixScan& preFixScan = m_instance.GetPrefixScan();

	const ndDeepBrainMemVector biasGradients(&m_biasGradients[preFixScan[layerIndex + 1]], layer->GetOuputSize());

	const ndInt32 inputCount = layer->GetInputSize();
	const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[layerIndex]];

	const ndDeepBrainMemVector z0(&instance_z[preFixScan[layerIndex]], inputCount);
	for (ndInt32 i = 0; i < layer->GetOuputSize(); ++i)
	{
		ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
		ndReal gValue = biasGradients[i];
		weightGradient.ScaleAdd(z0, gValue);
		weightGradientPtr += stride;
	}
}

void ndBrainTrainer::ApplyAdamCorrection()
{
	ndReal beta0 = m_beta;
	ndReal alpha0 = m_alpha;

	ndReal beta1 = 1.0f - m_beta;
	ndReal alpha1 = 1.0f - m_alpha;

	for (ndInt32 i = 0; i < m_biasGradientsAcc.GetCount(); ++i)
	{
		m_biasGradient_u[i] = m_biasGradient_u[i] * alpha0 + m_biasGradientsAcc[i] * alpha1;
		m_biasGradient_v[i] = m_biasGradient_v[i] * beta0 + m_biasGradientsAcc[i] * m_biasGradientsAcc[i] * beta1;
	}

	for (ndInt32 i = 0; i < m_weightGradients.GetCount(); ++i)
	{
		m_weightGradient_u[i] = m_weightGradient_u[i] * alpha0 + m_weightGradients[i] * alpha1;
		m_weightGradient_v[i] = m_weightGradient_v[i] * beta0 + m_weightGradients[i] * m_weightGradients[i] * beta1;
	}

	m_betaAcc = m_betaAcc * m_beta;
	m_alphaAcc = m_alphaAcc * m_alpha;

	ndReal epsilon2 = m_epsilon * m_epsilon;
	ndReal alphaWeight = 1.0f / (1.0f - m_alphaAcc);
	ndReal betaWeight = 1.0f / (1.0f - m_betaAcc);

	for (ndInt32 i = 0; i < m_biasGradientsAcc.GetCount(); ++i)
	{
		ndReal bias_uHot = m_biasGradient_u[i] * alphaWeight;
		ndReal bias_vHot = m_biasGradient_v[i] * betaWeight;

		ndReal weight_uHot = m_weightGradient_u[i] * alphaWeight;
		ndReal weight_vHot = m_weightGradient_v[i] * betaWeight;

		ndReal bias_den = ndReal(1.0f / ndSqrt(bias_vHot + epsilon2));
		ndReal weight_den = ndReal (1.0f / ndSqrt(weight_vHot + epsilon2));
		m_biasGradientsAcc[i] = bias_uHot * bias_den;
		m_weightGradients[i] = weight_uHot * weight_den;
	}
}

void ndBrainTrainer::UpdateWeights(ndReal learnRate, ndInt32 batchSize)
{
	ndReal regularizer = GetRegularizer();

	ndReal weight = 1.0f / batchSize;
	m_biasGradientsAcc.ScaleSet(m_biasGradientsAcc, weight);
	m_weightGradients.ScaleSet(m_weightGradients, weight);
	if (m_model == m_adam)
	{
		// apply adam optimizer
		ApplyAdamCorrection();
	}

	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = layers[i];
		const ndInt32 inputSize = layer->GetInputSize();
		const ndInt32 outputSize = layer->GetOuputSize();
		const ndBrainPrefixScan& preFixScan = m_instance.GetPrefixScan();
	
		ndBrainVector& bias = layer->GetBias();
		const ndDeepBrainMemVector biasGradients(&m_biasGradientsAcc[preFixScan[i + 1]], outputSize);
		bias.ScaleAdd(bias, -regularizer);
		bias.ScaleAdd(biasGradients, -learnRate);
	
		const ndInt32 weightGradientStride = (inputSize + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[i]];
	
		ndBrainMatrix& weightMatrix = *layer;
		for (ndInt32 j = 0; j < outputSize; ++j)
		{
			ndBrainVector& weightVector = weightMatrix[j];
			const ndDeepBrainMemVector weightGradients(weightGradientPtr, inputSize);
			weightVector.ScaleAdd(weightVector, -regularizer);
			weightVector.ScaleAdd(weightGradients, -learnRate);
			weightGradientPtr += weightGradientStride;
		}
	}
}

void ndBrainTrainer::ApplyWeightTranspose()
{
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 i = layers.GetCount() - 1; i >= 1; --i)
	{
		ndBrainLayer* const layer = layers[i];
		ndBrainMatrix& weightMatrix = *layer;
		m_weightsLayersTranspose[i]->SetTranspose(weightMatrix);
	}
}

void ndBrainTrainer::BackPropagate(const ndBrainVector& groundTruth)
{
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());

	BackPropagateOutputLayer(groundTruth);
	for (ndInt32 i = layers.GetCount() - 2; i >= 0; --i)
	{
		BackPropagateHiddenLayer(i);
	}
}

void ndBrainTrainer::ClearGradientsAcc()
{
	m_biasGradients.Set(0.0f);
	m_biasGradientsAcc.Set(0.0f);
	m_weightGradients.Set(0.0f);
}

void ndBrainTrainer::Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	ndFloatExceptions exception;
	ndAssert(inputBatch.GetCount() == groundTruth.GetCount());
	ndAssert(m_output.GetCount() == groundTruth[0].GetCount());
	
	ndBrain bestNetwork(*m_instance.GetBrain());
	ndArray<ndInt32> randomizeVector;
	randomizeVector.SetCount(inputBatch.GetCount());
	for (ndInt32 i = 0; i < inputBatch.GetCount(); ++i)
	{
		randomizeVector[i] = i;
	}

	const ndInt32 miniBatchSize = ndMin(m_miniBatchSize, inputBatch.GetCount());
	const ndInt32 batchCount = (inputBatch.GetCount() + miniBatchSize - 1) / miniBatchSize;

	m_bestCost = validator.Validate(inputBatch, groundTruth);
	for (ndInt32 i = 0; (i < steps) && (m_bestCost > 0.0f); ++i)
	{
		for (ndInt32 j = 0; j < batchCount; ++j)
		{
			ClearGradientsAcc();
			const ndInt32 start = j * miniBatchSize;
			const ndInt32 count = ((start + miniBatchSize) < inputBatch.GetCount()) ? miniBatchSize : inputBatch.GetCount() - start;
			for (ndInt32 k = 0; k < count; ++k)
			{
					ndInt32 index = randomizeVector[start + k];
					const ndBrainVector& input = inputBatch[index];
					const ndBrainVector& truth = groundTruth[index];
					MakePrediction(input);
					BackPropagate(truth);
			}
			UpdateWeights(learnRate, count);
		}
		ApplyWeightTranspose();
		randomizeVector.RandomShuffle(randomizeVector.GetCount());
		
		ndReal batchError = validator.Validate(inputBatch, groundTruth);
		if (batchError < m_bestCost)
		{
			m_bestCost = batchError;
			bestNetwork.CopyFrom(*m_instance.GetBrain());
		}
	}
	m_instance.GetBrain()->CopyFrom(bestNetwork);
}