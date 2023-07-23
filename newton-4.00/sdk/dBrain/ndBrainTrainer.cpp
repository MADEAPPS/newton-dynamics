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
#include "ndBrainLoss.h"
#include "ndBrainLayer.h"
#include "ndBrainTrainer.h"

ndBrainTrainer::ndBrainTrainer(ndBrain* const brain)
	:ndClassAlloc()
	,m_z()
	,m_zDerivative()
	,m_biasGradients()
	,m_weightGradients()
	,m_biasGradientsAcc()
	,m_biasGradient_u()
	,m_biasGradient_v()
	,m_weightGradient_u()
	,m_weightGradient_v()
	,m_weightGradientsPrefixScan()
	,m_brain(brain)
	,m_regularizer(1.0e-6f)
	,m_alpha(0.9f)
	,m_beta(0.999f)
	,m_epsilon(1.0e-8f)
	,m_alphaAcc(m_alpha)
	,m_betaAcc(m_beta)
	,m_model(m_adam)
{
	ndAssert(m_regularizer >= 0.0f);
	PrefixScan();
}

ndBrainTrainer::ndBrainTrainer(const ndBrainTrainer& src)
	:ndClassAlloc()
	,m_z(src.m_z)
	,m_zDerivative(src.m_zDerivative)
	,m_biasGradients(src.m_biasGradients)
	,m_weightGradients(src.m_weightGradients)
	,m_biasGradientsAcc(src.m_biasGradientsAcc)
	,m_biasGradient_u(src.m_biasGradient_u)
	,m_biasGradient_v(src.m_biasGradient_v)
	,m_weightGradient_u(src.m_weightGradient_u)
	,m_weightGradient_v(src.m_weightGradient_v)
	,m_weightGradientsPrefixScan(src.m_weightGradientsPrefixScan)
	,m_brain(src.m_brain)
	,m_regularizer(src.m_regularizer)
	,m_alpha(src.m_alpha)
	,m_beta(src.m_beta)
	,m_epsilon(src.m_epsilon)
	,m_alphaAcc(src.m_alphaAcc)
	,m_betaAcc(src.m_betaAcc)
	,m_model(src.m_model)
{
	ndAssert(0);
}

ndBrainTrainer::~ndBrainTrainer()
{
}

ndBrain* ndBrainTrainer::GetBrain() const
{
	return m_brain;
}

ndReal ndBrainTrainer::GetRegularizer() const
{
	return m_regularizer;
}

void ndBrainTrainer::SetRegularizer(ndReal regularizer)
{
	m_regularizer = ndClamp(regularizer, ndReal(0.0f), ndReal(0.01f));
}

void ndBrainTrainer::PrefixScan()
{
	const ndArray<ndBrainLayer*>& layers = *m_brain;
	const ndHidenVariableOffsets& preFixScan = m_brain->m_offsets;

	ndInt32 size = preFixScan[preFixScan.GetCount()-1];
	m_z.SetCount(size);
	m_zDerivative.SetCount(size);
	m_biasGradients.SetCount(size);
	m_biasGradientsAcc.SetCount(size);
	m_biasGradient_u.SetCount(size);
	m_biasGradient_v.SetCount(size);

	m_zDerivative.Set(0.0f);
	m_biasGradients.Set(0.0f);
	m_biasGradientsAcc.Set(0.0f);
	m_biasGradient_u.Set(0.0f);
	m_biasGradient_v.Set(0.0f);

	m_weightGradientsPrefixScan.SetCount(0);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndBrainLayer* const layer = layers[i];
		ndInt32 stride = (layer->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_weightGradientsPrefixScan.PushBack (stride * layer->GetOuputSize());
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_weightGradientsPrefixScan.GetCount(); ++i)
	{
		ndInt32 count = m_weightGradientsPrefixScan[i];
		m_weightGradientsPrefixScan[i] = sum;
		sum += count;
	}
	m_weightGradientsPrefixScan.PushBack(sum);
	m_weightGradients.SetCount(sum);
	m_weightGradient_u.SetCount(sum);
	m_weightGradient_v.SetCount(sum);

	m_weightGradients.Set(0.0f);
	m_weightGradient_u.Set(0.0f);
	m_weightGradient_v.Set(0.0f);
}

void ndBrainTrainer::ClearGradientsAcc()
{
	//m_biasGradients.Set(0.0f);
	m_biasGradientsAcc.Set(0.0f);
	m_weightGradients.Set(0.0f);
}

void ndBrainTrainer::AcculumateGradients(const ndBrainTrainer& src, ndInt32 threadIndex, ndInt32 threadCount)
{
	ndStartEnd biasStartEnd(m_biasGradientsAcc.GetCount(), threadIndex, threadCount);
	ndDeepBrainMemVector bias0(&m_biasGradientsAcc[biasStartEnd.m_start], biasStartEnd.m_end - biasStartEnd.m_start);
	const ndDeepBrainMemVector bias1(&src.m_biasGradientsAcc[biasStartEnd.m_start], biasStartEnd.m_end - biasStartEnd.m_start);
	bias0.Add(bias1);

	ndStartEnd weightStartEnd(m_weightGradients.GetCount(), threadIndex, threadCount);
	ndDeepBrainMemVector weigh0(&m_weightGradients[weightStartEnd.m_start], weightStartEnd.m_end - weightStartEnd.m_start);
	const ndDeepBrainMemVector weigh1(&src.m_weightGradients[weightStartEnd.m_start], weightStartEnd.m_end - weightStartEnd.m_start);
	weigh0.Add(weigh1);
}

void ndBrainTrainer::BackPropagateOutputLayer(ndBrainLoss& loss)
{
	const ndArray<ndBrainLayer*>& layers = *m_brain;
	const ndInt32 layerIndex = layers.GetCount() - 1;

	ndBrainLayer* const ouputLayer = layers[layerIndex];
	const ndInt32 inputCount = ouputLayer->GetInputSize();
	const ndInt32 outputCount = ouputLayer->GetOuputSize();
	const ndHidenVariableOffsets& preFixScan = m_brain->m_offsets;
	ndDeepBrainMemVector biasGradients(&m_biasGradients[preFixScan[layerIndex + 1]], outputCount);
	ndDeepBrainMemVector biasGradientsAcc(&m_biasGradientsAcc[preFixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector z(&m_z[preFixScan[layerIndex + 1]], outputCount);
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[preFixScan[layerIndex + 1]], outputCount);

	//biasGradients.Set(z);
	//biasGradients.Sub(groundTruth);
	loss.GetLoss(z, biasGradients);
	biasGradients.Mul(zDerivative);
	biasGradientsAcc.Add(biasGradients);

	const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[layerIndex]];
	const ndDeepBrainMemVector z0(&m_z[preFixScan[layerIndex]], inputCount);
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
	const ndArray<ndBrainLayer*>& layers = *m_brain;
	ndBrainLayer* const layer = layers[layerIndex + 1];
	const ndHidenVariableOffsets& preFixScan = m_brain->m_offsets;

	const ndDeepBrainMemVector biasGradients1(&m_biasGradients[preFixScan[layerIndex + 2]], layer->GetOuputSize());
	ndDeepBrainMemVector biasGradients(&m_biasGradients[preFixScan[layerIndex + 1]], layer->GetInputSize());
	ndDeepBrainMemVector biasGradientsAcc(&m_biasGradientsAcc[preFixScan[layerIndex + 1]], layer->GetInputSize());
	const ndDeepBrainMemVector zDerivative(&m_zDerivative[preFixScan[layerIndex + 1]], layer->GetInputSize());

	layer->TransposeMul(biasGradients1, biasGradients);
	biasGradients.Mul(zDerivative);
	biasGradientsAcc.Add(biasGradients);
}

void ndBrainTrainer::BackPropagateHiddenLayer(ndInt32 layerIndex)
{
	const ndArray<ndBrainLayer*>& layers = *m_brain;
	BackPropagateCalculateBiasGradient(layerIndex);

	ndBrainLayer* const layer = layers[layerIndex];
	const ndHidenVariableOffsets& preFixScan = m_brain->m_offsets;

	const ndDeepBrainMemVector biasGradients(&m_biasGradients[preFixScan[layerIndex + 1]], layer->GetOuputSize());

	const ndInt32 inputCount = layer->GetInputSize();
	const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[layerIndex]];

	const ndDeepBrainMemVector z0(&m_z[preFixScan[layerIndex]], inputCount);
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

	//for (ndInt32 i = 0; i < m_biasGradientsAcc.GetCount(); ++i)
	//{
	//	m_biasGradient_u[i] = m_biasGradient_u[i] * alpha0 + m_biasGradientsAcc[i] * alpha1;
	//	m_biasGradient_v[i] = m_biasGradient_v[i] * beta0 + m_biasGradientsAcc[i] * m_biasGradientsAcc[i] * beta1;
	//}
	//for (ndInt32 i = 0; i < m_weightGradients.GetCount(); ++i)
	//{
	//	m_weightGradient_u[i] = m_weightGradient_u[i] * alpha0 + m_weightGradients[i] * alpha1;
	//	m_weightGradient_v[i] = m_weightGradient_v[i] * beta0 + m_weightGradients[i] * m_weightGradients[i] * beta1;
	//}

	ndReal* const tempBuff = ndAlloca(ndReal, ndMax(m_biasGradientsAcc.GetCount(), m_weightGradients.GetCount()));
	ndDeepBrainMemVector temp(tempBuff, m_biasGradientsAcc.GetCount());

	temp.Set(m_biasGradientsAcc);
	temp.Mul(m_biasGradientsAcc);
	m_biasGradient_u.Scale(alpha0);
	m_biasGradient_u.ScaleAdd(m_biasGradientsAcc, alpha1);
	m_biasGradient_v.Scale(beta0);
	m_biasGradient_v.ScaleAdd(temp, beta1);

	temp.SetSize(m_weightGradients.GetCount());
	temp.Set(m_weightGradients);
	temp.Mul(m_weightGradients);
	m_weightGradient_u.Scale(alpha0);
	m_weightGradient_u.ScaleAdd(m_weightGradients, alpha1);
	m_weightGradient_v.Scale(beta0);
	m_weightGradient_v.ScaleAdd(temp, beta1);
	
	m_betaAcc = m_betaAcc * m_beta;
	m_alphaAcc = m_alphaAcc * m_alpha;

	ndReal epsilon2 = m_epsilon * m_epsilon;
	ndReal betaWeight = ndReal(1.0f) / (ndReal(1.0f) - m_betaAcc);
	ndReal alphaWeight = ndReal(1.0f) / (ndReal(1.0f) - m_alphaAcc);

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

	ndReal weight = 1.0f / ndReal(batchSize);
	m_biasGradientsAcc.Scale(weight);
	m_weightGradients.Scale(weight);
	if (m_model == m_adam)
	{
		// apply adam optimizer
		ApplyAdamCorrection();
	}

	const ndReal clampValue = ndReal(1.0e10f);
	const ndArray<ndBrainLayer*>& layers = *m_brain;
	const ndHidenVariableOffsets& preFixScan = m_brain->m_offsets;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = layers[i];
		const ndInt32 inputSize = layer->GetInputSize();
		const ndInt32 outputSize = layer->GetOuputSize();
	
		ndBrainVector& bias = layer->GetBias();
		const ndDeepBrainMemVector biasGradients(&m_biasGradientsAcc[preFixScan[i + 1]], outputSize);
		bias.ScaleAdd(bias, -regularizer);
		bias.ScaleAdd(biasGradients, -learnRate);
		bias.Clamp(-clampValue, clampValue);
	
		const ndInt32 weightGradientStride = (inputSize + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[i]];
	
		ndBrainMatrix& weightMatrix = *layer;
		for (ndInt32 j = 0; j < outputSize; ++j)
		{
			ndBrainVector& weightVector = weightMatrix[j];
			const ndDeepBrainMemVector weightGradients(weightGradientPtr, inputSize);
			weightVector.ScaleAdd(weightVector, -regularizer);
			weightVector.ScaleAdd(weightGradients, -learnRate);
			weightVector.Clamp(-clampValue, clampValue);
			weightGradientPtr += weightGradientStride;
		}
	}
}

void ndBrainTrainer::BackPropagate(const ndBrainVector& input, ndBrainLoss& loss)
{
	const ndArray<ndBrainLayer*>& layers = *m_brain;

	ndReal* const outputBuffer = ndAlloca(ndReal, layers[layers.GetCount() - 1]->GetOuputSize());
	ndDeepBrainMemVector output(outputBuffer, layers[layers.GetCount() - 1]->GetOuputSize());
	m_brain->MakePrediction(input, output, m_z);
	const ndHidenVariableOffsets& preFixScan = m_brain->m_offsets;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = layers[i];
		const ndDeepBrainMemVector z(&m_z[preFixScan[i + 1]], layer->GetOuputSize());
		ndDeepBrainMemVector zDerivative(&m_zDerivative[preFixScan[i + 1]], layer->GetOuputSize());
		layer->ActivationDerivative(z, zDerivative);
	}

	BackPropagateOutputLayer(loss);
	for (ndInt32 i = layers.GetCount() - 2; i >= 0; --i)
	{
		BackPropagateHiddenLayer(i);
	}
}