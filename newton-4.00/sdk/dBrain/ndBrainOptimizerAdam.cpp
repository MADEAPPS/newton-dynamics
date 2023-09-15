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
#include "ndBrainMatrix.h"
#include "ndBrainTrainer.h"
#include "ndBrainOptimizerAdam.h"

class ndBrainOptimizerAdam::ndBrainOptimizerAdam::ndAdabData
{
	public:
	ndBrainVector m_biasGradient_u;
	ndBrainVector m_biasGradient_v;
	ndBrainVector m_weightGradient_u;
	ndBrainVector m_weightGradient_v;
};

ndBrainOptimizerAdam::ndBrainOptimizerAdam(ndBrainTrainer* const trainer)
	:ndBrainOptimizer(trainer)
	,m_beta(0.999f)
	,m_alpha(0.9f)
	,m_epsilon(1.0e-8f)
	,m_betaAcc(m_beta)
	,m_alphaAcc(m_alpha)
{
}

ndBrainOptimizerAdam::~ndBrainOptimizerAdam()
{
	for (ndInt32 i = 0; i < m_data.GetCount(); ++i)
	{
		delete (m_data[i]);
	}
}

void ndBrainOptimizerAdam::Update(ndReal learnRate, ndInt32 bashSize)
{
	learnRate *= ndReal(-1.0f);
	ndReal regularizer = -GetRegularizer();
	ndReal den = ndReal(1.0f) / ndReal(bashSize);

	ndBrain* const brian = m_trainer->GetBrain();
	for (ndInt32 i = 0; brian->GetCount(); ++i)
	{
		ndBrainLayer* const layer = (*brian)[i];
		if (layer->HasParameters())
		{
			ndBrainVector& bias = *m_trainer->GetBias(i);
			ndBrainMatrix& weight = *m_trainer->GetWeight(i);
			ndBrainVector& biasGradients = *m_trainer->GetBiasGradients(i);
			ndBrainMatrix& weightGradients = *m_trainer->GetWeightGradients(i);

			biasGradients.Scale(den);
			biasGradients.Scale(learnRate);
			biasGradients.ScaleAdd(bias, regularizer);
			bias.Add(biasGradients);
			bias.FlushToZero();

			weightGradients.Scale(den);
			weightGradients.Scale(learnRate);
			weightGradients.ScaleAdd(weight, regularizer);
			weight.Add(weightGradients);
			weight.FlushToZero();

			//ndReal betaWeight = ndReal(1.0f) / (ndReal(1.0f) - m_betaAcc);
			//ndReal alphaWeight = ndReal(1.0f) / (ndReal(1.0f) - m_alphaAcc);
			//
			//ndInt32 tempSize = (ndMax(m_biasGradientsAcc.GetCount(), m_weightGradients.GetCount()) + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
			//ndReal* const tempBuff = ndAlloca(ndReal, tempSize * 2);
			//{
			//	m_biasGradient_v.Scale(m_beta);
			//	m_biasGradient_u.Scale(m_alpha);
			//
			//	ndBrainMemVector biasGradientsAcc2(tempBuff, m_biasGradientsAcc.GetCount());
			//	biasGradientsAcc2.Set(m_biasGradientsAcc);
			//	biasGradientsAcc2.Mul(m_biasGradientsAcc);
			//	m_biasGradient_v.ScaleAdd(biasGradientsAcc2, ndReal (1.0f) - m_beta);
			//	m_biasGradient_u.ScaleAdd(m_biasGradientsAcc, ndReal(1.0f) - m_alpha);
			//
			//	if (m_betaAcc > ndReal(0.0f))
			//	{
			//		ndBrainMemVector uHat(tempBuff, m_biasGradientsAcc.GetCount());
			//		ndBrainMemVector vHat(tempBuff + tempSize, m_biasGradientsAcc.GetCount());
			//		uHat.Set(m_biasGradient_u);
			//		vHat.Set(m_biasGradient_v);
			//		vHat.Scale(betaWeight);
			//		uHat.Scale(alphaWeight);
			//		for (ndInt32 i = m_biasGradientsAcc.GetCount() - 1; i >= 0; --i)
			//		{
			//			ndReal bias_den = ndReal(1.0f) / (ndReal(ndSqrt(vHat[i])) + m_epsilon);
			//			m_biasGradientsAcc[i] = uHat[i] * bias_den;
			//		}
			//	}
			//	else
			//	{
			//		for (ndInt32 i = m_biasGradientsAcc.GetCount() - 1; i >= 0; --i)
			//		{
			//			ndReal bias_den = ndReal(1.0f) / (ndReal(ndSqrt(m_biasGradient_v[i])) + m_epsilon);
			//			m_biasGradientsAcc[i] = m_biasGradient_u[i] * bias_den;
			//		}
			//	}
			//}
			//
			//{
			//	m_weightGradient_v.Scale(m_beta);
			//	m_weightGradient_u.Scale(m_alpha);
			//	
			//	ndBrainMemVector weightGradients2(tempBuff, m_weightGradients.GetCount());
			//	weightGradients2.Set(m_weightGradients);
			//	weightGradients2.Mul(m_weightGradients);
			//	m_weightGradient_v.ScaleAdd(weightGradients2, ndReal(1.0f) - m_beta);
			//	m_weightGradient_u.ScaleAdd(m_weightGradients, ndReal(1.0f) - m_alpha);
			//	
			//	if (m_betaAcc > ndReal(0.0f))
			//	{
			//		ndBrainMemVector uHat(tempBuff, m_weightGradients.GetCount());
			//		ndBrainMemVector vHat(tempBuff + tempSize, m_weightGradients.GetCount());
			//		uHat.Set(m_weightGradient_u);
			//		vHat.Set(m_weightGradient_v);
			//		vHat.Scale(betaWeight);
			//		uHat.Scale(alphaWeight);
			//		 
			//		for (ndInt32 i = m_weightGradients.GetCount() - 1; i >= 0; --i)
			//		{
			//			ndReal bias_den = ndReal(1.0f) / (ndReal(ndSqrt(vHat[i])) + m_epsilon);
			//			m_weightGradients[i] = uHat[i] * bias_den;
			//		}
			//	}
			//	else
			//	{
			//		for (ndInt32 i = m_weightGradients.GetCount() - 1; i >= 0; --i)
			//		{
			//			ndReal bias_den = ndReal(1.0f) / (ndReal(ndSqrt(m_weightGradient_v[i])) + m_epsilon);
			//			m_weightGradients[i] = m_weightGradient_u[i] * bias_den;
			//		}
			//	}
			//}
			//
			//m_betaAcc = ndFlushToZero(m_betaAcc * m_beta);
			//m_alphaAcc = ndFlushToZero(m_alphaAcc * m_alpha);
			//if (m_betaAcc < ndReal(1.0e-6f))
			//{
			//	m_betaAcc = ndReal(0.0f);
			//}
			//StochasticUpdate(learnRate);
		}
	}
}