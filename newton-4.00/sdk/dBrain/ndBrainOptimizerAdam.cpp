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

class ndBrainOptimizerAdam::ndBrainOptimizerAdam::ndAdamData : public ndClassAlloc
{
	public:
	ndAdamData(ndBrainLayer* const layer)
		:ndClassAlloc()
	{
		if (layer->HasParameters())
		{
			m_biasGradient_u.SetCount(layer->GetOutputSize());
			m_biasGradient_v.SetCount(layer->GetOutputSize());
			m_biasGradient_v2.SetCount(layer->GetOutputSize());
			m_weightGradient_u.Init(layer->GetOutputSize(), layer->GetInputSize());
			m_weightGradient_v.Init(layer->GetOutputSize(), layer->GetInputSize());
			m_weightGradient_v2.Init(layer->GetOutputSize(), layer->GetInputSize());

			m_biasGradient_u.Set(ndBrainFloat(0.0f));
			m_biasGradient_v.Set(ndBrainFloat(0.0f));
			m_biasGradient_v2.Set(ndBrainFloat(0.0f));
			m_weightGradient_u.Set(ndBrainFloat(0.0f));
			m_weightGradient_v.Set(ndBrainFloat(0.0f));
			m_weightGradient_v2.Set(ndBrainFloat(0.0f));
		}
	}

	ndBrainVector m_biasGradient_u;
	ndBrainVector m_biasGradient_v;
	ndBrainVector m_biasGradient_v2;
	ndBrainMatrix m_weightGradient_u;
	ndBrainMatrix m_weightGradient_v;
	ndBrainMatrix m_weightGradient_v2;
};

ndBrainOptimizerAdam::ndBrainOptimizerAdam()
	:ndBrainOptimizer()
	,m_beta(0.999f)
	,m_alpha(0.9f)
	,m_epsilon(1.0e-8f)
	,m_betaAcc(m_beta)
	,m_alphaAcc(m_alpha)
	,m_initalized(false)
{
	//ndBrain* const brain = trainer->GetBrain();
	//for (ndInt32 i = 0; i < brain->GetCount(); ++i)
	//{
	//	m_data.PushBack(new ndAdamData((*brain)[i]));
	//}
}

ndBrainOptimizerAdam::~ndBrainOptimizerAdam()
{
	for (ndInt32 i = 0; i < m_data.GetCount(); ++i)
	{
		delete (m_data[i]);
	}
}

void ndBrainOptimizerAdam::Update(ndBrainThreadPool* const threadPool, ndArray<ndBrainTrainer*>& partialGradients, ndBrainFloat learnRate)
{
	AccumulateGradients(threadPool, partialGradients);

	learnRate *= ndBrainFloat(-1.0f);
	ndBrainFloat regularizer = -GetRegularizer();
	ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(partialGradients.GetCount());

	ndBrainFloat betaWeight = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_betaAcc);
	ndBrainFloat alphaWeight = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_alphaAcc);

	ndBrainTrainer* const trainer = partialGradients[0];
	ndBrain* const brain = trainer->GetBrain();

	if (!m_initalized)
	{
		m_initalized = true;
		for (ndInt32 i = 0; i < brain->GetCount(); ++i)
		{
			m_data.PushBack(new ndAdamData((*brain)[i]));
		}
	}
	
	for (ndInt32 i = brain->GetCount() - 1; i >= 0 ; --i)
	{
		ndBrainLayer* const layer = (*brain)[i];
		if (layer->HasParameters())
		{
			ndAdamData& data = *m_data[i];
			ndBrainVector& bias = *trainer->GetBias(i);
			ndBrainMatrix& weight = *trainer->GetWeight(i);
			ndBrainVector& biasGradients = *trainer->GetBiasGradients(i);
			ndBrainMatrix& weightGradients = *trainer->GetWeightGradients(i);
	
			biasGradients.Scale(den);
			data.m_biasGradient_v.Scale(m_beta);
			data.m_biasGradient_u.Scale(m_alpha);
			
			data.m_biasGradient_v2.Set(biasGradients);
			data.m_biasGradient_v2.Mul(biasGradients);
			data.m_biasGradient_u.ScaleAdd(biasGradients, ndBrainFloat(1.0f) - m_alpha);
			data.m_biasGradient_v.ScaleAdd(data.m_biasGradient_v2, ndBrainFloat (1.0f) - m_beta);
			
			if (m_betaAcc > ndBrainFloat(0.0f))
			{
				ndBrainVector& vHat = biasGradients;
				ndBrainVector& uHat = data.m_biasGradient_v2;
	
				uHat.Set(data.m_biasGradient_u);
				vHat.Set(data.m_biasGradient_v);
				vHat.Scale(betaWeight);
				uHat.Scale(alphaWeight);
				for (ndInt32 j = biasGradients.GetCount() - 1; j >= 0; --j)
				{
					ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vHat[j])) + m_epsilon);
					biasGradients[j] = uHat[j] * bias_den;
				}
			}
			else
			{
				for (ndInt32 j = biasGradients.GetCount() - 1; j >= 0; --j)
				{
					ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(data.m_biasGradient_v[j])) + m_epsilon);
					biasGradients[j] = data.m_biasGradient_u[j] * bias_den;
				}
			}
			biasGradients.Scale(learnRate);
			biasGradients.ScaleAdd(bias, regularizer);
			bias.Add(biasGradients);
			bias.FlushToZero();
	
	
			weightGradients.Scale(den);
			data.m_weightGradient_v.Scale(m_beta);
			data.m_weightGradient_u.Scale(m_alpha);
			data.m_weightGradient_v2.Set(weightGradients);
			data.m_weightGradient_v2.Mul(weightGradients);
			data.m_weightGradient_v.ScaleAdd(data.m_weightGradient_v2, ndBrainFloat(1.0f) - m_beta);
			data.m_weightGradient_u.ScaleAdd(weightGradients, ndBrainFloat(1.0f) - m_alpha);
	
			if (m_betaAcc > ndBrainFloat(0.0f))
			{
				ndBrainMatrix& vHat = weightGradients;
				ndBrainMatrix& uHat = data.m_weightGradient_v2;
				
				uHat.Set(data.m_weightGradient_u);
				vHat.Set(data.m_weightGradient_v);
				vHat.Scale(betaWeight);
				uHat.Scale(alphaWeight);
	
				for (ndInt32 j = weightGradients.GetRows() - 1; j >= 0; --j)
				{
					for (ndInt32 k = weightGradients[j].GetCount() - 1; k >= 0; --k)
					{
						ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vHat[j][k])) + m_epsilon);
						weightGradients[j][k] = uHat[j][k] * bias_den;
					}
				}
			}
			else
			{
				for (ndInt32 j = weightGradients.GetRows() - 1; j >= 0; --j)
				{
					for (ndInt32 k = weightGradients[j].GetCount() - 1; k >= 0; --k)
					{
						ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(data.m_weightGradient_v[j][k])) + m_epsilon);
						weightGradients[j][k] = data.m_weightGradient_u[j][k] * bias_den;
					}
				}
			}
			weightGradients.Scale(learnRate);
			weightGradients.ScaleAdd(weight, regularizer);
			weight.Add(weightGradients);
			weight.FlushToZero();
		}
	}
	
	m_betaAcc = ndFlushToZero(m_betaAcc * m_beta);
	m_alphaAcc = ndFlushToZero(m_alphaAcc * m_alpha);
	if (m_betaAcc < ndBrainFloat(1.0e-6f))
	{
		m_betaAcc = ndBrainFloat(0.0f);
	}
}