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
#include "ndBrainThreadPool.h"
#include "ndBrainOptimizerAdam.h"

class ndBrainOptimizerAdam::ndBrainOptimizerAdam::ndAdamData : public ndClassAlloc
{
	public:
	ndAdamData(ndBrainLayer* const layer)
		:ndClassAlloc()
		,m_u(nullptr)
		,m_v(nullptr)
		,m_v2(nullptr)
	{
		if (layer->HasParameters())
		{
			m_u = layer->Clone();
			m_v = layer->Clone();
			m_v2 = layer->Clone();

			m_u->Clear();
			m_v->Clear();
			m_v2->Clear();
		}
	}

	~ndAdamData()
	{
		if (m_u)
		{
			delete m_u;
			delete m_v;
			delete m_v2;
		}
	}

	ndBrainLayer* m_u;
	ndBrainLayer* m_v;
	ndBrainLayer* m_v2;
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
	ndBrainTrainer* const trainer = partialGradients[0];
	ndBrain& brain = *trainer->GetBrain();

	if (!m_initalized)
	{
		m_initalized = true;
		for (ndInt32 i = 0; i < brain.GetCount(); ++i)
		{
			m_data.PushBack(new ndAdamData(brain[i]));
		}
	}

#if 0
	ndFixSizeArray<ndInt32, 256> paramLayer;
	for (ndInt32 i = 0; i < brain.GetCount(); ++i)
	{
		if (brain[i]->HasParameters())
		{
			paramLayer.PushBack(i);
		}
	}

	auto UpdateGradients = ndMakeObject::ndFunction([this, learnRate, &paramLayer, &partialGradients](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer* const trainer = partialGradients[0];
		const ndStartEnd startEnd(paramLayer.GetCount(), threadIndex, threadCount);

		ndBrainFloat betaWeight = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_betaAcc);
		ndBrainFloat alphaWeight = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_alphaAcc);

		ndBrainFloat regularizer = -GetRegularizer();
		ndBrainFloat descendRate = learnRate * ndBrainFloat(-1.0f);
		ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(partialGradients.GetCount());

		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = paramLayer[i];
			ndAssert((*trainer->GetBrain())[index]->HasParameters());

			ndBrainLayer& weights = *trainer->GetWeightsLayer(index);
			ndBrainLayer& gradients = *trainer->GetGradientLayer(index);

			for (ndInt32 j = 1; j < partialGradients.GetCount(); ++j)
			{
				const ndBrainTrainer* const src = partialGradients[j];
				trainer->AcculumateGradients(*src, index);
			}

			ndAdamData& data = *m_data[index];
			gradients.Scale(den);
			data.m_v->Scale(m_beta);
			data.m_u->Scale(m_alpha);
			data.m_v2->Set(gradients);
			data.m_v2->Mul(gradients);

			data.m_u->ScaleAdd(gradients, ndBrainFloat(1.0f) - m_alpha);
			data.m_v->ScaleAdd(*data.m_v2, ndBrainFloat(1.0f) - m_beta);

			if (m_betaAcc > ndBrainFloat(0.0f))
			{
 				ndBrainLayer& vHat = gradients;
				ndBrainLayer& uHat = *data.m_v2;
	
				uHat.Set(*data.m_u);
				vHat.Set(*data.m_v);
				vHat.Scale(betaWeight);
				uHat.Scale(alphaWeight);
				gradients.AdamUpdate(uHat, vHat, m_epsilon);
			}
			else
			{
				gradients.AdamUpdate(*data.m_u, *data.m_v, m_epsilon);
			}

			gradients.Scale(descendRate);
			gradients.ScaleAdd(weights, regularizer);
			weights.Add(gradients);
			weights.FlushToZero();
		}
	});

	threadPool->ndBrainThreadPool::ParallelExecute(UpdateGradients);
#else

	for (ndInt32 i = 1; i < partialGradients.GetCount(); ++i)
	{
		const ndBrainTrainer* const src = partialGradients[i];
		trainer->AddGradients(src);
	}

	ndBrainFloat betaWeight = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_betaAcc);
	ndBrainFloat alphaWeight = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_alphaAcc);

	ndBrainFloat regularizer = -GetRegularizer();
	ndBrainFloat descendRate = learnRate * ndBrainFloat(-1.0f);
	ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(partialGradients.GetCount());

	for (ndInt32 i = 0; i < brain.GetCount(); ++i)
	{
		if (brain[i]->HasParameters())
		{
			ndAssert(m_data[i]);
			ndAdamData& data = *m_data[i];
			ndBrainLayer& gradients = *trainer->GetGradientLayer(i);

			gradients.Scale(den);
			data.m_v->Scale(m_beta);
			data.m_u->Scale(m_alpha);
			data.m_v2->Set(gradients);
			data.m_v2->Mul(gradients);

			data.m_u->ScaleAdd(gradients, ndBrainFloat(1.0f) - m_alpha);
			data.m_v->ScaleAdd(*data.m_v2, ndBrainFloat(1.0f) - m_beta);

			#if 0
				if (m_betaAcc > ndBrainFloat(0.0f))
				{
					ndBrainLayer& vHat = gradients;
					ndBrainLayer& uHat = *data.m_v2;

					uHat.Set(*data.m_u);
					vHat.Set(*data.m_v);
					vHat.Scale(betaWeight);
					uHat.Scale(alphaWeight);
					gradients.AdamUpdate(uHat, vHat, m_epsilon);
				}
				else
				{
					gradients.AdamUpdate(*data.m_u, *data.m_v, m_epsilon);
				}
			#else
				ndBrainLayer& vHat = gradients;
				ndBrainLayer& uHat = *data.m_v2;

				uHat.Set(*data.m_u);
				vHat.Set(*data.m_v);
				vHat.Scale(betaWeight);
				uHat.Scale(alphaWeight);
				gradients.AdamUpdate(uHat, vHat, m_epsilon);
			#endif

			gradients.Scale(descendRate);

			ndBrainLayer& weights = *trainer->GetWeightsLayer(i);
			gradients.ScaleAdd(weights, regularizer);
			weights.Add(gradients);
			weights.FlushToZero();
		}
	}

#endif

	
	m_betaAcc = ndFlushToZero(m_betaAcc * m_beta);
	m_alphaAcc = ndFlushToZero(m_alphaAcc * m_alpha);
	if (m_betaAcc < ndBrainFloat(1.0e-6f))
	{
		m_betaAcc = ndBrainFloat(0.0f);
	}
}