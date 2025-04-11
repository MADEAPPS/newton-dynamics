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
		,m_vdw(nullptr)
		,m_vdw2(nullptr)
		,m_temp(nullptr)
		,m_vdwCorrected(nullptr)
		,m_vdw2Corrected(nullptr)
	{
		if (layer->HasParameters())
		{
			m_vdw = ndSharedPtr<ndBrainLayer>(layer->Clone());
			m_vdw2 = ndSharedPtr<ndBrainLayer>(layer->Clone());
			m_temp = ndSharedPtr<ndBrainLayer>(layer->Clone());
			m_vdwCorrected = ndSharedPtr<ndBrainLayer>(layer->Clone());
			m_vdw2Corrected = ndSharedPtr<ndBrainLayer>(layer->Clone());

			m_vdw->Clear();
			m_vdw2->Clear();

		}
	}

	~ndAdamData()
	{
	}

	ndSharedPtr<ndBrainLayer> m_vdw;
	ndSharedPtr<ndBrainLayer> m_vdw2;
	ndSharedPtr<ndBrainLayer> m_temp;
	ndSharedPtr<ndBrainLayer> m_vdwCorrected;
	ndSharedPtr<ndBrainLayer> m_vdw2Corrected;
};

ndBrainOptimizerAdam::ndBrainOptimizerAdam()
	:ndBrainOptimizer()
	,m_beta(ndBrainFloat(0.999f))
	,m_alpha(ndBrainFloat(0.9f))
	,m_epsilon(1.0e-6f)
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

//#pragma optimize( "", off )
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

	for (ndInt32 i = 1; i < partialGradients.GetCount(); ++i)
	{
		const ndBrainTrainer* const src = partialGradients[i];
		trainer->AddGradients(src);
	}

#if 0
	ndBrainFloat descendRate = -learnRate;
	ndBrainFloat regularizer = -GetRegularizer();
	ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(partialGradients.GetCount());

	for (ndInt32 i = 0; i < brain.GetCount(); ++i)
	{
		if (brain[i]->HasParameters())
		{
			ndAssert(m_data[i]);
			ndAdamData& data = *m_data[i];
			ndBrainLayer& gradients = *trainer->GetGradientLayer(i);
			gradients.Scale(den);
			data.m_temp->Set(gradients);

			// calculate moving average
			data.m_vdw->Scale(m_alpha);
			data.m_vdw->ScaleAdd(*(*data.m_temp), ndBrainFloat(1.0) - m_alpha);

			// caluate RMS
			data.m_vdw2->Scale(m_beta);
			data.m_temp->Mul(*(*data.m_temp));
			data.m_vdw2->ScaleAdd(*(*data.m_temp), ndBrainFloat(1.0) - m_beta);

			data.m_vdwCorrected->Set(*(*data.m_vdw));
			data.m_vdw2Corrected->Set(*(*data.m_vdw2));
			if (m_alphaAcc > ndBrainFloat(0.0f))
			{
				data.m_vdwCorrected->Scale(ndBrainFloat(1.0f / (1.0f - m_alphaAcc)));
			}

			if (m_betaAcc > ndBrainFloat(0.0f))
			{
				data.m_vdw2Corrected->Scale(ndBrainFloat(1.0f / (1.0f - m_betaAcc)));
			}
		
			gradients.AdamUpdate(*(*data.m_vdwCorrected), *(*data.m_vdw2Corrected), m_epsilon);
			ndBrainLayer& weights = *trainer->GetWeightsLayer(i);
			//gradients.ScaleAdd(weights, regularizer);
			switch (m_regularizerType)
			{
				case m_Lasso:
					gradients.AddReqularizerL1(weights, regularizer);
					break;

				case m_Ridge:
					gradients.AddReqularizerL2(weights, regularizer);
					break;

				case m_None:;
			}
	
			weights.ScaleAdd(gradients, descendRate);
			weights.FlushToZero();
		}
	}

#else

	ndAtomic<ndInt32> iterator(0);
	auto CalculateLayerGradients = ndMakeObject::ndFunction([this, learnRate, &partialGradients, &brain, trainer, &iterator](ndInt32, ndInt32)
	{
		ndBrainFloat descendRate = -learnRate;
		ndBrainFloat regularizer = -GetRegularizer();
		ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(partialGradients.GetCount());

		for (ndInt32 i = iterator++; i < brain.GetCount(); i = iterator++)
		{
			if (brain[i]->HasParameters())
			{
				ndAssert(m_data[i]);
				ndAdamData& data = *m_data[i];
				ndBrainLayer& gradients = *trainer->GetGradientLayer(i);
				gradients.Scale(den);
				data.m_temp->Set(gradients);

				// calculate moving average
				data.m_vdw->Scale(m_alpha);
				data.m_vdw->ScaleAdd(*(*data.m_temp), ndBrainFloat(1.0) - m_alpha);

				// caluate RMS
				data.m_vdw2->Scale(m_beta);
				data.m_temp->Mul(*(*data.m_temp));
				data.m_vdw2->ScaleAdd(*(*data.m_temp), ndBrainFloat(1.0) - m_beta);

				data.m_vdwCorrected->Set(*(*data.m_vdw));
				data.m_vdw2Corrected->Set(*(*data.m_vdw2));
				if (m_alphaAcc > ndBrainFloat(0.0f))
				{
					data.m_vdwCorrected->Scale(ndBrainFloat(1.0f / (1.0f - m_alphaAcc)));
				}

				if (m_betaAcc > ndBrainFloat(0.0f))
				{
					data.m_vdw2Corrected->Scale(ndBrainFloat(1.0f / (1.0f - m_betaAcc)));
				}

				gradients.AdamUpdate(*(*data.m_vdwCorrected), *(*data.m_vdw2Corrected), m_epsilon);
				ndBrainLayer& weights = *trainer->GetWeightsLayer(i);
				//gradients.ScaleAdd(weights, regularizer);
				switch (m_regularizerType)
				{
					case m_Lasso:
						gradients.AddReqularizerL1(weights, regularizer);
						break;

					case m_Ridge:
						gradients.AddReqularizerL2(weights, regularizer);
						break;

					case m_None:;
				}

				weights.ScaleAdd(gradients, descendRate);
				weights.FlushToZero();
			}
		}
	});
	threadPool->ndBrainThreadPool::ParallelExecute(CalculateLayerGradients);
#endif

	m_betaAcc = ndFlushToZero(m_betaAcc * m_beta);
	m_alphaAcc = ndFlushToZero(m_alphaAcc * m_alpha);
	if (m_betaAcc < ndBrainFloat(1.0e-6f))
	{
		m_betaAcc = ndBrainFloat(0.0f);
	}
	if (m_alphaAcc < ndBrainFloat(1.0e-6f))
	{
		m_alphaAcc = ndBrainFloat(0.0f);
	}
}

