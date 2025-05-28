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
#include "ndBrainTrainerCpu.h"
#include "ndBrainThreadPool.h"
#include "ndBrainOptimizerAdamCpu.h"

ndBrainOptimizerAdamCpu::ndBrainOptimizerAdamCpu(ndBrainThreadPool* const threadPool, ndInt32 size)
	:ndBrainOptimizer()
	,m_vdw()
	,m_vdw2()
	,m_temp()
	,m_vdwCorrected()
	,m_vdw2Corrected()
	,m_beta(ndBrainFloat(0.999f))
	,m_alpha(ndBrainFloat(0.9f))
	,m_epsilon(1.0e-6f)
	,m_betaAcc(m_beta)
	,m_alphaAcc(m_alpha)
	,m_threadPool(threadPool)
	,m_miniBatchSize(256)
{
	m_vdw.SetCount(size);
	m_vdw2.SetCount(size);
	m_temp.SetCount(size);
	m_vdwCorrected.SetCount(size);
	m_vdw2Corrected.SetCount(size);

	m_vdw.Set(ndBrainFloat(0.0f));
	m_vdw2.Set(ndBrainFloat(0.0f));
	m_temp.Set(ndBrainFloat(0.0f));
	m_vdwCorrected.Set(ndBrainFloat(0.0f));
	m_vdw2Corrected.Set(ndBrainFloat(0.0f));
}

//#pragma optimize( "", off )
void ndBrainOptimizerAdamCpu::Update(ndBrainVector& parameters, const ndBrainVector& gradients, ndBrainFloat learnRate)
{
	ndAtomic<ndInt32> iterator(0);
	auto CalculateLayerGradients = ndMakeObject::ndFunction([this, learnRate, &gradients, &parameters, &iterator](ndInt32, ndInt32)
	{
		ndBrainFloat descendRate = -learnRate;
		ndBrainFloat regularizer = -GetRegularizer();
		//ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(partialGradients.GetCount());
		//ndBrainFloat den = ndBrainFloat(1.0f);

		const ndInt32 stride = 1024 * 4;
		const ndInt32 size = ndInt32(gradients.GetCount());
		const ndInt32 slices = (ndInt32(gradients.GetCount() + stride - 1)) / stride;
		for (ndInt32 i = iterator++; i < slices; i = iterator++)
		{
			ndInt32 start = i * stride;
			ndInt32 end = ((start + stride) > size) ? size : start + stride;
			ndInt32 count = end - start;

			//ndAssert(m_data[i]);
			//ndAdamData& data = *m_data[i];
			//ndBrainLayer& gradients = *trainer->GetGradientLayer(i);
			//gradients.Scale(den);

			ndBrainMemVector grad(&gradients[start], count);
			ndBrainMemVector temp(&m_temp[start], count);
			ndBrainMemVector vdw(&m_vdw[start], count);
			ndBrainMemVector vdw2(&m_vdw2[start], count);
			ndBrainMemVector vdwCorrected(&m_vdwCorrected[start], count);
			ndBrainMemVector vdw2Corrected(&m_vdw2Corrected[start], count);

			temp.Set(grad);
			
			//// calculate moving average
			//data.m_vdw->Scale(m_alpha);
			//data.m_vdw->ScaleAdd(*(*data.m_temp), ndBrainFloat(1.0) - m_alpha);
			vdw.Scale(m_alpha);
			vdw.ScaleAdd(temp, ndBrainFloat(1.0f) - m_alpha);

			//// caluate RMS
			//data.m_vdw2->Scale(m_beta);
			//data.m_temp->Mul(*(*data.m_temp));
			//data.m_vdw2->ScaleAdd(*(*data.m_temp), ndBrainFloat(1.0) - m_beta);
			vdw2.Scale(m_beta);
			temp.Mul(temp);
			vdw2.ScaleAdd(temp, ndBrainFloat(1.0) - m_beta);

			//data.m_vdwCorrected->Set(*(*data.m_vdw));
			//data.m_vdw2Corrected->Set(*(*data.m_vdw2));
			vdwCorrected.Set(vdw);
			vdw2Corrected.Set(vdw2);
			if (m_alphaAcc > ndBrainFloat(0.0f))
			{
				//data.m_vdwCorrected->Scale(ndBrainFloat(1.0f / (1.0f - m_alphaAcc)));
				vdwCorrected.Scale(ndBrainFloat(1.0f / (1.0f - m_alphaAcc)));
			}
			
			if (m_betaAcc > ndBrainFloat(0.0f))
			{
				//data.m_vdw2Corrected->Scale(ndBrainFloat(1.0f / (1.0f - m_betaAcc)));
				vdw2Corrected.Scale(ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_betaAcc));
			}
			
			for (ndInt32 j = ndInt32(grad.GetCount() - 1); j >= 0; --j)
			{
				ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected[j])) + m_epsilon);
				grad[j] = vdwCorrected[j] * bias_den;
			}
			 
			//ndBrainLayer& weights = *trainer->GetWeightsLayer(i);
			ndBrainMemVector weights (&parameters[start], count);
			switch (m_regularizerType)
			{
				case m_lasso:
				{
					//gradients.AddReqularizerL1(weights, regularizer);
					ndBrainFloat negativeRegularizer = -regularizer;
					for (ndInt32 j = ndInt32(grad.GetCount()) - 1; j >= 0; --j)
					{
						ndBrainFloat b = grad[j];
						grad[j] += (b > ndFloat32(0.0f)) ? regularizer : negativeRegularizer;
					}
					break;
				}
			
				case m_ridge:
				{
					//gradients.AddReqularizerL2(weights, regularizer);
					grad.ScaleAdd(weights, regularizer);
					break;
				}
			
				case m_none:;
			}
			//weights.ScaleAdd(gradients, descendRate);
			weights.ScaleAdd(grad, descendRate);
			weights.FlushToZero();
		}
	});
	m_threadPool->ndBrainThreadPool::ParallelExecute(CalculateLayerGradients);
	
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
