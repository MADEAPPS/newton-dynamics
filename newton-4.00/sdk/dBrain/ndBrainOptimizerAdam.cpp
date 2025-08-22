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
#include "ndBrainCpuContext.h"
#include "ndBrainThreadPool.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainTrainerInference.h"

ndBrainOptimizerAdam::ndBrainOptimizerAdam(const ndSharedPtr<ndBrainContext>& context)
	:ndBrainOptimizer(context)
	,m_vdw()
	,m_vdw2()
	,m_parameters()
{
}

void ndBrainOptimizerAdam::Init(ndInt32 parametersBufferSizeInFloats)
{
	ndBrainVector buffer;
	buffer.SetCount(parametersBufferSizeInFloats);
	buffer.Set(ndReal(0.0f));

	m_parameters.m_decayRegularizer = GetRegularizer();
	m_vdw = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, buffer));
	m_vdw2 = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, buffer));
}

#if 0
void ndBrainOptimizerAdam::Update(ndBrainVector&, const ndBrainVector&)
{
	ndAtomic<ndInt32> iterator(0);
	auto CalculateLayerGradients = ndMakeObject::ndFunction([this, learnRate, &gradients, &parameters, &iterator](ndInt32, ndInt32)
	{
		ndBrainFloat descendRate = -learnRate;
		ndBrainFloat regularizer = -GetRegularizer();
	
		const ndInt32 stride = 1024 * 4;
		const ndInt32 size = ndInt32(gradients.GetCount());
		const ndInt32 slices = (ndInt32(gradients.GetCount() + stride - 1)) / stride;
		for (ndInt32 i = iterator++; i < slices; i = iterator++)
		{
			ndInt32 start = i * stride;
			ndInt32 end = ((start + stride) > size) ? size : start + stride;
			ndInt32 count = end - start;
	
			ndBrainMemVector grad(&gradients[start], count);
			ndBrainMemVector temp(&m_temp[start], count);
			ndBrainMemVector vdw(&m_vdw[start], count);
			ndBrainMemVector vdw2(&m_vdw2[start], count);
			ndBrainMemVector vdwCorrected(&m_vdwCorrected[start], count);
			ndBrainMemVector vdw2Corrected(&m_vdw2Corrected[start], count);
	
			temp.Set(grad);
			
			// calculate moving average
			vdw.Scale(m_alpha);
			vdw.ScaleAdd(temp, ndBrainFloat(1.0f) - m_alpha);
	
			// caluate RMS
			vdw2.Scale(m_beta);
			temp.Mul(temp);
			vdw2.ScaleAdd(temp, ndBrainFloat(1.0) - m_beta);
	
			vdwCorrected.Set(vdw);
			vdw2Corrected.Set(vdw2);
			if (m_alphaAcc > ndBrainFloat(0.0f))
			{
				vdwCorrected.Scale(ndBrainFloat(1.0f / (1.0f - m_alphaAcc)));
			}
			
			if (m_betaAcc > ndBrainFloat(0.0f))
			{
				vdw2Corrected.Scale(ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_betaAcc));
			}
			
			for (ndInt32 j = ndInt32(grad.GetCount() - 1); j >= 0; --j)
			{
				ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected[j])) + m_epsilon);
				grad[j] = vdwCorrected[j] * bias_den;
			}
			 
			ndBrainMemVector weights (&parameters[start], count);
			switch (m_regularizerType)
			{
				case m_lasso:
				{
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
					grad.ScaleAdd(weights, regularizer);
					break;
				}
			
				case m_none:;
			}
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
#endif

void ndBrainOptimizerAdam::ApplyLearnRate(ndBrainFloat learnRate)
{ 
	m_context->ApplyLeanRateCommands(*m_commands.GetFirst()->GetInfo(), learnRate);
	m_context->SubmitBufferCommand(*m_commands.GetFirst()->GetNext()->GetInfo());
}