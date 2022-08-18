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
#include "ndDeepBrainParallelGradientDescendTrainingOperator.h"

ndDeepBrainParallelGradientDescendTrainingOperator::ndDeepBrainParallelGradientDescendTrainingOperator(ndDeepBrain* const brain)
	:ndDeepBrainGradientDescendTrainingOperator(brain)
	,ndThreadPool("neuralNet")
	,m_inputBatch(nullptr)
	,m_groundTruth(nullptr)
	,m_learnRate(0.0f)
	,m_steps(0)
{
	SetThreadCount(1);
}

ndDeepBrainParallelGradientDescendTrainingOperator::~ndDeepBrainParallelGradientDescendTrainingOperator()
{
	Finish();
	for (ndInt32 i = 0; i < m_subBatch.GetCount(); ++i)
	{
		delete m_subBatch[i];
	}
}

void ndDeepBrainParallelGradientDescendTrainingOperator::SetThreadCount(ndInt32 threads)
{
	threads = ndMin(threads, D_MAX_THREADS_COUNT);
	if (threads != m_subBatch.GetCount())
	{
		for (ndInt32 i = 0; i < m_subBatch.GetCount(); ++i)
		{
			delete m_subBatch[i];
		}
		m_subBatch.SetCount(0);

		for (ndInt32 i = 0; i < threads; ++i)
		{
			m_subBatch.PushBack(new ndDeepBrainGradientDescendTrainingOperator(*this));
		}
	}
}

void ndDeepBrainParallelGradientDescendTrainingOperator::ThreadFunction()
{
	Begin();
	Optimize();
	End();
}

void ndDeepBrainParallelGradientDescendTrainingOperator::Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	m_inputBatch = &inputBatch;
	m_groundTruth = &groundTruth;
	m_learnRate = learnRate;
	m_steps = steps;
	TickOne();
	Sync();
}

void ndDeepBrainParallelGradientDescendTrainingOperator::Optimize()
{
	ndAssert(m_output.GetCount() == (*m_groundTruth)[0].GetCount());
	ndAssert(m_inputBatch->GetCount() == m_groundTruth->GetCount());

	for (ndInt32 i = 0; i < m_steps; ++i)
	{
		m_averageError = 0.0f;
		for (ndInt32 j = m_inputBatch->GetCount() - 1; j >= 0; --j)
		{
			const ndDeepBrainVector& input = (*m_inputBatch)[j];
			const ndDeepBrainVector& truth = (*m_groundTruth)[j];
			MakePrediction(input);
			BackPropagate(truth);
			UpdateWeights(m_learnRate);
			ndFloat32 error = CalculateMeanSquareError(truth);
			//ndTrace(("%d %f\n", j, m_averageError));
			m_averageError += error;
		}
		ApplyWeightTranspose();
		m_averageError = ndSqrt(m_averageError / m_inputBatch->GetCount());
		//ndTrace(("%f\n", m_averageError));
		ndExpandTraceMessage("%f\n", m_averageError);
	}
}