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

class ndDeepBrainParallelGradientDescendTrainingOperator::ndGradientChannel : public ndDeepBrainTrainingOperator
{
	public:
	ndGradientChannel(ndDeepBrainParallelGradientDescendTrainingOperator* const owner)
		:ndDeepBrainTrainingOperator(owner->m_instance.GetBrain())
		,m_g(owner->m_g)
		,m_output(owner->m_output)
		,m_zDerivative(owner->m_zDerivative)
		,m_weightGradients(owner->m_weightGradients)
		,m_owner(owner)
	{
	}

	virtual ~ndGradientChannel()
	{
	}

	void MakePrediction(const ndDeepBrainVector& input)
	{
		m_instance.MakePrediction(input, m_output);
		const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());

		const ndDeepBrainVector& z = m_instance.GetOutPut();
		const ndDeepBrainPrefixScan& zPrefixScan = m_instance.GetPrefixScan();
		for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
		{
			ndDeepBrainLayer* const layer = layers[i];
			const ndDeepBrainMemVector z(&z[zPrefixScan[i + 1]], layer->GetOuputSize());
			ndDeepBrainMemVector zDerivative(&m_zDerivative[zPrefixScan[i + 1]], layer->GetOuputSize());
			layer->ActivationDerivative(z, zDerivative);
		}
	}

	void BackPropagateOutputLayer(const ndDeepBrainVector& groundTruth)
	{

	}

	void BackPropagateHiddenLayer(ndInt32 layerIndex)
	{

	}

	void BackPropagate(const ndDeepBrainVector& groundTruth)
	{
		BackPropagateOutputLayer(groundTruth);

		const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
		for (ndInt32 i = layers.GetCount() - 2; i >= 0; --i)
		{
			BackPropagateHiddenLayer(i);
		}
	}

	virtual void Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
	{
		ndAssert(0);
	}

	ndDeepBrainVector m_g;
	ndDeepBrainVector m_output;
	ndDeepBrainVector m_zDerivative;
	ndDeepBrainVector m_weightGradients;

	ndDeepBrainParallelGradientDescendTrainingOperator* m_owner;
};

ndDeepBrainParallelGradientDescendTrainingOperator::ndDeepBrainParallelGradientDescendTrainingOperator(ndDeepBrain* const brain, ndInt32 threads)
	:ndDeepBrainGradientDescendTrainingOperator(brain)
	,ndThreadPool("neuralNet")
	,m_inputBatch(nullptr)
	,m_groundTruth(nullptr)
	,m_learnRate(0.0f)
	,m_steps(0)
{
	SetThreadCount(threads);
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
	ndThreadPool::SetThreadCount(threads);

	if (threads != m_subBatch.GetCount())
	{
		for (ndInt32 i = 0; i < m_subBatch.GetCount(); ++i)
		{
			delete m_subBatch[i];
		}
		m_subBatch.SetCount(0);

		for (ndInt32 i = 0; i < threads; ++i)
		{
			m_subBatch.PushBack(new ndGradientChannel(this));
		}
	}
}

void ndDeepBrainParallelGradientDescendTrainingOperator::ThreadFunction()
{
	Begin();
	//Optimize();
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
	ndAssert(m_inputBatch->GetCount() == m_groundTruth->GetCount());
	ndAssert(m_output.GetCount() == (*m_groundTruth)[0].GetCount());

	ndInt32 index = 0;
	ndInt32 batchCount = (m_inputBatch->GetCount() + m_miniBatchSize - 1) / m_miniBatchSize;
	for (ndInt32 i = 0; i < m_steps; ++i)
	{
		auto OptimizeMiniBatch = ndMakeObject::ndFunction([this, index, batchCount](ndInt32 threadIndex, ndInt32 threadCount)
		{
			const ndInt32 batchStart = index * m_miniBatchSize;
			const ndInt32 batchSize = index != (batchCount - 1) ? m_miniBatchSize : m_inputBatch->GetCount() - batchStart;

			ndGradientChannel* const channel = m_subBatch[threadIndex];

			const ndStartEnd startEnd(batchSize, threadIndex, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				const ndDeepBrainVector& input = (*m_inputBatch)[batchStart + i];
				const ndDeepBrainVector& truth = (*m_groundTruth)[batchStart + i];
				//MakePrediction(input);
				channel->MakePrediction(input);
				//BackPropagate(truth);
				channel->BackPropagate(truth);
				//UpdateWeights(learnRate);
				//ndFloat32 error = CalculateMeanSquareError(truth);
				//m_averageError += error;

			}
		});

		ParallelExecute(OptimizeMiniBatch);
		index = (index + 1) % batchCount;

		//m_averageError = 0.0f;
		//for (ndInt32 j = 0; j < batchSize; ++j)
		//{
		//	const ndDeepBrainVector& input = inputBatch[batchStart + j];
		//	const ndDeepBrainVector& truth = groundTruth[batchStart + j];
		//	MakePrediction(input);
		//	BackPropagate(truth);
		//	UpdateWeights(learnRate);
		//	ndFloat32 error = CalculateMeanSquareError(truth);
		//	m_averageError += error;
		//}
		//ApplyWeightTranspose();
		//m_averageError = ndSqrt(m_averageError / batchSize);
		ndExpandTraceMessage("%f\n", m_averageError);
	}
}