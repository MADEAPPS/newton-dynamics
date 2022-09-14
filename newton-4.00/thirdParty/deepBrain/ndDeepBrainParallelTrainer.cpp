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

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrain.h"
#include "ndDeepBrainLayer.h"
#include "ndDeepBrainParallelTrainer.h"

//ndDeepBrainParallelTrainer::LocalData::LocalData(const ndDeepBrainTrainer& src)
//	//:ndDeepBrain(*src.GetBrain())
//	:ndDeepBrainTrainer((ndDeepBrain*)this)
//	//,m_averageError(0.0f)
//{
//}

//void ndDeepBrainParallelTrainer::LocalData::CopyTranspose(const ndArray<ndDeepBrainMatrix*>& src)
//{
//	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
//	for (ndInt32 i = layers.GetCount() - 1; i >= 1; --i)
//	{
//		ndDeepBrainMatrix& transposeMatrix = *m_weightsLayersTranspose[i];
//		transposeMatrix.Set(*src[i]);
//	}
//}

ndDeepBrainParallelTrainer::ndDeepBrainParallelTrainer(ndDeepBrain* const brain, ndInt32 threads)
	:ndDeepBrainTrainer(brain)
	,ndThreadPool("neuralNet")
	,m_inputBatch(nullptr)
	,m_groundTruth(nullptr)
	,m_validator(nullptr)
	,m_learnRate(0.0f)
	,m_steps(0)
{
	threads = ndMin(threads, D_MAX_THREADS_COUNT);
	SetThreadCount(threads);

	for (ndInt32 i = 0; i < threads; i++)
	{
		//m_threadData.PushBack(new LocalData(*this));
		m_threadData.PushBack(new ndDeepBrainTrainer(*this));
	}
}

ndDeepBrainParallelTrainer::~ndDeepBrainParallelTrainer()
{
	Finish();
	for (ndInt32 i = 0; i < GetThreadCount(); i++)
	{
		delete m_threadData[i];
	}
}

void ndDeepBrainParallelTrainer::ThreadFunction()
{
	Begin();
	Optimize();
	End();
}

void ndDeepBrainParallelTrainer::Optimize(ndValidation& validator, const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	m_steps = steps;
	m_learnRate = learnRate;
	m_inputBatch = &inputBatch;
	m_groundTruth = &groundTruth;
	m_validator = &validator;
	TickOne();
	Sync();
}

void ndDeepBrainParallelTrainer::AverageWeights()
{
	const ndInt32 threads = m_threadData.GetCount();
	const ndReal weightFactor = 1.0f / threads;
	
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 j = layers.GetCount() - 1; j >= 0; --j)
	{
		ndDeepBrainLayer& layer = *layers[j];
		ndDeepBrainVector& bias = layer.GetBias();
	
		layer.Set(0);
		bias.Set(0.0f);
		for (ndInt32 i = 0; i < threads; ++i)
		{
			const ndDeepBrainLayer& srcLayer = *(*m_threadData[i]->GetBrain())[j];
			bias.Add(bias, srcLayer.GetBias());
			for (ndInt32 k = 0; k < layer.GetOuputSize(); k++)
			{
				layer[k].Add(layer[k], srcLayer[k]);
			}
		}
		
		layer.GetBias().ScaleSet(layer.GetBias(), weightFactor);
		for (ndInt32 k = 0; k < layer.GetOuputSize(); k++)
		{
			layer[k].ScaleSet(layer[k], weightFactor);
		}
	}
}

void ndDeepBrainParallelTrainer::Optimize()
{
/*
	ndAssert(m_inputBatch->GetCount() == m_groundTruth->GetCount());
	ndAssert(m_output.GetCount() == (*m_groundTruth)[0].GetCount());
	
	ndDeepBrain bestNetwork(*m_instance.GetBrain());
	ndReal bestCost = 1.0e10f;
	
	ndInt32 index = 0;
	ndInt32 batchCount = (m_inputBatch->GetCount() + m_miniBatchSize - 1) / m_miniBatchSize;
	ndArray<ndInt32> randomizeVector;
	randomizeVector.SetCount(m_inputBatch->GetCount());
	for (ndInt32 i = 0; i < m_inputBatch->GetCount(); ++i)
	{
		randomizeVector[i] = i;
	}
	
	ndInt32 movingAverageIndex = 0;
	ndFloat32 movingAverageError = 0.0f;
	for (ndInt32 i = 0; i < m_steps; ++i)
	{
		const ndInt32 batchStart = index * m_miniBatchSize;
		const ndInt32 batchSize = index != (batchCount - 1) ? m_miniBatchSize : m_inputBatch->GetCount() - batchStart;
	
		auto CalculateGradients = ndMakeObject::ndFunction([this, batchStart, batchSize, &randomizeVector](ndInt32 threadIndex, ndInt32 threadCount)
		{
			ndAssert(0);
			//LocalData& optimizer = *m_threadData[threadIndex];
			//optimizer.m_averageError = 0.0f;
			//
			//const ndStartEnd startEnd(batchSize, threadIndex, threadCount);
			//for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			//{
			//	ndInt32 k = randomizeVector[batchStart + i];
			//	const ndDeepBrainVector& input = (*m_inputBatch)[k];
			//	const ndDeepBrainVector& truth = (*m_groundTruth)[k];
			//	optimizer.MakePrediction(input);
			//	optimizer.BackPropagate(truth);
			//	//optimizer.UpdateWeights(m_learnRate, 1);
			//	//optimizer.m_averageError += optimizer.CalculateMeanSquareError(truth);
			//}
		});
		ParallelExecute(CalculateGradients);
	
		//AverageWeights();
		//ApplyWeightTranspose();
		//
		//ndReal averageError = 0.0f;
		//for(ndInt32 j = GetThreadCount() - 1; j >= 0; --j)
		//{
		//	averageError += m_threadData[j]->m_averageError;
		//	m_threadData[j]->GetBrain()->CopyFrom(*GetBrain());
		//	m_threadData[j]->CopyTranspose(m_weightsLayersTranspose);
		//}
		//
		//movingAverageIndex += batchSize;
		//movingAverageError += averageError;
		//
		//averageError = ndSqrt(averageError / batchSize);
		//ndExpandTraceMessage("%f %d\n", averageError, i);
		//
		//index = (index + 1) % batchCount;
		//if (index == 0)
		//{
		//	randomizeVector.RandomShuffle(randomizeVector.GetCount());
		//	movingAverageError = ndSqrt(movingAverageError / movingAverageIndex);
		//	if (movingAverageError < bestCost)
		//	{
		//		bestCost = movingAverageError;
		//		bestNetwork.CopyFrom(*m_instance.GetBrain());
		//	}
		//	movingAverageIndex = 0;
		//	movingAverageError = 0.0f;
		//}
	}
	
	m_instance.GetBrain()->CopyFrom(bestNetwork);
*/

	ndFloatExceptions exception;

	ndValidation& validator = *m_validator;
	const ndDeepBrainMatrix& inputBatch = *m_inputBatch;
	const ndDeepBrainMatrix& groundTruth = *m_groundTruth;

	ndAssert(inputBatch.GetCount() == groundTruth.GetCount());
	ndAssert(m_output.GetCount() == groundTruth[0].GetCount());

	ndDeepBrain bestNetwork(*m_instance.GetBrain());
	ndArray<ndInt32> randomizeVector;
	randomizeVector.SetCount(inputBatch.GetCount());
	for (ndInt32 i = 0; i < inputBatch.GetCount(); ++i)
	{
		randomizeVector[i] = i;
	}

	const ndInt32 miniBatchSize = ndMin(m_miniBatchSize, inputBatch.GetCount());
	const ndInt32 batchCount = (inputBatch.GetCount() + miniBatchSize - 1) / miniBatchSize;

	m_bestCost = validator.Validate(inputBatch, groundTruth);
	for (ndInt32 i = 0; (i < m_steps) && (m_bestCost > 0.0f); ++i)
	{
		for (ndInt32 j = 0; j < batchCount; ++j)
		{
			const ndInt32 start = j * miniBatchSize;
			const ndInt32 count = ((start + miniBatchSize) < inputBatch.GetCount()) ? miniBatchSize : inputBatch.GetCount() - start;

			ClearGradientsAcc();
			for (ndInt32 k = 0; k < count; ++k)
			{
				ndInt32 index = randomizeVector[start + k];
				const ndDeepBrainVector& input = inputBatch[index];
				const ndDeepBrainVector& truth = groundTruth[index];
				MakePrediction(input);
				BackPropagate(truth);
			}

			UpdateWeights(m_learnRate, count);
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