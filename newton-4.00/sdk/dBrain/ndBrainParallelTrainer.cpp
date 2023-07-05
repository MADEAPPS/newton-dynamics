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
#include "ndBrainLayer.h"
#include "ndBrainParallelTrainer.h"

class ndBrainParallelTrainer::ndBrainTrainerChannel : public ndBrainTrainer
{	
	public:
	ndBrainTrainerChannel(const ndBrainParallelTrainer& src)
		:ndBrainTrainer(src)
	{
		for (ndInt32 i = 0; i < m_weightsLayersTranspose.GetCount(); ++i)
		{
			delete m_weightsLayersTranspose[i];
			m_weightsLayersTranspose[i] = src.m_weightsLayersTranspose[i];
		}
	}

	~ndBrainTrainerChannel()
	{
		for (ndInt32 i = 0; i < m_weightsLayersTranspose.GetCount(); ++i)
		{
			m_weightsLayersTranspose[i] = nullptr;
		}
	}
};

//ndBrainParallelTrainer::ndBrainParallelTrainer(ndBrain* const brain, ndInt32 threads)
ndBrainParallelTrainer::ndBrainParallelTrainer(ndBrain* const brain, ndInt32)
	:ndBrainTrainer(brain)
	,ndThreadPool("neuralNet")
	,m_inputBatch(nullptr)
	,m_groundTruth(nullptr)
	,m_validator(nullptr)
	,m_learnRate(0.0f)
	,m_steps(0)
{
	ndAssert(0);
	//threads = ndMin(threads, D_MAX_THREADS_COUNT);
	//SetThreadCount(threads);
	//
	//for (ndInt32 i = 0; i < threads; i++)
	//{
	//	ndBrainTrainerChannel* const channel = new ndBrainTrainerChannel(*this);
	//	m_threadData.PushBack(channel);
	//}
}

ndBrainParallelTrainer::~ndBrainParallelTrainer()
{
	Finish();
	for (ndInt32 i = 0; i < GetThreadCount(); i++)
	{
		delete m_threadData[i];
	}
}

void ndBrainParallelTrainer::ThreadFunction()
{
	Begin();
	Optimize();
	End();
}

void ndBrainParallelTrainer::Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	m_steps = steps;
	m_learnRate = learnRate;
	m_inputBatch = &inputBatch;
	m_groundTruth = &groundTruth;
	m_validator = &validator;
	TickOne();
	Sync();
}

void ndBrainParallelTrainer::AverageWeights()
{
	const ndInt32 threads = m_threadData.GetCount();
	const ndReal weightFactor = 1.0f / ndReal(threads);
	
	const ndArray<ndBrainLayer*>& layers = (*m_instance.GetBrain());
	for (ndInt32 j = layers.GetCount() - 1; j >= 0; --j)
	{
		ndBrainLayer& layer = *layers[j];
		ndBrainVector& bias = layer.GetBias();
	
		layer.Set(0);
		bias.Set(0.0f);
		for (ndInt32 i = 0; i < threads; ++i)
		{
			const ndBrainLayer& srcLayer = *(*m_threadData[i]->GetBrain())[j];
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

ndReal ndBrainParallelTrainer::Validate(const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndBrainVector&)
{
	ndReal subBatchError2[D_MAX_THREADS_COUNT];
	auto Validate = ndMakeObject::ndFunction([this, &inputBatch, &groundTruth, &subBatchError2](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndReal errorAcc = 0.0f;
		ndBrainTrainer& optimizer = *m_threadData[threadIndex];

		const ndStartEnd startEnd(inputBatch.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			const ndBrainVector& input = inputBatch[i];
			const ndBrainVector& truth = groundTruth[i];
			optimizer.m_instance.MakePrediction(input, optimizer.m_output);
			optimizer.m_output.Sub(optimizer.m_output, truth);
			errorAcc += optimizer.m_output.Dot(optimizer.m_output);
		}
		subBatchError2[threadIndex] = errorAcc;
	});
	ParallelExecute(Validate);

	ndReal error2 = 0;
	for(ndInt32 i = 0; i < GetThreadCount(); ++i)
	{
		error2 += subBatchError2[i];
	}

	ndReal error = ndReal(ndSqrt(error2 / ndReal(inputBatch.GetCount())));
	return error;
}


void ndBrainParallelTrainer::Optimize()
{
	ndFloatExceptions exception;

	ndValidation& validator = *m_validator;
	const ndBrainMatrix& inputBatch = *m_inputBatch;
	const ndBrainMatrix& groundTruth = *m_groundTruth;

	ndAssert(inputBatch.GetCount() == groundTruth.GetCount());
	ndAssert(m_output.GetCount() == groundTruth[0].GetCount());

	ndBrain bestNetwork(*m_instance.GetBrain());

	ndArray<ndInt32> randomizeVector;
	randomizeVector.SetCount(inputBatch.GetCount());
	for (ndInt32 i = 0; i < inputBatch.GetCount(); ++i)
	{
		randomizeVector[i] = i;
	}

	const ndInt32 miniBatchSize = ndMin(m_miniBatchSize, inputBatch.GetCount());
	const ndInt32 batchCount = (inputBatch.GetCount() + miniBatchSize - 1) / miniBatchSize;

	ndAssert(0);
	//m_bestCost = validator.Validate(inputBatch, groundTruth);
	m_bestCost = validator.Validate(inputBatch);
	for (ndInt32 i = 0; (i < m_steps) && (m_bestCost > 0.0f); ++i)
	{
		for (ndInt32 j = 0; j < batchCount; ++j)
		{
			const ndInt32 batchStart = j * miniBatchSize;
			const ndInt32 batchSize = ((batchStart + miniBatchSize) < inputBatch.GetCount()) ? miniBatchSize : inputBatch.GetCount() - batchStart;

			auto CalculateGradients = ndMakeObject::ndFunction([this, batchStart, batchSize, &randomizeVector](ndInt32 threadIndex, ndInt32 threadCount)
			{
				ndBrainTrainer& optimizer = *m_threadData[threadIndex];

				optimizer.ClearGradientsAcc();
				const ndStartEnd startEnd(batchSize, threadIndex, threadCount);
				for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
				{
					ndInt32 k = randomizeVector[batchStart + i];
					const ndBrainVector& input = (*m_inputBatch)[k];
					const ndBrainVector& truth = (*m_groundTruth)[k];
					optimizer.MakePrediction(input);
					optimizer.BackPropagate(truth);
				}
			});
			ParallelExecute(CalculateGradients);

			ClearGradientsAcc();
			auto AddGradients = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
			{
				const ndStartEnd biasStartEnd(m_biasGradientsAcc.GetCount(), threadIndex, threadCount);
				const ndStartEnd weightStartEnd(m_weightGradients.GetCount(), threadIndex, threadCount);

				ndDeepBrainMemVector biasAcc(&m_biasGradientsAcc[biasStartEnd.m_start], biasStartEnd.m_end - biasStartEnd.m_start);
				ndDeepBrainMemVector weightAcc(&m_weightGradients[weightStartEnd.m_start], weightStartEnd.m_end - weightStartEnd.m_start);
				for (ndInt32 i = 0; i < threadCount; ++i)
				{
					const ndBrainTrainer& optimizer = *m_threadData[i];
					const ndDeepBrainMemVector biasSrc(&optimizer.m_biasGradientsAcc[biasStartEnd.m_start], biasStartEnd.m_end - biasStartEnd.m_start);
					const ndDeepBrainMemVector weightSrc(&optimizer.m_weightGradients[weightStartEnd.m_start], weightStartEnd.m_end - weightStartEnd.m_start);
					biasAcc.Add(biasAcc, biasSrc);
					weightAcc.Add(weightAcc, weightSrc);
				}
				threadIndex *= 1;
			});
			ParallelExecute(AddGradients);
			UpdateWeights(m_learnRate, batchSize);
		}

		ApplyWeightTranspose();
		randomizeVector.RandomShuffle(randomizeVector.GetCount());

		//ndReal batchError = validator.Validate(inputBatch, groundTruth);
		ndReal batchError = validator.Validate(inputBatch);
		if (batchError < m_bestCost)
		{
			m_bestCost = batchError;
			bestNetwork.CopyFrom(*m_instance.GetBrain());
		}
	}
	m_instance.GetBrain()->CopyFrom(bestNetwork);
}