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

ndDeepBrainParallelGradientDescendTrainingOperator::ndDeepBrainParallelGradientDescendTrainingOperator(ndDeepBrain* const brain, ndInt32 threads)
	:ndDeepBrainGradientDescendTrainingOperator(brain)
	,ndThreadPool("neuralNet")
	,m_inputBatch(nullptr)
	,m_groundTruth(nullptr)
	,m_learnRate(0.0f)
	,m_steps(0)
{
	threads = ndMin(threads, D_MAX_THREADS_COUNT);
	SetThreadCount(threads);
}

ndDeepBrainParallelGradientDescendTrainingOperator::~ndDeepBrainParallelGradientDescendTrainingOperator()
{
	Finish();
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

void ndDeepBrainParallelGradientDescendTrainingOperator::MakePredictionParallel(const ndDeepBrainVector& input)
{
	m_instance.MakePredictionParallel(*this, input, m_output);
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	
	const ndDeepBrainVector& output = m_instance.GetOutPut();
	const ndDeepBrainPrefixScan& zPrefixScan = m_instance.GetPrefixScan();
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		const ndDeepBrainMemVector z(&output[zPrefixScan[i + 1]], layer->GetOuputSize());
		ndDeepBrainMemVector zDerivative(&m_zDerivative[zPrefixScan[i + 1]], layer->GetOuputSize());
		layer->ActivationDerivative(z, zDerivative);
	}
}

void ndDeepBrainParallelGradientDescendTrainingOperator::BackPropagateOutputLayerParallel(const ndDeepBrainVector& groundTruth)
{
	auto BackPropagateOutputLayer = ndMakeObject::ndFunction([this, &groundTruth](ndInt32 threadIndex, ndInt32 threadCount)
	{
		const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
		const ndInt32 layerIndex = layers.GetCount() - 1;
		ndDeepBrainLayer* const ouputLayer = layers[layerIndex];

		const ndStartEnd startEnd(ouputLayer->GetOuputSize(), threadIndex, threadCount);
		const ndInt32 outputCount = startEnd.m_end - startEnd.m_start;
		if (outputCount)
		{
			const ndInt32 inputCount = ouputLayer->GetInputSize();
			ndDeepBrainMemVector biasGradients(&m_biasGradients[startEnd.m_start + m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
			const ndDeepBrainMemVector z(&m_instance.m_z[startEnd.m_start + m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
			const ndDeepBrainMemVector zDerivative(&m_zDerivative[startEnd.m_start + m_instance.m_zPrefixScan[layerIndex + 1]], outputCount);
			const ndDeepBrainMemVector truth(&groundTruth[startEnd.m_start], outputCount);

			biasGradients.Sub(z, truth);
			biasGradients.Mul(biasGradients, zDerivative);

			const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
			ndReal* weightGradientPtr = &m_weightGradients[startEnd.m_start * stride + m_weightGradientsPrefixScan[layerIndex]];
			const ndDeepBrainMemVector z0(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex]], inputCount);
			for (ndInt32 i = 0; i < outputCount; ++i)
			{
				ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
				ndFloat32 gValue = biasGradients[i];
				weightGradient.ScaleSet(z0, gValue);
				weightGradientPtr += stride;
			}
		}
	});
	ParallelExecute(BackPropagateOutputLayer);
}

void ndDeepBrainParallelGradientDescendTrainingOperator::BackPropagateHiddenLayerParallel(ndInt32 layerIndex)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	BackPropagateCalculateBiasGradient(layerIndex);

	ndDeepBrainLayer* const layer = layers[layerIndex];

	const ndDeepBrainMemVector biasGradients(&m_biasGradients[m_instance.m_zPrefixScan[layerIndex + 1]], layer->GetOuputSize());

	const ndInt32 inputCount = layer->GetInputSize();
	const ndInt32 stride = (inputCount + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[layerIndex]];

	const ndDeepBrainMemVector z0(&m_instance.m_z[m_instance.m_zPrefixScan[layerIndex]], inputCount);
	for (ndInt32 i = 0; i < layer->GetOuputSize(); ++i)
	{
		ndDeepBrainMemVector weightGradient(weightGradientPtr, inputCount);
		ndFloat32 gValue = biasGradients[i];
		weightGradient.ScaleSet(z0, gValue);
		weightGradientPtr += stride;
	}
}

void ndDeepBrainParallelGradientDescendTrainingOperator::BackPropagateParallel(const ndDeepBrainVector& groundTruth)
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	BackPropagateOutputLayerParallel(groundTruth);
	for (ndInt32 i = layers.GetCount() - 2; i >= 0; --i)
	{
		BackPropagateHiddenLayerParallel(i);
	}
}

void ndDeepBrainParallelGradientDescendTrainingOperator::Optimize()
{
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

	ndInt32 m_movingAverageIndex = 0;
	ndFloat32 m_movingAverageError = 0.0f;
	for (ndInt32 i = 0; i < m_steps; ++i)
	{
		const ndInt32 batchStart = index * m_miniBatchSize;
		const ndInt32 batchSize = index != (batchCount - 1) ? m_miniBatchSize : m_inputBatch->GetCount() - batchStart;
		index = (index + 1) % batchCount;

		m_averageError = 0.0f;
		for (ndInt32 j = 0; j < batchSize; ++j)
		{
			ndInt32 k = randomizeVector[batchStart + j];
			const ndDeepBrainVector& input = (*m_inputBatch)[k];
			const ndDeepBrainVector& truth = (*m_groundTruth)[k];
			//MakePredictionParallel(input);
			MakePrediction(input);
			//BackPropagateParallel(truth);
			BackPropagate(truth);
			UpdateWeights(m_learnRate);
			ndFloat32 error = CalculateMeanSquareError(truth);
			m_averageError += error;
		}
		ApplyWeightTranspose();
		m_movingAverageError += m_averageError;
		m_movingAverageIndex += batchSize;

		m_averageError = ndSqrt(m_averageError / batchSize);
		ndExpandTraceMessage("%f %d\n", m_averageError, i);

		index = (index + 1) % batchCount;
		if (index == 0)
		{
			randomizeVector.RandomShuffle();
			m_movingAverageError = ndSqrt(m_movingAverageError / m_movingAverageIndex);
			if (m_movingAverageError < bestCost)
			{
				bestCost = m_movingAverageError;
				bestNetwork.CopyFrom(*m_instance.GetBrain());
			}
			m_movingAverageIndex = 0;
			m_movingAverageError = 0.0f;
		}
	}
	m_instance.GetBrain()->CopyFrom(bestNetwork);
}