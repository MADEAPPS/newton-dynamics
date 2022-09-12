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
#include "ndDeepBrainParallelTrainer_Experiment.h"

ndDeepBrainParallelTrainer_Experiment::ndDeepBrainParallelTrainer_Experiment(ndDeepBrain* const brain, ndReal regularizer, ndInt32 threads)
	:ndDeepBrainTrainer(brain, regularizer)
	,ndThreadPool("neuralNet")
	,m_inputBatch(nullptr)
	,m_groundTruth(nullptr)
	,m_learnRate(0.0f)
	,m_steps(0)
{
	threads = ndMin(threads, D_MAX_THREADS_COUNT);
	SetThreadCount(threads);
}

ndDeepBrainParallelTrainer_Experiment::~ndDeepBrainParallelTrainer_Experiment()
{
	Finish();
}

void ndDeepBrainParallelTrainer_Experiment::ThreadFunction()
{
	Begin();
	ndDeepBrainTrainer::Optimize(*m_inputBatch, *m_groundTruth, m_learnRate, m_steps);
	End();
}

void ndDeepBrainParallelTrainer_Experiment::Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	m_steps = steps;
	m_learnRate = learnRate;
	m_inputBatch = &inputBatch;
	m_groundTruth = &groundTruth;
	TickOne();
	Sync();
}

void ndDeepBrainParallelTrainer_Experiment::MakePrediction(const ndDeepBrainVector& input)
{
	//m_instance.MakePrediction(input, m_output);
	m_instance.MakePrediction(*this, input, m_output);
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());

	ndDeepBrainVector& instance_z = m_instance.GetOutPut();
	const ndDeepBrainPrefixScan& preFixScan = m_instance.GetPrefixScan();
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		const ndDeepBrainMemVector z(&instance_z[preFixScan[i + 1]], layer->GetOuputSize());
		ndDeepBrainMemVector zDerivative(&m_zDerivative[preFixScan[i + 1]], layer->GetOuputSize());
		layer->ActivationDerivative(z, zDerivative);
	}
}