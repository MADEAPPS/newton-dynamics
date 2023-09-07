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

#ifndef _ND_BRAIN_AGENT_TD3_TRAINER_H__
#define _ND_BRAIN_AGENT_TD3_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"

// this is an implementation of more stable policy gradient for
// continues action controller using deep re enforcement learning. 
// td3 algorithm as described here: 
// https://arxiv.org/pdf/1802.09477.pdf

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentTD3_Trainer : public ndBrainAgent, public ndBrainThreadPool
{
	public:

	class HyperParameters
	{
		public:
		HyperParameters()
		{
			m_bashBufferSize = 64;
			m_discountFactor = ndReal(0.99f);
			m_regularizer = ndReal(2.0e-6f);
			m_actorLearnRate = ndReal(0.0001f);
			m_criticLearnRate = ndReal(0.001f);
			m_replayBufferSize = 1024 * 512;
			m_replayBufferPrefill = 1024 * 4;
			m_softTargetFactor = ndReal(1.0e-3f);
			m_actionNoiseVariance = ndReal(0.05f);
			m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize / 4);
		}

		ndReal m_discountFactor;
		ndReal m_regularizer;
		ndReal m_actorLearnRate;
		ndReal m_criticLearnRate;
		ndReal m_softTargetFactor;
		ndReal m_actionNoiseVariance;

		ndInt32 m_threadsCount;
		ndInt32 m_bashBufferSize;
		ndInt32 m_replayBufferSize;
		ndInt32 m_replayBufferPrefill;
	};

	ndBrainAgentTD3_Trainer(const HyperParameters& hyperParameters, const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic);
	~ndBrainAgentTD3_Trainer();

	ndReal GetCurrentValue() const;
	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;
	ndInt32 GetEpisodeFrames() const;

	ndReal GetLearnRate() const;
	void SetLearnRate(ndReal learnRate);

	ndReal GetActionNoise() const;
	void SetActionNoise(ndReal learnRate);

	protected:
	void Step();
	void Optimize();
	void OptimizeStep();
	bool IsTrainer() const;
	bool IsSampling() const;
	bool IsTerminal() const;
	ndReal GetReward() const;
	void SetBufferSize(ndInt32 size);
	ndReal PerturbeAction(ndReal action) const;
	void Save(ndBrainSave* const loadSave) const;

	void BackPropagate();
	void BackPropagateActor(const ndUnsigned32* const bashIndex);
	void BackPropagateCritic(const ndUnsigned32* const bashIndex);
	void CalculateQvalue(const ndBrainVector& state, const ndBrainVector& actions);

	void InitWeights();
	void InitWeights(ndReal weighVariance, ndReal biasVariance);

	ndBrain m_actor;
	ndBrain m_critic0;
	ndBrain m_critic1;

	ndBrain m_targetActor;
	ndBrain m_targetCritic0;
	ndBrain m_targetCritic1;

	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_actorOptimizer;
	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_criticOptimizer0;
	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_criticOptimizer1;

	ndArray<ndInt32> m_bashSamples;
	ndBrainReplayBuffer<ndReal, statesDim, actionDim> m_replayBuffer;
	ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim> m_currentTransition;

	ndReal m_discountFactor;
	ndReal m_currentQValue;
	ndReal m_actorLearnRate;
	ndReal m_criticLearnRate;
	ndReal m_softTargetFactor;
	ndReal m_actionNoiseVariance;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_replayBufferPrefill;
	bool m_collectingSamples;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentTD3_Trainer<statesDim, actionDim>::ndBrainAgentTD3_Trainer(const HyperParameters& hyperParameters, const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic)
	:ndBrainAgent()
	,ndBrainThreadPool()
	,m_actor(*(*actor))
	,m_critic0(*(*critic))
	,m_critic1(*(*critic))
	,m_targetActor(m_actor)
	,m_targetCritic0(m_critic0)
	,m_targetCritic1(m_critic1)
	,m_bashSamples()
	,m_replayBuffer()
	,m_currentTransition()
	,m_discountFactor(hyperParameters.m_discountFactor)
	,m_currentQValue(ndReal(0.0f))
	,m_actorLearnRate(hyperParameters.m_actorLearnRate)
	,m_criticLearnRate(hyperParameters.m_criticLearnRate)
	,m_softTargetFactor(hyperParameters.m_softTargetFactor)
	,m_actionNoiseVariance(hyperParameters.m_actionNoiseVariance)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_replayBufferPrefill(hyperParameters.m_replayBufferPrefill)
	,m_collectingSamples(true)
{
	ndAssert(0);
	//ndAssert(m_critic0.GetOutputSize() == 1);
	//ndAssert(m_critic1.GetOutputSize() == 1);
	//ndAssert((m_actor[m_actor.GetCount() - 1])->GetActivationType() == m_tanh);
	//ndAssert(m_critic0.GetInputSize() == (m_actor.GetInputSize() + m_actor.GetOutputSize()));
	//ndAssert(m_critic1.GetInputSize() == (m_actor.GetInputSize() + m_actor.GetOutputSize()));
	//
	//SetThreadCount(hyperParameters.m_threadsCount);
	//for (ndInt32 i = 0; i < ndBrainThreadPool::GetThreadCount(); ++i)
	//{
	//	m_actorOptimizer.PushBack(new ndBrainTrainer(&m_actor));
	//	m_criticOptimizer0.PushBack(new ndBrainTrainer(&m_critic0));
	//	m_criticOptimizer1.PushBack(new ndBrainTrainer(&m_critic1));
	//	m_actorOptimizer[m_actorOptimizer.GetCount() - 1]->SetRegularizer(hyperParameters.m_regularizer);
	//	m_criticOptimizer0[m_criticOptimizer0.GetCount() - 1]->SetRegularizer(hyperParameters.m_regularizer);
	//	m_criticOptimizer1[m_criticOptimizer1.GetCount() - 1]->SetRegularizer(hyperParameters.m_regularizer);
	//}
	//
	//SetBufferSize(hyperParameters.m_replayBufferSize);
	//InitWeights();
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentTD3_Trainer<statesDim, actionDim>::~ndBrainAgentTD3_Trainer()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave) const
{
	loadSave->Save(&m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::SetBufferSize(ndInt32 size)
{
	m_replayBuffer.SetSize(size);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentTD3_Trainer<statesDim, actionDim>::IsTrainer() const
{
	return true;
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentTD3_Trainer<statesDim, actionDim>::IsSampling() const
{
	return m_collectingSamples;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentTD3_Trainer<statesDim, actionDim>::GetCurrentValue() const
{
	return m_currentQValue;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentTD3_Trainer<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentTD3_Trainer<statesDim, actionDim>::GetEposideCount() const
{
	return m_eposideCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentTD3_Trainer<statesDim, actionDim>::GetEpisodeFrames() const
{
	return m_framesAlive;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentTD3_Trainer<statesDim, actionDim>::GetActionNoise() const
{
	return m_actionNoiseVariance;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::SetActionNoise(ndReal noiseVariance)
{
	m_actionNoiseVariance = noiseVariance;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::InitWeights()
{
	m_actor.InitWeightsXavierMethod();
	m_critic0.InitWeightsXavierMethod();
	m_critic1.InitWeightsXavierMethod();

	m_targetActor.CopyFrom(m_actor);
	m_targetCritic0.CopyFrom(m_critic0);
	m_targetCritic1.CopyFrom(m_critic1);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::InitWeights(ndReal weighVariance, ndReal biasVariance)
{
	m_actor.InitWeights(weighVariance, biasVariance);
	m_critic0.InitWeights(weighVariance, biasVariance);
	m_critic1.InitWeights(weighVariance, biasVariance);

	m_targetActor.CopyFrom(m_actor);
	m_targetCritic0.CopyFrom(m_critic0);
	m_targetCritic1.CopyFrom(m_critic1);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentTD3_Trainer<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentTD3_Trainer<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndReal(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentTD3_Trainer<statesDim, actionDim>::PerturbeAction(ndReal action) const
{
	ndReal actionNoise = ndReal(ndGaussianRandom(action, m_actionNoiseVariance));
	return ndClamp(actionNoise, ndReal(-1.0f), ndReal(1.0f));
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::BackPropagateCritic(const ndUnsigned32* const shuffleBuffer)
{
	auto PropagateBash = ndMakeObject::ndFunction([this, &shuffleBuffer](ndInt32 threadIndex, ndInt32 threadCount)
	{
		class Loss : public ndBrainLeastSquareErrorLoss
		{
			public:
			Loss(ndBrainTrainer& trainer, ndBrain* const targetCritic, const ndBrainReplayBuffer<ndReal, statesDim, actionDim>& replayBuffer)
				:ndBrainLeastSquareErrorLoss(trainer.GetBrain()->GetOutputSize())
				,m_criticTrainer(trainer)
				,m_targetCritic(targetCritic)
				,m_replayBuffer(replayBuffer)
				,m_reward(0.0f)
			{
			}

			void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(loss.GetCount() == 1);
				ndAssert(output.GetCount() == 1);
				ndAssert(m_truth.GetCount() == m_criticTrainer.GetBrain()->GetOutputSize());

				ndReal criticOutputBuffer[2];
				ndBrainMemVector criticOutput(criticOutputBuffer, 1);

				criticOutput[0] = m_reward;
				SetTruth(criticOutput);
				ndBrainLeastSquareErrorLoss::GetLoss(output, loss);
			}

			ndBrainTrainer& m_criticTrainer;
			ndBrain* m_targetCritic;
			const ndBrainReplayBuffer<ndReal, statesDim, actionDim>& m_replayBuffer;
			ndFloat32 m_reward;
		};

		ndBrainTrainer& trainer0 = *(*m_criticOptimizer0[threadIndex]);
		ndBrainTrainer& trainer1 = *(*m_criticOptimizer1[threadIndex]);
		trainer0.ClearGradientsAcc();
		trainer1.ClearGradientsAcc();

		Loss loss0(trainer0, &m_targetCritic0, m_replayBuffer);
		Loss loss1(trainer1, &m_targetCritic1, m_replayBuffer);

		const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);

		ndReal criticOutputBuffer[2];
		ndReal targetInputBuffer[statesDim * 2];
		ndReal targetOutputBuffer[actionDim * 2];
		ndReal criticInputBuffer[(statesDim + actionDim) * 2];

		ndBrainMemVector criticOutput(criticOutputBuffer, 1);
		ndBrainMemVector targetInput(targetInputBuffer, statesDim);
		ndBrainMemVector targetOutput(targetOutputBuffer, actionDim);
		ndBrainMemVector criticInput(criticInputBuffer, statesDim + actionDim);

		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = ndInt32(shuffleBuffer[i]);
			const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_replayBuffer[index];

			for (ndInt32 j = 0; j < statesDim; ++j)
			{
				targetInput[j] = transition.m_nextState[j];
				criticInput[j] = transition.m_nextState[j];
			}
			m_targetActor.MakePrediction(targetInput, targetOutput);
			for (ndInt32 j = 0; j < actionDim; ++j)
			{
				criticInput[j + statesDim] = targetOutput[j];
			}

			ndReal targetValue = transition.m_reward;
			if (!transition.m_terminalState)
			{
				m_targetCritic1.MakePrediction(criticInput, criticOutput);
				ndReal value1 = criticOutput[0];

				m_targetCritic0.MakePrediction(criticInput, criticOutput);
				ndReal value0 = criticOutput[0];

				targetValue = transition.m_reward + m_discountFactor * ndMin (value0, value1);
			}
			loss0.m_reward = targetValue;
			loss1.m_reward = targetValue;

			for (ndInt32 j = 0; j < statesDim; ++j)
			{
				criticInput[j] = transition.m_state[j];
			}
			for (ndInt32 j = 0; j < actionDim; ++j)
			{
				criticInput[j + statesDim] = transition.m_action[j];
			}

			trainer0.BackPropagate(criticInput, loss0);
			trainer1.BackPropagate(criticInput, loss1);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer* const trainer0 = *m_criticOptimizer0[0];;
		ndBrainTrainer* const trainer1 = *m_criticOptimizer1[0];
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer* const srcTrainer0 = *m_criticOptimizer0[i];
			ndBrainTrainer* const srcTrainer1 = *m_criticOptimizer1[i];
			trainer0->AcculumateGradients(*srcTrainer0, threadIndex, threadCount);
			trainer1->AcculumateGradients(*srcTrainer1, threadIndex, threadCount);
		}
	});

	ndBrainThreadPool::ParallelExecute(PropagateBash);
	ndBrainThreadPool::ParallelExecute(AccumulateWeight);
	m_criticOptimizer0[0]->UpdateWeights(m_criticLearnRate, m_bashBufferSize);
	m_criticOptimizer1[0]->UpdateWeights(m_criticLearnRate, m_bashBufferSize);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::BackPropagateActor(const ndUnsigned32* const shuffleBuffer)
{
	auto PropagateBash = ndMakeObject::ndFunction([this, &shuffleBuffer](ndInt32 threadIndex, ndInt32 threadCount)
	{
		class ActorLoss : public ndBrainLoss
		{
			public:
			ActorLoss(ndBrainTrainer& actorTrainer, ndBrainAgentTD3_Trainer<statesDim, actionDim>* const agent)
				:ndBrainLoss()
				,m_actorTrainer(actorTrainer)
				,m_agent(agent)
				,m_index(0)
			{
			}

			void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(loss.GetCount() == actionDim);
				ndAssert(output.GetCount() == actionDim);
				const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_agent->m_replayBuffer[m_index];
				
				ndReal criticInputBuffer[(statesDim + actionDim) * 2];
				ndBrainMemVector input(criticInputBuffer, statesDim + actionDim);
				for (ndInt32 i = 0; i < statesDim; ++i)
				{
					input[i] = transition.m_state[i];
				}
				for (ndInt32 i = 0; i < actionDim; ++i)
				{
					input[i + statesDim] = output[i];
				}
				m_agent->m_critic0.CalculateInputGradients(input, input);

				for (ndInt32 i = 0; i < actionDim; ++i)
				{
					loss[i] = input[statesDim + i];
				}
			}

			ndBrainTrainer& m_actorTrainer;
			ndBrainAgentTD3_Trainer<statesDim, actionDim>* m_agent;
			ndInt32 m_index;
		};

		ndBrainTrainer& actorTrainer = *(*m_actorOptimizer[threadIndex]);
		ActorLoss loss(actorTrainer, this);

		ndReal inputBuffer[statesDim * 2];
		ndBrainMemVector input(inputBuffer, statesDim);

		actorTrainer.ClearGradientsAcc();
		const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = ndInt32(shuffleBuffer[i]);
			const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_replayBuffer[index];

			for (ndInt32 j = 0; j < statesDim; ++j)
			{
				input[j] = transition.m_state[j];
			}
			loss.m_index = index;
			actorTrainer.BackPropagate(input, loss);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer& trainer = *(*m_actorOptimizer[0]);
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer& srcTrainer = *(*m_actorOptimizer[i]);
			trainer.AcculumateGradients(srcTrainer, threadIndex, threadCount);
		}
	});

	ParallelExecute(PropagateBash);
	ParallelExecute(AccumulateWeight);
	m_actorOptimizer[0]->UpdateWeights(-m_actorLearnRate, m_bashBufferSize);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::BackPropagate()
{
	ndFixSizeArray<ndUnsigned32, 1024> shuffleBuffer;
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		shuffleBuffer.PushBack(ndRandInt() % m_replayBuffer.GetCount());
	}

	BackPropagateCritic(&shuffleBuffer[0]);
	BackPropagateActor(&shuffleBuffer[0]);

	m_targetActor.SoftCopy(m_actor, m_softTargetFactor);
	m_targetCritic0.SoftCopy(m_critic0, m_softTargetFactor);
	m_targetCritic1.SoftCopy(m_critic1, m_softTargetFactor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::Optimize()
{
	BackPropagate();
	if (IsSampling())
	{
		ndExpandTraceMessage("%d star training: episode %d\n", m_frameCount, m_eposideCount);
	}
	m_collectingSamples = false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::CalculateQvalue(const ndBrainVector& state, const ndBrainVector& actions)
{
	ndReal buffer[(statesDim + actionDim) * 2];
	ndBrainMemVector criticInput(buffer, statesDim + actionDim);
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		criticInput[i] = state[i];
	}
	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		criticInput[i + statesDim] = actions[i];
	}

	ndReal currentQValueBuffer[2];
	ndBrainMemVector currentQValue(currentQValueBuffer, 1);

	m_critic1.MakePrediction(criticInput, currentQValue);
	ndReal reward1 = currentQValue[0];

	m_critic0.MakePrediction(criticInput, currentQValue);
	ndReal reward0 = currentQValue[0];

	m_currentQValue = ndMin(reward0, reward1);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::Step()
{
	ndReal stateBuffer[statesDim * 2];
	ndReal actionBuffer[actionDim * 2];
	ndBrainMemVector state(stateBuffer, statesDim);
	ndBrainMemVector actions(actionBuffer, actionDim);

	GetObservation(&m_currentTransition.m_state[0]);
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		state[i] = m_currentTransition.m_state[i];
	}
	m_actor.MakePrediction(state, actions);

	// explore environment
	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		actions[i] = PerturbeAction(actions[i]);
	}
	ApplyActions(&actions[0]);
	m_currentTransition.m_reward = GetReward();

	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		m_currentTransition.m_action[i] = actions[i];
	}

	if (!IsSampling())
	{
		// Get Q vale from Critic
		CalculateQvalue(state, actions);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ResetModel();
	}

	GetObservation(&m_currentTransition.m_nextState[0]);
	m_currentTransition.m_terminalState = IsTerminal();
	m_replayBuffer.AddTransition(m_currentTransition);

	if (m_frameCount > m_replayBufferPrefill)
	{
		Optimize();
	}

	if (m_currentTransition.m_terminalState)
	{
		ResetModel();
		m_currentTransition.Clear();
		if (IsSampling() && (m_eposideCount % 100 == 0))
		{
			ndExpandTraceMessage("collecting samples: frame %d out of %d, episode %d \n", m_frameCount, m_replayBuffer.GetCapacity(), m_eposideCount);
		}
		m_eposideCount++;
		m_framesAlive = 0;
	}

	m_frameCount++;
	m_framesAlive++;
}

#endif 
