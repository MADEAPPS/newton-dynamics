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

#ifndef _ND_DQN_BRAIN_AGENT_DQN_TRAINER_H__
#define _ND_DQN_BRAIN_AGENT_DQN_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"

// this is an implementation of the vanilla dqn agent trainer as described 
// on the nature paper below. 
// https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf

// default hyper parameters defaults
#define D_DQN_LEARN_RATE				ndReal(5.0e-3f)
#define D_DQN_DISCOUNT_FACTOR			ndReal (0.99f)
#define D_DQN_REPLAY_BUFFERSIZE			(1024 * 512)
//#define D_DQN_REPLAY_BUFFERSIZE			(1024)
#define D_DQN_REPLAY_BASH_SIZE			32
#define D_DQN_TARGET_UPDATE_PERIOD		1000
#define D_DQN_REGULARIZER				ndReal (2.0e-6f)
#define D_DQN_MIN_EXPLORE_PROBABILITY	ndReal(1.0f/100.0f)
#define D_DQN_EXPLORE_ANNELININGING		(D_DQN_MIN_EXPLORE_PROBABILITY / ndReal(2.0f))

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDQN_Trainer: public ndBrainAgent, public ndBrainThreadPool
{
	public: 
	ndBrainAgentDQN_Trainer(const ndSharedPtr<ndBrain>& actor);
	virtual ~ndBrainAgentDQN_Trainer();

	ndReal GetCurrentValue() const;
	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;
	ndInt32 GetEpisodeFrames() const;

	protected:
	void Step();
	void OptimizeStep();
	void Save(ndBrainSave* const loadSave) const;

	bool IsSampling() const;
	bool IsTerminal() const;
	ndReal GetReward() const;

	private:
	void Optimize();
	void BackPropagate();
	void PopulateReplayBuffer();
	void SetBufferSize(ndInt32 size);

	ndSharedPtr<ndBrain> m_actor;
	ndBrain m_target;
	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_actorOtimizer;

	ndBrainVector m_state;
	ndBrainVector m_actions;
	ndBrainReplayBuffer<ndInt32, statesDim, 1> m_replayBuffer;
	ndBrainReplayTransitionMemory<ndInt32, statesDim, 1> m_currentTransition;
	
	ndReal m_gamma;
	ndReal m_learnRate;
	ndReal m_currentQValue;
	ndReal m_explorationProbability;
	ndReal m_minExplorationProbability;
	ndReal m_explorationProbabilityAnnelining;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_targetUpdatePeriod;
	bool m_collectingSamples;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN_Trainer<statesDim, actionDim>::ndBrainAgentDQN_Trainer(const ndSharedPtr<ndBrain>& actor)
	:ndBrainAgent()
	,ndBrainThreadPool()
	,m_actor(actor)
	,m_target(*(*m_actor))
	,m_replayBuffer()
	,m_gamma(D_DQN_DISCOUNT_FACTOR)
	,m_learnRate(D_DQN_LEARN_RATE)
	,m_currentQValue(ndReal(0.0f))
	,m_explorationProbability(ndReal(1.0f))
	,m_minExplorationProbability(D_DQN_MIN_EXPLORE_PROBABILITY)
	,m_explorationProbabilityAnnelining(D_DQN_EXPLORE_ANNELININGING)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(D_DQN_REPLAY_BASH_SIZE)
	,m_targetUpdatePeriod(D_DQN_TARGET_UPDATE_PERIOD)
	,m_collectingSamples(true)
{
	ndInt32 threadCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize / 4);
threadCount = 1;
	SetThreadCount(threadCount);
	for (ndInt32 i = 0; i < GetThreadCount(); ++i)
	{
		m_actorOtimizer.PushBack(new ndBrainTrainer(*m_actor));
		m_actorOtimizer[m_actorOtimizer.GetCount() - 1]->SetRegularizer(D_DQN_REGULARIZER);
	}

	m_state.SetCount(statesDim);
	m_actions.SetCount(actionDim);
	m_state.Set(ndReal(0.0f));
	m_actions.Set(ndReal(0.0f));
	
	SetBufferSize(D_DQN_REPLAY_BUFFERSIZE);
	m_target.CopyFrom(*(*m_actor));

	m_explorationProbabilityAnnelining = (m_explorationProbability - m_minExplorationProbability) / ndReal (m_replayBuffer.GetCapacity());
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN_Trainer<statesDim, actionDim>::~ndBrainAgentDQN_Trainer()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetCurrentValue() const
{
	return m_currentQValue;
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDQN_Trainer<statesDim, actionDim>::IsSampling() const
{
	return m_collectingSamples;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetEposideCount() const
{
	return m_eposideCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetEpisodeFrames() const
{
	return m_framesAlive;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::SetBufferSize(ndInt32 size)
{
	m_replayBuffer.SetSize(size);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::BackPropagate()
{
	ndUnsigned32 shuffleBuffer[1024];
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		shuffleBuffer[i] = ndRandInt() % m_replayBuffer.GetCount();
	}

	auto OptimizeActor = ndMakeObject::ndFunction([this, &shuffleBuffer](ndInt32 threadIndex, ndInt32 threadCount)
	{
		class Loss: public ndBrainLeastSquareErrorLoss
		{
			public:
			Loss(ndBrainTrainer& trainer, ndBrainAgentDQN_Trainer<statesDim, actionDim>* const agent)
				:ndBrainLeastSquareErrorLoss(trainer.GetBrain()->GetOutputSize())
				,m_trainer(trainer)
				,m_agent(agent)
				,m_index(0)
			{
			}

			virtual void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(output.GetCount() == actionDim);
				ndAssert(m_truth.GetCount() == m_trainer.GetBrain()->GetOutputSize());

				ndReal stateBuffer[statesDim * 2];
				ndReal actionBuffer[actionDim * 2];
				ndDeepBrainMemVector state(stateBuffer, statesDim);
				ndDeepBrainMemVector action(actionBuffer, actionDim);

				const ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& transition = m_agent->m_replayBuffer[m_index];
#if 0
				for (ndInt32 i = 0; i < actionDim; ++i)
				{
					action[i] = output[i];
				}
				ndInt32 actionIndex = transition.m_action[0];
				if (transition.m_terminalState)
				{
					action[actionIndex] = transition.m_reward;
				}
				else
				{
					ndReal actionBuffer1[actionDim * 2];
					ndDeepBrainMemVector action1(actionBuffer1, actionDim);
				
					for (ndInt32 i = 0; i < statesDim; ++i)
					{
						state[i] = transition.m_nextState[i];
					}
					m_agent->m_target.MakePrediction(state, action1);
					action[actionIndex] = transition.m_reward + m_agent->m_gamma * action1[actionIndex];
				}
#else
				for (ndInt32 i = 0; i < statesDim; ++i)
				{
					state[i] = transition.m_nextState[i];
				}
				m_agent->m_target.MakePrediction(state, action);
				for (ndInt32 i = 0; i < actionDim; ++i)
				{
					action[i] = transition.m_reward + m_agent->m_gamma * action[i];
				}
				if (transition.m_terminalState)
				{
					ndInt32 actionIndex = transition.m_action[0];
					action[actionIndex] = transition.m_reward;
				}
#endif
				SetTruth(action);
				ndBrainLeastSquareErrorLoss::GetLoss(output, loss);
			}

			ndBrainTrainer& m_trainer;
			ndBrainAgentDQN_Trainer<statesDim, actionDim>* m_agent;
			ndInt32 m_index;
		};

		ndBrainTrainer& trainer = *(*m_actorOtimizer[threadIndex]);
		trainer.ClearGradientsAcc();

		Loss loss(trainer, this);
		ndReal stateBuffer[statesDim * 2];
		ndDeepBrainMemVector state(stateBuffer, statesDim);

		const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = ndInt32 (shuffleBuffer[i]);
			const ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& transition = m_replayBuffer[index];

			for (ndInt32 j = 0; j < statesDim; ++j)
			{
				state[j] = transition.m_state[j];
			}
			loss.m_index = index;
			trainer.BackPropagate(state, loss);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer& trainer = *(*m_actorOtimizer[0]);
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer& srcTrainer = *(*m_actorOtimizer[i]);
			trainer.AcculumateGradients(srcTrainer, threadIndex, threadCount);
		}
	});

	ParallelExecute(OptimizeActor);
	ParallelExecute(AccumulateWeight);
	m_actorOtimizer[0]->UpdateWeights(m_learnRate, m_bashBufferSize);

	if ((m_frameCount % m_targetUpdatePeriod) == (m_targetUpdatePeriod - 1))
	{
		// update on line network
		m_target.CopyFrom(*(*m_actor));
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave) const
{
	loadSave->Save(*m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDQN_Trainer<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndReal (0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::PopulateReplayBuffer()
{
	GetObservation(&m_currentTransition.m_nextState[0]);
	m_currentTransition.m_reward = GetReward();
	m_currentTransition.m_terminalState = IsTerminal();
	m_replayBuffer.AddTransition(m_currentTransition);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::Optimize()
{
	BackPropagate();
	if (IsSampling())
	{
		ndExpandTraceMessage("%d start training: episode %d\n", m_frameCount, m_eposideCount);
	}
	m_collectingSamples = false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::Step()
{
	GetObservation(&m_state[0]);
	m_actor->MakePrediction(m_state, m_actions);
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		m_currentTransition.m_state[i] = ndBrainAgentDQN_Trainer<statesDim, actionDim>::m_state[i];
	}

	ndInt32 action = 0;
	ndFloat32 explore = ndRand();
	if (explore <= m_explorationProbability)
	{
		// explore environment
		ndUnsigned32 randomIndex = ndRandInt();
		action = ndInt32(randomIndex % actionDim);
	}
	else
	{
		// exploit environment
		action = 0;
		ndReal maxQValue = m_actions[0];
		for (ndInt32 i = 1; i < actionDim; ++i)
		{
			if (m_actions[i] > maxQValue)
			{
				action = i;
				maxQValue = m_actions[i];
			}
		}

		m_currentQValue = maxQValue;
	}

	ndReal bestAction = ndReal(action);
	m_currentTransition.m_action[0] = action;

	ApplyActions(&bestAction);
	if (bestAction != ndReal(m_currentTransition.m_action[0]))
	{
		m_currentTransition.m_action[0] = ndInt32(bestAction);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ResetModel();
	}

	PopulateReplayBuffer();
	if (m_replayBuffer.GetCount() == m_replayBuffer.GetCapacity())
	{
		Optimize();
	}

	if (m_currentTransition.m_terminalState)
	{
		ndBrainAgentDQN_Trainer<statesDim, actionDim>::m_state.Set(ndReal(0.0f));
		ndBrainAgentDQN_Trainer<statesDim, actionDim>::m_actions.Set(ndReal(0.0f));
		ResetModel();
		m_currentTransition.Clear();
		if (IsSampling() && (m_eposideCount % 500 == 0))
		{
			ndExpandTraceMessage("collecting samples: frame %d out of %d, episode %d \n", m_frameCount, m_replayBuffer.GetCapacity(), m_eposideCount);
		}
		m_eposideCount++;
		m_framesAlive = 0;
	}

	m_frameCount++;
	m_framesAlive++;
	m_explorationProbability = ndMax(m_explorationProbability - m_explorationProbabilityAnnelining, m_minExplorationProbability);
}

#endif 