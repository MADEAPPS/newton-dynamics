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

#ifndef _ND_BRAIN_AGENT_DQN_H__
#define _ND_BRAIN_AGENT_DQN_H__

#include "ndBrainStdafx.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"

// this is an implementation of the vanilla dqn agent trainer as described 
// on the nature paper below. 
// https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDQN : public ndBrainAgent
{
	public:
	ndBrainAgentDQN(const ndSharedPtr<ndBrain>& qValuePredictor)
		:ndBrainAgent()
		,m_onlineNetwork(qValuePredictor)
		,m_instance(*m_onlineNetwork)
	{
		m_state.SetCount(statesDim);
		m_actions.SetCount(actionDim);
	}

	void GetAction(ndReal* const actionSpace) const
	{
		GetObservation(&m_state[0]);
		m_instance.MakePrediction(m_state, m_actions);

		ndInt32 bestAction = 0;
		ndReal maxReward = m_actions[0];
		for (ndInt32 i = 1; i < actionDim; ++i)
		{
			if (m_actions[i] > maxReward)
			{
				bestAction = i;
				maxReward = m_actions[i];
			}
		}
		actionSpace[0] = ndReal (bestAction);
	}

	ndSharedPtr<ndBrain> m_onlineNetwork;
	mutable ndBrainInstance m_instance;
	mutable ndBrainVector m_state;
	mutable ndBrainVector m_actions;
};

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDQN_Trainner: public ndBrainAgentDQN<statesDim, actionDim>
{
	#define D_LEARN_RATE				ndReal(2.0e-4f)
	#define D_DISCOUNT_FACTOR			ndReal (0.99f)
	#define D_REPLAY_BUFFERSIZE			(1024 * 512)
	#define D_REPLAY_BASH_SIZE			(32)
	#define D_TARGET_UPDATE_PERIOD		(1000)
	#define D_MIN_EXPLORE_PROBABILITY	ndReal(1.0f/1024.0f)
	#define D_STAR_OPTIMIZATION			(D_REPLAY_BUFFERSIZE - 1000)
	#define D_EXPLORE_ANNELININGING		(D_MIN_EXPLORE_PROBABILITY / ndReal(2.0f))

	class ndOptimizer: public ndBrainTrainer
	{
		public:
		ndOptimizer(ndBrain* const brain)
			:ndBrainTrainer(brain)
			,m_inputBatch()
			,m_agent(nullptr)
		{
			m_inputBatch.SetCount(statesDim);
			m_outputBatch.SetCount(actionDim);
		}

		ndOptimizer(const ndOptimizer& src)
			:ndBrainTrainer(src)
			,m_inputBatch()
			,m_agent(nullptr)
		{
			m_inputBatch.SetCount(statesDim);
		}

		//virtual void GetGroundTruth(ndInt32 index, ndBrainVector& groundTruth, const ndBrainVector& output) const
		void EvaluateBellmanEquation(ndInt32 index)
		{
			ndBrainVector& groundTruth = m_truth;
			ndAssert(groundTruth.GetCount() == m_output.GetCount());
			ndAssert(groundTruth.GetCount() == m_outputBatch.GetCount());
			const ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& transition = m_agent->m_replayBuffer[index];
			
			for (ndInt32 i = 0; i < statesDim; ++i)
			{
				m_inputBatch[i] = transition.m_state[i];
			}
			MakePrediction(m_inputBatch);
			groundTruth = m_outputBatch;

			ndInt32 action = transition.m_action[0];
			if (transition.m_terminalState)
			{
				groundTruth[action] = transition.m_reward;
			}
			else
			{
				for (ndInt32 i = 0; i < statesDim; ++i)
				{
					m_inputBatch[i] = transition.m_nextState[i];
				}
				m_agent->m_targetInstance.MakePrediction(m_inputBatch, m_outputBatch);
				groundTruth[action] = transition.m_reward + m_agent->m_gamma * m_outputBatch[action];
			}
		}

		virtual void Optimize(ndValidation&, const ndBrainMatrix&, ndReal, ndInt32 )
		{
			m_truth.SetCount(m_output.GetCount());
			ndArray<ndInt32>& shuffleBuffer = m_agent->m_shuffleBuffer;
			//const ndBrainReplayBuffer<ndInt32, statesDim, 1>& replayBuffer = m_agent->m_replayBuffer;

			shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
			ClearGradientsAcc();
			for (ndInt32 i = 0; i < m_agent->m_bashBufferSize; ++i)
			{
				ndInt32 index = shuffleBuffer[i];
				//GetGroundTruth(index, truth, m_output);
				EvaluateBellmanEquation(index);
				BackPropagate(m_truth);
			}
			UpdateWeights(m_agent->m_learnRate, m_agent->m_bashBufferSize);
			ApplyWeightTranspose();
		}

		ndBrainVector m_inputBatch;
		ndBrainVector m_outputBatch;
		ndBrainAgentDQN_Trainner<statesDim, actionDim>* m_agent;
	};

	public: 
	ndBrainAgentDQN_Trainner(const ndSharedPtr<ndBrain>& qValuePredictor);
	virtual ~ndBrainAgentDQN_Trainner();

	void GetAction(ndReal* const actionSpace) const;

	protected:
	virtual void OptimizeStep();
	
	void BackPropagate();
	void SetBufferSize(ndInt32 size);
	ndInt32 GetExploreExploitAction();

	ndBrain m_targetNetwork;
	ndOptimizer m_trainer;
	ndBrainInstance m_targetInstance;
	ndArray<ndInt32> m_shuffleBuffer;
	ndBrainReplayBuffer<ndInt32, statesDim, 1> m_replayBuffer;
	ndBrainReplayTransitionMemory<ndInt32, statesDim, 1> m_currentTransition;
	
	ndReal m_gamma;
	ndReal m_learnRate;
	ndReal m_explorationProbability;
	ndReal m_minExplorationProbability;
	ndReal m_explorationProbabilityAnnelining;
	ndInt32 m_frameCount;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_startOptimization;
	ndInt32 m_targetUpdatePeriod;
	bool m_collectingSamples;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN_Trainner<statesDim, actionDim>::ndBrainAgentDQN_Trainner(const ndSharedPtr<ndBrain>& qValuePredictor)
	:ndBrainAgentDQN<statesDim, actionDim>(qValuePredictor)
	,m_targetNetwork(*(*m_onlineNetwork))
	,m_trainer(*m_onlineNetwork)
	,m_targetInstance(&m_targetNetwork)
	,m_shuffleBuffer()
	,m_replayBuffer()
	,m_gamma(D_DISCOUNT_FACTOR)
	,m_learnRate(D_LEARN_RATE)
	,m_explorationProbability(ndReal(1.0f))
	,m_minExplorationProbability(D_MIN_EXPLORE_PROBABILITY)
	,m_explorationProbabilityAnnelining(D_EXPLORE_ANNELININGING)
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_bashBufferSize(D_REPLAY_BASH_SIZE)
	,m_startOptimization(D_STAR_OPTIMIZATION)
	,m_targetUpdatePeriod(D_TARGET_UPDATE_PERIOD)
	,m_collectingSamples(true)
{
	m_trainer.m_agent = this;
	m_explorationProbabilityAnnelining = (m_explorationProbability - m_minExplorationProbability) / D_STAR_OPTIMIZATION;

	SetBufferSize(D_REPLAY_BUFFERSIZE);
	m_targetNetwork.CopyFrom(*(*m_onlineNetwork));
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN_Trainner<statesDim, actionDim>::~ndBrainAgentDQN_Trainner()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainner<statesDim, actionDim>::SetBufferSize(ndInt32 size)
{
	m_shuffleBuffer.SetCount(size);
	m_replayBuffer.SetCount(size);

	m_shuffleBuffer.SetCount(0);
	m_replayBuffer.SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainner<statesDim, actionDim>::GetAction(ndReal* const actionSpace) const
{
	actionSpace[0] = ndReal (m_currentTransition.m_action[0]);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN_Trainner<statesDim, actionDim>::GetExploreExploitAction()
{
	ndInt32 action = 0;
	ndFloat32 explore = ndRand();
	if (explore <= m_explorationProbability)
	{
		ndUnsigned32 randomIndex = ndRandInt();
		action = ndInt32(randomIndex % actionDim);
	}
	else
	{
		ndReal maxAction;
		ndBrainAgentDQN<statesDim, actionDim>::GetAction(&maxAction);
		action = ndInt32(maxAction);
	}

	return action;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainner<statesDim, actionDim>::BackPropagate()
{
	class ndTestValidator : public ndBrainTrainer::ndValidation
	{
		public:
		ndTestValidator(ndBrainTrainer& trainer)
			:ndBrainTrainer::ndValidation(trainer)
		{
		}
	
		ndReal Validate(const ndBrainMatrix&)
		{
			return ndReal (1.0f);
		}
	};

	ndBrainMatrix inputBatch;
	ndTestValidator validator(m_trainer);
	m_trainer.Optimize(validator, inputBatch, m_learnRate, 1);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainner<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ResetModel();
		m_currentTransition.Clear();
	}

	GetObservation(&m_currentTransition.m_nextState[0]);
	m_currentTransition.m_reward = GetReward();
	m_currentTransition.m_terminalState = IsTerminal();

	m_replayBuffer.AddTransition(m_currentTransition);

	if (m_frameCount < m_shuffleBuffer.GetCapacity())
	{
		m_shuffleBuffer.PushBack(m_frameCount);
	}

	m_currentTransition.m_state = m_currentTransition.m_nextState;
	m_currentTransition.m_action[0] = GetExploreExploitAction();

	if (m_currentTransition.m_terminalState)
	{
		ResetModel();
		m_currentTransition.Clear();
		if ((m_frameCount < m_startOptimization) && (m_eposideCount % 32 == 0))
		{
			ndExpandTraceMessage("collecting samples: frame %d out of %d, episode %d \n", m_frameCount, m_startOptimization, m_eposideCount);
		}
		m_eposideCount++;
	}

	m_explorationProbability = ndMax(m_explorationProbability - m_explorationProbabilityAnnelining, m_minExplorationProbability);
	if (m_frameCount > m_startOptimization)
	{
		BackPropagate();
		if (m_collectingSamples)
		{
			ndExpandTraceMessage("%d star training: episode %d\n", m_frameCount, m_eposideCount);
		}
		m_collectingSamples = false;
	}

	if ((m_frameCount % m_targetUpdatePeriod) == (m_targetUpdatePeriod - 1))
	{
		// update on line network
		m_targetNetwork.CopyFrom(*(*m_onlineNetwork));
	}

	m_frameCount++;
}

#endif 
