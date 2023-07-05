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

#ifndef _ND_BRAIN_AGENT_DDPG_H__
#define _ND_BRAIN_AGENT_DDPG_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"

// this is an implementation of the vanilla 
// Continuous control with deep re enforcement learning (ddpg agent)
// trainer as described in: https://arxiv.org/pdf/1509.02971.pdf

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDDPG: public ndBrainAgent
{
	public:
	ndBrainAgentDDPG(const ndSharedPtr<ndBrain>& actor);
	void Step();

	protected:
	void OptimizeStep();
	void ResetModel() const;
	bool IsTerminal() const;
	ndReal GetReward() const;
	void ApplyRandomAction() const;
	//virtual ndInt32 SelectBestAction();
	ndInt32 GetOpmizationDelay() const;
	void SetOpmizationDelay(ndInt32 delay);
	void Save(ndBrainSave* const loadSave) const;

	ndSharedPtr<ndBrain> m_actor;
	ndBrainInstance m_actorInstance;
	ndBrainVector m_state;
	ndBrainVector m_actions;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDDPG<statesDim, actionDim>::ndBrainAgentDDPG(const ndSharedPtr<ndBrain>& actor)
	:ndBrainAgent()
	,m_actor(actor)
	,m_actorInstance(*m_actor)
{
	m_state.SetCount(statesDim);
	m_actions.SetCount(actionDim);
	m_state.Set(ndReal(0.0f));
	m_actions.Set(ndReal(0.0f));
}

//template<ndInt32 statesDim, ndInt32 actionDim>
//ndInt32 ndBrainAgentDDPG<statesDim, actionDim>::SelectBestAction()
//{
//	ndInt32 bestAction = 0;
//	ndReal maxReward = m_actions[0];
//	for (ndInt32 i = 1; i < actionDim; ++i)
//	{
//		if (m_actions[i] > maxReward)
//		{
//			bestAction = i;
//			maxReward = m_actions[i];
//		}
//	}
//	return bestAction;
//}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::ApplyRandomAction() const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDDPG<statesDim, actionDim>::IsTerminal() const
{
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG<statesDim, actionDim>::GetOpmizationDelay() const
{
	return 0;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::SetOpmizationDelay(ndInt32)
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::Step()
{
	GetObservation(&m_state[0]);
	m_actorInstance.MakePrediction(m_state, m_actions);
	ApplyActions(&m_actions[0]);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDDPG<statesDim, actionDim>::GetReward() const
{
	return ndReal(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::ResetModel() const
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::OptimizeStep()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::Save(ndBrainSave* const) const
{
	ndAssert(0);
}

// ******************************************************************************
//
// ******************************************************************************
template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDDPG_Trainner: public ndBrainAgent
{
	// hyper parameters defaults
	#define D_DDPG_LEARN_RATE				ndReal(2.0e-4f)
	#define D_DDPG_DISCOUNT_FACTOR			ndReal (0.99f)
	#define D_DDPG_REPLAY_BUFFERSIZE		(1024 * 512)
	#define D_DDPG_MOVING_AVERAGE			64
	#define D_DDPG_REPLAY_BASH_SIZE			32
	#define D_DDPG_TARGET_UPDATE_PERIOD		1000
	#define D_DDPG_OPTIMIZATION_DELAY		3
	#define D_DDPG_REGULARIZER				ndReal (2.0e-6f)
	#define D_DDPG_MIN_EXPLORE_PROBABILITY	ndReal(1.0f/100.0f)
	#define D_DDPG_STAR_OPTIMIZATION		(D_DDPG_REPLAY_BUFFERSIZE - 1000)
	#define D_DDPG_EXPLORE_ANNELININGING	(D_DDPG_MIN_EXPLORE_PROBABILITY / ndReal(2.0f))

	public:
	ndBrainAgentDDPG_Trainner(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic);
	virtual ~ndBrainAgentDDPG_Trainner();

	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;

	protected:
	void OptimizeStep();
	void Save(ndBrainSave* const loadSave) const;

	private:
	void Step();
	void PrintDebug();
	void BackPropagate();
	void ApplyRandomAction() const;
	void SetBufferSize(ndInt32 size);
	ndInt32 GetOpmizationDelay() const;
	void SetOpmizationDelay(ndInt32 delay);
	//virtual ndInt32 SelectBestAction();

	class ndOptimizer : public ndBrainTrainer
	{
		public:
		ndOptimizer(ndBrain* const brain)
			:ndBrainTrainer(brain)
			,m_truth()
			,m_inputBatch()
			,m_outputBatch()
			,m_agent(nullptr)
		{
			ndAssert(0);
			m_truth.SetCount(actionDim);
			m_inputBatch.SetCount(statesDim);
			m_outputBatch.SetCount(actionDim);
		}

		ndOptimizer(const ndOptimizer& src)
			:ndBrainTrainer(src)
			,m_truth()
			,m_inputBatch()
			,m_outputBatch()
			,m_agent(nullptr)
		{
			ndAssert(0);
			m_truth.SetCount(actionDim);
			m_inputBatch.SetCount(statesDim);
			m_outputBatch.SetCount(actionDim);
		}

		//virtual void GetGroundTruth(ndInt32 index, ndBrainVector& groundTruth, const ndBrainVector& output) const
		//void EvaluateBellmanEquation(ndInt32 index)
		void EvaluateBellmanEquation(ndInt32)
		{
			ndAssert(0);
			//ndAssert(m_truth.GetCount() == m_output.GetCount());
			//ndAssert(m_truth.GetCount() == m_outputBatch.GetCount());
			//const ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& transition = m_agent->m_replayBuffer[index];
			//
			//for (ndInt32 i = 0; i < statesDim; ++i)
			//{
			//	m_inputBatch[i] = transition.m_state[i];
			//}
			//MakePrediction(m_inputBatch);
			//
			//for (ndInt32 i = 0; i < actionDim; ++i)
			//{
			//	m_truth[i] = m_output[i];
			//	ndAssert(ndAbs(m_output[i]) < ndReal(100.0f));
			//}
			//
			//ndInt32 action = transition.m_action[0];
			//if (transition.m_terminalState)
			//{
			//	m_truth[action] = transition.m_reward;
			//}
			//else
			//{
			//	for (ndInt32 i = 0; i < statesDim; ++i)
			//	{
			//		m_inputBatch[i] = transition.m_nextState[i];
			//	}
			//	m_agent->m_targetActorInstance.MakePrediction(m_inputBatch, m_outputBatch);
			//	m_truth[action] = transition.m_reward + m_agent->m_gamma * m_outputBatch[action];
			//}
		}

		virtual void Optimize(ndValidation&, const ndBrainMatrix&, ndReal, ndInt32)
		{
			ndAssert(0);
			ndArray<ndInt32>& shuffleBuffer = m_agent->m_shuffleBuffer;

			ClearGradientsAcc();
			shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
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

		ndBrainVector m_truth;
		ndBrainVector m_inputBatch;
		ndBrainVector m_outputBatch;
		ndBrainAgentDDPG_Trainner<statesDim, actionDim>* m_agent;
	};

	ndSharedPtr<ndBrain> m_actor;
	ndSharedPtr<ndBrain> m_critic;
	ndBrain m_actorTarget;
	ndBrain m_criticTarget;
	ndOptimizer m_actorOptimizer;
	ndOptimizer m_criticOptimizer;
	ndBrainInstance m_actorTargetInstance;
	ndBrainInstance m_criticTargetInstance;
	ndArray<ndInt32> m_movingAverage;
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

	ndInt32 m_framesAlive;
	ndInt32 m_movingAverageIndex;
	ndInt32 m_optimizationDelay;
	ndInt32 m_optimizationDelayCount;
	bool m_collectingSamples;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDDPG_Trainner<statesDim, actionDim>::ndBrainAgentDDPG_Trainner(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic)
	:ndBrainAgent()
	,m_actor(actor)
	,m_critic(critic)
	,m_actorTarget(*(*m_actor))
	,m_criticTarget(*(*m_critic))
	,m_actorOptimizer(*m_actor)
	,m_criticOptimizer(*m_critic)
	,m_actorTargetInstance(&m_actorTarget)
	,m_criticTargetInstance(&m_criticTarget)
	,m_movingAverage()
	,m_shuffleBuffer()
	,m_replayBuffer()
	,m_gamma(D_DDPG_DISCOUNT_FACTOR)
	,m_learnRate(D_DDPG_LEARN_RATE)
	,m_explorationProbability(ndReal(1.0f))
	,m_minExplorationProbability(D_DDPG_MIN_EXPLORE_PROBABILITY)
	,m_explorationProbabilityAnnelining(D_DDPG_EXPLORE_ANNELININGING)
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_bashBufferSize(D_DDPG_REPLAY_BASH_SIZE)
	,m_startOptimization(D_DDPG_STAR_OPTIMIZATION)
	,m_targetUpdatePeriod(D_DDPG_TARGET_UPDATE_PERIOD)
	,m_framesAlive(0)
	,m_movingAverageIndex(0)
	,m_optimizationDelay(D_DDPG_OPTIMIZATION_DELAY)
	,m_optimizationDelayCount(0)
	,m_collectingSamples(true)
{
	ndAssert(0);
	//m_actorOptimizer.m_agent = this;
	//m_actorOptimizer.SetRegularizer(D_DDPG_REGULARIZER);
	//m_explorationProbabilityAnnelining = (m_explorationProbability - m_minExplorationProbability) / D_DDPG_STAR_OPTIMIZATION;
	//
	//SetBufferSize(D_DDPG_REPLAY_BUFFERSIZE);
	//m_targetActor.CopyFrom(*(*m_actor));
	//
	//for (ndInt32 i = 0; i < D_DDPG_MOVING_AVERAGE; ++i)
	//{
	//	m_movingAverage.PushBack(0);
	//}
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDDPG_Trainner<statesDim, actionDim>::~ndBrainAgentDDPG_Trainner()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainner<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainner<statesDim, actionDim>::GetEposideCount() const
{
	return m_eposideCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::SetBufferSize(ndInt32 size)
{
	m_shuffleBuffer.SetCount(size);
	m_replayBuffer.SetCount(size);

	m_shuffleBuffer.SetCount(0);
	m_replayBuffer.SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::BackPropagate()
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
			return ndReal(1.0f);
		}
	};

	ndBrainMatrix inputBatch;
	ndTestValidator validator(m_actorOptimizer);
	m_actorOptimizer.Optimize(validator, inputBatch, m_learnRate, 1);
}

//template<ndInt32 statesDim, ndInt32 actionDim>
//ndInt32 ndBrainAgentDDPG_Trainner<statesDim, actionDim>::SelectBestAction()
//{
//	ndFloat32 explore = ndRand();
//	if (explore <= m_explorationProbability)
//	{
//		// explore environment
//		ndUnsigned32 randomIndex = ndRandInt();
//		ndInt32 action = ndInt32(randomIndex % actionDim);
//		m_currentTransition.m_action[0] = action;
//		return action;
//	}
//	else
//	{
//		// exploit environment
//		ndInt32 action = ndBrainAgentDDPG<statesDim, actionDim>::SelectBestAction();
//		m_currentTransition.m_action[0] = action;
//		return action;
//	}
//}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::ApplyRandomAction() const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::PrintDebug()
{
	m_movingAverage[m_movingAverageIndex] = m_framesAlive;
	m_movingAverageIndex = (m_movingAverageIndex + 1) % m_movingAverage.GetCount();

	ndInt32 sum = 0;
	m_framesAlive = 0;
	for (ndInt32 i = m_movingAverage.GetCount() - 1; i >= 0; --i)
	{
		sum += m_movingAverage[i];
	}
	sum = sum / m_movingAverage.GetCount();
	if (!m_collectingSamples)
	{
		ndExpandTraceMessage("%d moving average alive frames:%d\n", m_frameCount - 1, sum);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainner<statesDim, actionDim>::GetOpmizationDelay() const
{
	return m_optimizationDelay;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::SetOpmizationDelay(ndInt32 delay)
{
	m_optimizationDelay = delay;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::Save(ndBrainSave* const loadSave) const
{
	loadSave->Save(*m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::Step()
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainner<statesDim, actionDim>::OptimizeStep()
{
	ndAssert(0);
	//m_optimizationDelayCount++;
	//if (m_optimizationDelayCount <= GetOpmizationDelay())
	//{
	//	ApplyRandomAction();
	//	return;
	//}
	//
	//if (!m_frameCount)
	//{
	//	ndBrainAgentDDPG_Trainner<statesDim, actionDim>::m_state.Set(ndReal(0.0f));
	//	ndBrainAgentDDPG_Trainner<statesDim, actionDim>::m_actions.Set(ndReal(0.0f));
	//	ResetModel();
	//	m_currentTransition.Clear();
	//	m_optimizationDelayCount = 0;
	//}
	//
	//GetObservation(&m_currentTransition.m_nextState[0]);
	//m_currentTransition.m_reward = GetReward();
	//m_currentTransition.m_terminalState = IsTerminal();
	//for (ndInt32 i = 0; i < statesDim; ++i)
	//{
	//	m_currentTransition.m_state[i] = ndBrainAgentDDPG_Trainner<statesDim, actionDim>::m_state[i];
	//}
	//
	//m_replayBuffer.AddTransition(m_currentTransition);
	//if (m_frameCount < m_shuffleBuffer.GetCapacity())
	//{
	//	m_shuffleBuffer.PushBack(m_frameCount);
	//}
	//
	//if (m_currentTransition.m_terminalState)
	//{
	//	ndBrainAgentDDPG_Trainner<statesDim, actionDim>::m_state.Set(ndReal(0.0f));
	//	ndBrainAgentDDPG_Trainner<statesDim, actionDim>::m_actions.Set(ndReal(0.0f));
	//	ResetModel();
	//	m_currentTransition.Clear();
	//	if ((m_frameCount < m_startOptimization) && (m_eposideCount % 32 == 0))
	//	{
	//		ndExpandTraceMessage("collecting samples: frame %d out of %d, episode %d \n", m_frameCount, m_startOptimization, m_eposideCount);
	//	}
	//	m_eposideCount++;
	//	m_optimizationDelayCount = 0;
	//	ndBrainAgentDDPG_Trainner<statesDim, actionDim>::PrintDebug();
	//}
	//
	//// begin exploitation only after there are a replay buffer is full of random training samples
	//m_explorationProbability = ndMax(m_explorationProbability - m_explorationProbabilityAnnelining, m_minExplorationProbability);
	//if (m_frameCount > m_startOptimization)
	//	//if (m_frameCount > 100)
	//{
	//	BackPropagate();
	//	if (m_collectingSamples)
	//	{
	//		ndExpandTraceMessage("%d star training: episode %d\n", m_frameCount, m_eposideCount);
	//	}
	//	m_collectingSamples = false;
	//}
	//
	//if ((m_frameCount % m_targetUpdatePeriod) == (m_targetUpdatePeriod - 1))
	//{
	//	// update on line network
	//	m_targetActor.CopyFrom(*(*m_actor));
	//}
	//
	//m_frameCount++;
	//m_framesAlive++;
}

#endif 

