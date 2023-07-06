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

#ifndef _ND_BRAIN_AGENT_DDPG_TRAINER_H__
#define _ND_BRAIN_AGENT_DDPG_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"

// this is an implementation of the vanilla 
// Continuous control with deep re enforcement learning (ddpg agent)
// trainer as described in: https://arxiv.org/pdf/1509.02971.pdf

// defualt hyper parameters defaults
#define D_DDPG_LEARN_RATE				ndReal(2.0e-4f)
#define D_DDPG_DISCOUNT_FACTOR			ndReal (0.99f)
#define D_DDPG_REPLAY_BUFFERSIZE		(1024 * 512)
#define D_DDPG_MOVING_AVERAGE			64
#define D_DDPG_REPLAY_BASH_SIZE			32
#define D_DDPG_TARGET_UPDATE_PERIOD		1000
#define D_DDPG_OPTIMIZATION_DELAY		3
#define D_DDPG_REGULARIZER				ndReal (2.0e-6f)
#define D_DDPG_ACTION_NOISE_DEVIATION	ndReal (0.25f)
#define D_DDPG_MIN_EXPLORE_PROBABILITY	ndReal(1.0f/100.0f)
#define D_DDPG_STAR_OPTIMIZATION		(D_DDPG_REPLAY_BUFFERSIZE - 1000)
#define D_DDPG_EXPLORE_ANNELININGING	(D_DDPG_MIN_EXPLORE_PROBABILITY / ndReal(2.0f))

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDDPG_Trainer : public ndBrainAgent
{
	public:
	ndBrainAgentDDPG_Trainer(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic);
	virtual ~ndBrainAgentDDPG_Trainer();

	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;

	protected:
	void Step();
	void OptimizeStep();
	void Save(ndBrainSave* const loadSave) const;

	bool IsTerminal() const;
	ndReal GetReward() const;

	private:
	void PrintDebug();
	void BackPropagate();
	void SelectActions();
	void ApplyRandomAction() const;
	void SetBufferSize(ndInt32 size);
	ndInt32 GetOpmizationDelay() const;
	void SetOpmizationDelay(ndInt32 delay);
	void Optimize();
	void PopulateReplayBuffer();

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
			m_truth.SetCount(actionDim);
			m_inputBatch.SetCount(statesDim);
			m_outputBatch.SetCount(actionDim);
		}

		//virtual void GetGroundTruth(ndInt32 index, ndBrainVector& groundTruth, const ndBrainVector& output) const
		void EvaluateBellmanEquation(ndInt32 index)
		{
			ndAssert(m_truth.GetCount() == m_output.GetCount());
			ndAssert(m_truth.GetCount() == m_outputBatch.GetCount());
			const ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& transition = m_agent->m_replayBuffer[index];

			for (ndInt32 i = 0; i < statesDim; ++i)
			{
				m_inputBatch[i] = transition.m_state[i];
			}
			MakePrediction(m_inputBatch);

			for (ndInt32 i = 0; i < actionDim; ++i)
			{
				m_truth[i] = m_output[i];
				ndAssert(ndAbs(m_output[i]) < ndReal(100.0f));
			}

			ndInt32 action = transition.m_action[0];
			if (transition.m_terminalState)
			{
				m_truth[action] = transition.m_reward;
			}
			else
			{
				for (ndInt32 i = 0; i < statesDim; ++i)
				{
					m_inputBatch[i] = transition.m_nextState[i];
				}
				m_agent->m_targetActorInstance.MakePrediction(m_inputBatch, m_outputBatch);
				m_truth[action] = transition.m_reward + m_agent->m_gamma * m_outputBatch[action];
			}
		}

		virtual void Optimize(ndValidation&, const ndBrainMatrix&, ndReal, ndInt32)
		{
			ndArray<ndInt32>& shuffleBuffer = m_agent->m_replayBuffer.m_shuffleBuffer;

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
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>* m_agent;
	};

	ndSharedPtr<ndBrain> m_actor;
	ndSharedPtr<ndBrain> m_critic;
	ndBrain m_targetActor;
	ndBrain m_targetCritic;
	ndOptimizer m_actorOtimizer;
	ndOptimizer m_criticOtimizer;
	ndBrainInstance m_actorInstance;
	ndBrainInstance m_criticInstance;
	ndBrainInstance m_targetActorInstance;
	ndBrainInstance m_targetCriticInstance;

	ndBrainVector m_state;
	ndBrainVector m_actions;
	ndArray<ndInt32> m_movingAverage;
	ndBrainReplayBuffer<ndInt32, statesDim, 1> m_replayBuffer;
	ndBrainReplayTransitionMemory<ndInt32, statesDim, 1> m_currentTransition;

	ndReal m_gamma;
	ndReal m_learnRate;
	ndReal m_actionNoiseDeviation;
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
ndBrainAgentDDPG_Trainer<statesDim, actionDim>::ndBrainAgentDDPG_Trainer(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic)
	:ndBrainAgent()
	,m_actor(actor)
	,m_critic(critic)
	,m_targetActor(*(*m_actor))
	,m_targetCritic(*(*m_critic))
	,m_actorOtimizer(*m_actor)
	,m_criticOtimizer(*m_critic)
	,m_actorInstance(*m_actor)
	,m_criticInstance(*m_critic)
	,m_targetActorInstance(&m_targetActor)
	,m_targetCriticInstance(&m_targetCritic)
	,m_movingAverage()
	,m_replayBuffer()
	,m_gamma(D_DDPG_DISCOUNT_FACTOR)
	,m_learnRate(D_DDPG_LEARN_RATE)
	,m_actionNoiseDeviation(D_DDPG_ACTION_NOISE_DEVIATION)
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
	ndAssert(m_critic->GetOutputSize() == 1);
	ndAssert(((*(*m_actor))[m_actor->GetCount() - 1])->GetActivationType() == m_tanh);
	ndAssert(m_critic->GetInputSize() == (m_actor->GetInputSize() + m_actor->GetOutputSize()));

	m_state.SetCount(statesDim);
	m_actions.SetCount(actionDim);
	m_state.Set(ndReal(0.0f));
	m_actions.Set(ndReal(0.0f));

	m_actorOtimizer.m_agent = this;
	m_actorOtimizer.SetRegularizer(D_DDPG_REGULARIZER);
	m_explorationProbabilityAnnelining = (m_explorationProbability - m_minExplorationProbability) / D_DDPG_STAR_OPTIMIZATION;

	SetBufferSize(D_DDPG_REPLAY_BUFFERSIZE);
	m_targetActor.CopyFrom(*(*m_actor));

	for (ndInt32 i = 0; i < D_DDPG_MOVING_AVERAGE; ++i)
	{
		m_movingAverage.PushBack(0);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDDPG_Trainer<statesDim, actionDim>::~ndBrainAgentDDPG_Trainer()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetEposideCount() const
{
	return m_eposideCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::SetBufferSize(ndInt32 size)
{
	m_replayBuffer.SetSize(size);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::BackPropagate()
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
	ndTestValidator validator(m_actorOtimizer);
	m_actorOtimizer.Optimize(validator, inputBatch, m_learnRate, 1);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::SelectActions()
{
	ndFloat32 explore = ndRand();

	m_actorInstance.MakePrediction(m_state, m_actions);
	if (explore <= m_explorationProbability)
	{
		//for (ndInt32 i = 0; i < 5000; ++i)
		//{
		//	ndReal noisyAction = ndGaussianRandom(ndFloat32(0.0f), ndFloat32(m_actionNoiseDeviation));
		//	ndTrace(("%f\n", noisyAction));
		//}

		for (ndInt32 i = 0; i < actionDim; ++i)
		{
			ndReal noisyAction = ndGaussianRandom(ndFloat32(m_actions[i]), ndFloat32(m_actionNoiseDeviation));
			m_actions[i] = ndClamp(noisyAction, ndReal(-1.0f), ndReal(1.0f));
		}
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::ApplyRandomAction() const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::PrintDebug()
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
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave) const
{
	loadSave->Save(*m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDDPG_Trainer<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndReal(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetOpmizationDelay() const
{
	return m_optimizationDelay;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::SetOpmizationDelay(ndInt32 delay)
{
	m_optimizationDelay = delay;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::Step()
{
	GetObservation(&m_state[0]);
	m_actorInstance.MakePrediction(m_state, m_actions);

	SelectActions();
	ApplyActions(&m_actions[0]);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_state.Set(ndReal(0.0f));
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_actions.Set(ndReal(0.0f));
		ResetModel();
		m_currentTransition.Clear();
		m_optimizationDelayCount = 0;
	}

	if (m_optimizationDelayCount >= GetOpmizationDelay())
	{
		PopulateReplayBuffer();
		if (m_replayBuffer.GetCount() == m_replayBuffer.GetCapacity())
		{
			Optimize();
		}

		if (m_currentTransition.m_terminalState)
		{
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_state.Set(ndReal(0.0f));
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_actions.Set(ndReal(0.0f));
			ResetModel();
			m_currentTransition.Clear();
			if ((m_frameCount < m_startOptimization) && (m_eposideCount % 32 == 0))
			{
				ndExpandTraceMessage("collecting samples: frame %d out of %d, episode %d \n", m_frameCount, m_startOptimization, m_eposideCount);
			}
			m_eposideCount++;
			m_optimizationDelayCount = 0;
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::PrintDebug();
		}
	}
	else
	{
		ApplyRandomAction();
	}
	m_optimizationDelayCount++;

	m_frameCount++;
	m_framesAlive++;
	m_explorationProbability = ndMax(m_explorationProbability - m_explorationProbabilityAnnelining, m_minExplorationProbability);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::PopulateReplayBuffer()
{
	GetObservation(&m_currentTransition.m_nextState[0]);
	m_currentTransition.m_reward = GetReward();
	m_currentTransition.m_terminalState = IsTerminal();
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		m_currentTransition.m_state[i] = ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_state[i];
	}

	m_replayBuffer.AddTransition(m_currentTransition);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::Optimize()
{
	ndAssert(0);
	//BackPropagate();
	//if (m_collectingSamples)
	//{
	//	ndExpandTraceMessage("%d star training: episode %d\n", m_frameCount, m_eposideCount);
	//}
	//m_collectingSamples = false;
	//
	//if ((m_frameCount % m_targetUpdatePeriod) == (m_targetUpdatePeriod - 1))
	//{
	//	// update on line network
	//	m_target.CopyFrom(*(*m_actor));
	//}
}


#endif 

