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

#ifndef _ND_BRAIN_AGENT_DQN_TRAINER_H__
#define _ND_BRAIN_AGENT_DQN_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"
#include "ndBrainLossLeastSquaredError.h"

// this is an implementation of the vanilla dqn agent trainer as described 
// on the nature paper below. 
// https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDQN_Trainer: public ndBrainAgent, public ndBrainThreadPool
{
	public: 
	class HyperParameters
	{
		public:
		HyperParameters()
		{
			m_discountFactor = ndBrainFloat(0.99f);
			m_regularizer = ndBrainFloat(1.0e-6f);
			m_learnRate = ndBrainFloat(0.001f);
			m_bashBufferSize = 64;
			m_replayBufferSize = 1024 * 512;
			m_targetUpdatePeriod = 1000;

			m_exploreMinProbability = ndBrainFloat(1.0f / 100.0f);
			m_exploreAnnelining = (m_exploreMinProbability / ndBrainFloat(2.0f));
			m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize / 4);
		}

		ndBrainFloat m_learnRate;
		ndBrainFloat m_regularizer;
		ndBrainFloat m_discountFactor;
		ndBrainFloat m_exploreAnnelining;
		ndBrainFloat m_exploreMinProbability;

		ndInt32 m_threadsCount;
		ndInt32 m_bashBufferSize;
		ndInt32 m_replayBufferSize;
		ndInt32 m_targetUpdatePeriod;
		ndInt32 m_replayBufferPrefill;
	};

	ndBrainAgentDQN_Trainer(const HyperParameters& hyperParameters, const ndBrain& actor);
	virtual ~ndBrainAgentDQN_Trainer();

	ndBrainFloat GetCurrentValue() const;
	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;
	ndInt32 GetEpisodeFrames() const;

	protected:
	void Step();
	void OptimizeStep();
	bool IsTrainer() const;
	void Save(ndBrainSave* const loadSave) const;

	void InitWeights();
	void InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance);

	bool IsSampling() const;
	bool IsTerminal() const;
	ndBrainFloat GetReward() const;
	void AddExploration(ndBrainFloat* const actions);

	private:
	void Optimize();
	void BackPropagate();
	void PopulateReplayBuffer();
	void SetBufferSize(ndInt32 size);

	protected:
	ndBrain m_actor;
	ndBrain m_target;
	ndBrainOptimizerAdam* m_optimizer;
	ndArray<ndBrainTrainer*> m_trainers;

	ndBrainReplayBuffer<statesDim, 1> m_replayBuffer;
	ndBrainReplayTransitionMemory<statesDim, 1> m_currentTransition;
	
	ndBrainFloat m_gamma;
	ndBrainFloat m_learnRate;
	ndBrainFloat m_currentQValue;
	ndBrainFloat m_explorationProbability;
	ndBrainFloat m_minExplorationProbability;
	ndBrainFloat m_explorationProbabilityAnnelining;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_targetUpdatePeriod;
	bool m_collectingSamples;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN_Trainer<statesDim, actionDim>::ndBrainAgentDQN_Trainer(const HyperParameters& hyperParameters, const ndBrain& actor)
	:ndBrainAgent()
	,ndBrainThreadPool()
	,m_actor(actor)
	,m_target(actor)
	,m_optimizer(nullptr)
	,m_replayBuffer()
	,m_gamma(hyperParameters.m_discountFactor)
	,m_learnRate(hyperParameters.m_learnRate)
	,m_currentQValue(ndBrainFloat(0.0f))
	,m_explorationProbability(ndBrainFloat(1.0f))
	,m_minExplorationProbability(hyperParameters.m_exploreMinProbability)
	,m_explorationProbabilityAnnelining(hyperParameters.m_exploreAnnelining)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_targetUpdatePeriod(hyperParameters.m_targetUpdatePeriod)
	,m_collectingSamples(true)
{
	m_trainers.SetCount(0);
	SetThreadCount(hyperParameters.m_threadsCount);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_actor);
		m_trainers.PushBack(trainer);
	}

	SetBufferSize(hyperParameters.m_replayBufferSize);
	m_explorationProbabilityAnnelining = (m_explorationProbability - m_minExplorationProbability) / ndBrainFloat (m_replayBuffer.GetCapacity());

	InitWeights();
	m_optimizer = new ndBrainOptimizerAdam();
	m_optimizer->SetRegularizer(hyperParameters.m_regularizer);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN_Trainer<statesDim, actionDim>::~ndBrainAgentDQN_Trainer()
{
	for (ndInt32 i = 0; i < m_trainers.GetCount(); ++i)
	{
		delete m_trainers[i];
	}
	delete m_optimizer;
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDQN_Trainer<statesDim, actionDim>::IsTrainer() const
{
	return true;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::InitWeights()
{
	m_actor.InitWeightsXavierMethod();
	m_target.CopyFrom(m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance)
{
	m_actor.InitWeights(weighVariance, biasVariance);
	m_target.CopyFrom(m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainFloat ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetCurrentValue() const
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

	auto PropagateBash = ndMakeObject::ndFunction([this, &shuffleBuffer](ndInt32 threadIndex, ndInt32 threadCount)
	{
		class Loss: public ndBrainLossLeastSquaredError
		{
			public:
			Loss(ndBrainTrainer& trainer, ndBrainAgentDQN_Trainer<statesDim, actionDim>* const agent)
				:ndBrainLossLeastSquaredError(trainer.GetBrain()->GetOutputSize())
				,m_trainer(trainer)
				,m_agent(agent)
				,m_index(0)
			{
			}

			void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(output.GetCount() == actionDim);
				ndAssert(m_truth.GetCount() == m_trainer.GetBrain()->GetOutputSize());

				ndBrainFloat actionBuffer[actionDim * 2];
				ndBrainMemVector action(actionBuffer, actionDim);
				
				const ndBrainReplayTransitionMemory<statesDim, 1>& transition = m_agent->m_replayBuffer[m_index];
				action.Set(output);
				ndInt32 actionIndex = ndInt32(transition.m_action[0]);
				if (transition.m_terminalState)
				{
					action[actionIndex] = transition.m_reward;
				}
				else
				{
					ndBrainFloat nextActionBuffer[actionDim * 2];
					ndBrainMemVector nextAction(nextActionBuffer, actionDim);
					m_agent->m_target.MakePrediction(transition.m_nextState, nextAction);
					action[actionIndex] = transition.m_reward + m_agent->m_gamma * nextAction[actionIndex];
				}
				
				SetTruth(action);
				ndBrainLossLeastSquaredError::GetLoss(output, loss);
			}

			ndBrainTrainer& m_trainer;
			ndBrainAgentDQN_Trainer<statesDim, actionDim>* m_agent;
			ndInt32 m_index;
		};

		ndAssert(m_bashBufferSize == m_trainers.GetCount());
		const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBrainTrainer& trainer = *m_trainers[i];
			Loss loss(trainer, this);
			ndInt32 index = ndInt32 (shuffleBuffer[i]);
			const ndBrainReplayTransitionMemory<statesDim, 1>& transition = m_replayBuffer[index];
			loss.m_index = index;
			trainer.BackPropagate(transition.m_state, loss);
		}
	});

	ndBrainThreadPool::ParallelExecute(PropagateBash);
	m_optimizer->Update(this, m_trainers, m_learnRate);
	
	if ((m_frameCount % m_targetUpdatePeriod) == (m_targetUpdatePeriod - 1))
	{
		// update on line network
		m_target.CopyFrom(m_actor);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave) const
{
	loadSave->Save(&m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDQN_Trainer<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainFloat ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndBrainFloat (0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::PopulateReplayBuffer()
{
	GetObservation(&m_currentTransition.m_nextState[0]);
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
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::AddExploration(ndBrainFloat* const actions)
{
	ndInt32 action = 0;
	ndFloat32 explore = ndRand();
	
	const ndBrainMemVector qActionValues(actions, actionDim);
	if (explore <= m_explorationProbability)
	{
		// explore environment
		ndUnsigned32 randomIndex = ndRandInt();
		action = ndInt32(randomIndex % actionDim);
	}
	else
	{
		action = qActionValues.ArgMax();
	}
	
	m_currentQValue = qActionValues[action];
	m_currentTransition.m_action[0] = ndBrainFloat(action);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::Step()
{
	ndBrainFloat actionBuffer[actionDim * 2];
	ndBrainMemVector actions(actionBuffer, actionDim);

	GetObservation(&m_currentTransition.m_state[0]);
	m_actor.MakePrediction(m_currentTransition.m_state, actions);

	AddExploration(&actions[0]);
	ndBrainFloat bestAction = ndBrainFloat(m_currentTransition.m_action[0]);
	
	ApplyActions(&bestAction);
	if (bestAction != ndBrainFloat(m_currentTransition.m_action[0]))
	{
		m_currentTransition.m_action[0] = bestAction;
	}
	
	m_currentTransition.m_reward = GetReward();
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
		ResetModel();
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