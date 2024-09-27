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
#include "ndBrainLayer.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainLayerActivationTanh.h"
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
			m_numberOfHiddenLayers = 3;
			m_hiddenLayersNumberOfNeurons = 64;

			m_bashBufferSize = 64;
			m_replayBufferSize = 1024 * 512;
			m_targetUpdatePeriod = 1000;
			
			m_learnRate = ndBrainFloat(0.0001f);
			m_regularizer = ndBrainFloat(1.0e-6f);
			m_discountFactor = ndBrainFloat(0.99f);
			m_exploreMinProbability = ndBrainFloat(1.0f / 100.0f);
			m_exploreAnnelining = (m_exploreMinProbability / ndBrainFloat(2.0f));
			m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize);
			//m_threadsCount = 1;
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
		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_hiddenLayersNumberOfNeurons;
	};

	ndBrainAgentDQN_Trainer(const HyperParameters& hyperParameters);
	virtual ~ndBrainAgentDQN_Trainer();

	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;
	ndInt32 GetEpisodeFrames() const;

	protected:
	void Step();
	void OptimizeStep();
	bool IsTrainer() const;
	void Save(ndBrainSave* const loadSave);

	void InitWeights();
	bool IsSampling() const;
	bool IsTerminal() const;
	ndBrainFloat CalculateReward();

	private:
	void Optimize();
	void BackPropagate();
	void PopulateReplayBuffer();
	void SetBufferSize(ndInt32 size);
	void AddExploration(ndBrainFloat* const actions);

	protected:
	ndBrain m_policy;
	ndBrain m_target;
	ndBrainOptimizerAdam* m_optimizer;
	ndArray<ndBrainTrainer*> m_trainers;

	ndBrainReplayBuffer<statesDim, 1> m_replayBuffer;
	ndBrainReplayTransitionMemory<statesDim, 1> m_currentTransition;
	
	ndBrainFloat m_gamma;
	ndBrainFloat m_learnRate;
	ndBrainFloat m_explorationProbability;
	ndBrainFloat m_minExplorationProbability;
	ndBrainFloat m_explorationProbabilityAnnelining;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_targetUpdatePeriod;
	ndMovingAverage<1024> m_averageQvalue;
	ndMovingAverage<64> m_averageFramesPerEpisodes;
	bool m_collectingSamples;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN_Trainer<statesDim, actionDim>::ndBrainAgentDQN_Trainer(const HyperParameters& hyperParameters)
	:ndBrainAgent()
	,ndBrainThreadPool()
	,m_policy()
	,m_target()
	,m_optimizer(nullptr)
	,m_replayBuffer()
	,m_gamma(hyperParameters.m_discountFactor)
	,m_learnRate(hyperParameters.m_learnRate)
	,m_explorationProbability(ndBrainFloat(1.0f))
	,m_minExplorationProbability(hyperParameters.m_exploreMinProbability)
	,m_explorationProbabilityAnnelining(hyperParameters.m_exploreAnnelining)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_targetUpdatePeriod(hyperParameters.m_targetUpdatePeriod)
	,m_averageQvalue()
	,m_averageFramesPerEpisodes()
	,m_collectingSamples(true)
{
	// build neural net
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_neuronPerLayers));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, hyperParameters.m_neuronPerLayers));
		layers.PushBack(new ndBrainLayerActivationTanh(hyperParameters.m_neuronPerLayers));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, actionDim));

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_policy.AddLayer(layers[i]);
		m_target.AddLayer(layers[i]->Clone());
	}

	m_trainers.SetCount(0);
	SetThreadCount(hyperParameters.m_threadsCount);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_policy);
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
	m_policy.InitWeights();
	m_target.CopyFrom(m_policy);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN_Trainer<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
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

				ndBrainFixSizeVector<actionDim> action;
				
				const ndBrainReplayTransitionMemory<statesDim, 1>& transition = m_agent->m_replayBuffer[m_index];
				action.Set(output);
				ndInt32 actionIndex = ndInt32(transition.m_action[0]);
				if (transition.m_terminalState)
				{
					action[actionIndex] = transition.m_reward;
				}
				else
				{
					ndBrainFixSizeVector<actionDim> nextAction;
					m_agent->m_target.MakePrediction(transition.m_nextObservation, nextAction);
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
			trainer.BackPropagate(transition.m_observation, loss);
		}
	});

	ndBrainThreadPool::ParallelExecute(PropagateBash);
	m_optimizer->Update(this, m_trainers, m_learnRate);
	
	if ((m_frameCount % m_targetUpdatePeriod) == (m_targetUpdatePeriod - 1))
	{
		// update on line network
		m_target.CopyFrom(m_policy);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave)
{
	loadSave->Save(&m_policy);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDQN_Trainer<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainFloat ndBrainAgentDQN_Trainer<statesDim, actionDim>::CalculateReward()
{
	ndAssert(0);
	return ndBrainFloat (0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::PopulateReplayBuffer()
{
	GetObservation(&m_currentTransition.m_nextObservation[0]);
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

		// using soft max and cumulative distribution instead
		// seem like a good idead but in fact si really bad. 
		//ndBrainFloat max = ndBrainFloat(1.0e-16f);
		//for (ndInt32 i = actionDim - 1; i >= 0; --i)
		//{
		//	max = ndMax(actions[i], max);
		//}
		//ndBrainFixSizeVector<actionDim + 1> pdf;
		//pdf.SetCount(0);
		//ndBrainFloat acc = ndBrainFloat(0.0f);
		//for (ndInt32 i = 0; i < actionDim; ++i)
		//{
		//	ndBrainFloat in = ndMax((actions[i] - max), ndBrainFloat(-30.0f));
		//	ndAssert(in <= ndBrainFloat(0.0f));
		//	ndBrainFloat prob = ndBrainFloat(ndExp(in));
		//	pdf.PushBack(acc);
		//	acc += prob;
		//}
		//pdf.PushBack(acc);
		//pdf.Scale(ndBrainFloat(1.0f) / acc);
		//
		//ndFloat32 r = ndRand();
		//action = actionDim - 1;
		//for (ndInt32 i = actionDim - 1; i >= 0; --i)
		//{
		//	action = i;
		//	if (pdf[i] < r)
		//	{
		//		break;
		//	}
		//}
	}
	else
	{
		action = qActionValues.ArgMax();
	}
	
	if (!IsSampling())
	{
		m_averageQvalue.Update(qActionValues[action]);
	}
	m_currentTransition.m_action[0] = ndBrainFloat(action);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN_Trainer<statesDim, actionDim>::Step()
{
	ndBrainFixSizeVector<actionDim> actions;

	GetObservation(&m_currentTransition.m_observation[0]);
	m_policy.MakePrediction(m_currentTransition.m_observation, actions);

	AddExploration(&actions[0]);
	ndBrainFloat bestAction = ndBrainFloat(m_currentTransition.m_action[0]);
	
	ApplyActions(&bestAction);
	if (bestAction != ndBrainFloat(m_currentTransition.m_action[0]))
	{
		m_currentTransition.m_action[0] = bestAction;
	}
	
	m_currentTransition.m_reward = CalculateReward();
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

		if (!IsSampling())
		{
			m_averageFramesPerEpisodes.Update(ndBrainFloat(m_framesAlive));
		}
		m_eposideCount++;
		m_framesAlive = 0;
	}

	m_frameCount++;
	m_framesAlive++;
	m_explorationProbability = ndMax(m_explorationProbability - m_explorationProbabilityAnnelining, m_minExplorationProbability);
}

#endif 