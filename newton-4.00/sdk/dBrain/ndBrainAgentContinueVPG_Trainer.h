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

#ifndef _ND_AGENT_CONTINUE_VPG_H_TRAINER_H__
#define _ND_AGENT_CONTINUE_VPG_H_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"
#include "ndBrainLossLeastSquaredError.h"

// this is an implementation of the vanilla policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/vpg.html

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentContinueVPG_Trainer : public ndBrainAgent, public ndBrainThreadPool
{
	public:
	
	class HyperParameters
	{
		public:
		HyperParameters()
		{
			m_bashBufferSize = 32;
			m_numberOfHiddenLayers = 3;
			m_maxTrajectorySteps = 1024 * 2;
			m_extraTrajectorySteps = 1024 * 2;
			
			m_hiddenLayersNumberOfNeurons = 64;
			//m_hiddenLayersNumberOfNeurons = 64 + 3;

			m_sigma = ndBrainFloat(0.5f);
			m_learnRate = ndBrainFloat(0.0005f);
			m_regularizer = ndBrainFloat(1.0e-6f);
			m_discountFactor = ndBrainFloat(0.99f);
			m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), ndMin(m_bashBufferSize, 16));
			//m_threadsCount = 1;
		}

		ndBrainFloat m_sigma;
		ndBrainFloat m_learnRate;
		ndBrainFloat m_regularizer;
		ndBrainFloat m_discountFactor;

		ndInt32 m_threadsCount;
		ndInt32 m_bashBufferSize;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_extraTrajectorySteps;
		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_hiddenLayersNumberOfNeurons;
	};

	class ndTrajectoryStep
	{
		public:
		ndTrajectoryStep()
			:m_actions()
			,m_observation()
			,m_reward(ndBrainFloat(0.0f))
		{
		}

		ndTrajectoryStep(const ndTrajectoryStep& src)
			:m_actions(src.m_actions)
			,m_observation(src.m_observation)
			,m_reward(src.m_reward)
		{
		}

		ndTrajectoryStep& operator=(const ndTrajectoryStep& src)
		{
			new (this) ndTrajectoryStep(src);
			return*this;
		}

		ndBrainFixSizeVector<actionDim> m_actions;
		ndBrainFixSizeVector<statesDim> m_observation;
		ndBrainFloat m_reward;
	};

	ndBrainAgentContinueVPG_Trainer(const HyperParameters& hyperParameters);
	virtual ~ndBrainAgentContinueVPG_Trainer();

	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;
	ndInt32 GetEpisodeFrames() const;

	bool IsTrainer() const;

	protected:
	void Step();
	void OptimizeStep();

	void Save(ndBrainSave* const loadSave) const;

	void InitWeights();
	void InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance);

	bool IsSampling() const;
	bool IsTerminal() const;
	ndBrainFloat GetReward() const;

	private:
	void Optimize();
	void BackPropagate();

	void CalcucateRewards();
	void SelectAction(ndBrainVector& probabilities) const;

	protected:
	ndBrain m_actor;
	ndBrainOptimizerAdam* m_optimizer;
	ndArray<ndBrainTrainer*> m_trainers;
	ndArray<ndBrainTrainer*> m_weightedTrainer;
	ndArray<ndBrainTrainer*> m_auxiliaryTrainers;

	ndArray<ndTrajectoryStep> m_trajectory;
	ndBrainVector m_rewards;

	ndBrainFloat m_sigma;
	ndBrainFloat m_gamma;
	ndBrainFloat m_learnRate;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_maxTrajectorySteps;
	ndInt32 m_extraTrajectorySteps;
	ndMovingAverage<128> m_averageQvalue;
	ndMovingAverage<128> m_averageFramesPerEpisodes;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::ndBrainAgentContinueVPG_Trainer(const HyperParameters& hyperParameters)
	:ndBrainAgent()
	,ndBrainThreadPool()
	,m_actor()
	,m_trajectory()
	,m_rewards()
	,m_sigma(hyperParameters.m_sigma)
	,m_gamma(hyperParameters.m_discountFactor)
	,m_learnRate(hyperParameters.m_learnRate)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_averageQvalue()
	,m_averageFramesPerEpisodes()
{
	// build neural net
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, hyperParameters.m_hiddenLayersNumberOfNeurons));
		//layers.PushBack(new ndBrainLayerTanhActivation(hyperParameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerApproximateTanhActivation(hyperParameters.m_hiddenLayersNumberOfNeurons));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, m_actionsSize));
	//layers.PushBack(new ndBrainLayerSoftmaxActivation(m_actionsSize));
	layers.PushBack(new ndBrainLayerTanhActivation(m_actionsSize));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_actor.AddLayer(layers[i]);
	}
	InitWeights();

	m_trainers.SetCount(0);
	m_auxiliaryTrainers.SetCount(0);
	SetThreadCount(hyperParameters.m_threadsCount);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_actor);
		m_trainers.PushBack(trainer);

		ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(&m_actor);
		m_auxiliaryTrainers.PushBack(auxiliaryTrainer);
	}

	m_weightedTrainer.PushBack(m_trainers[0]);
	m_optimizer = new ndBrainOptimizerAdam();
	m_optimizer->SetRegularizer(hyperParameters.m_regularizer);

	m_rewards.SetCount(m_maxTrajectorySteps + m_extraTrajectorySteps);
	m_trajectory.SetCount(m_maxTrajectorySteps + m_extraTrajectorySteps);
	m_trajectory.SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::~ndBrainAgentContinueVPG_Trainer()
{
	for (ndInt32 i = 0; i < m_trainers.GetCount(); ++i)
	{
		delete m_trainers[i];
		delete m_auxiliaryTrainers[i];
	}
	delete m_optimizer;
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::IsTrainer() const
{
	return true;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::InitWeights()
{
	m_actor.InitWeightsXavierMethod();
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance)
{
	ndAssert(0);
	m_actor.InitWeights(weighVariance, biasVariance);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::IsSampling() const
{
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::GetEposideCount() const
{
	return m_eposideCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::GetEpisodeFrames() const
{
	return m_framesAlive;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::BackPropagate()
{
	auto ClearGradients = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		const ndStartEnd startEnd(m_trainers.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBrainTrainer* const trainer = m_trainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndMin(m_maxTrajectorySteps, m_trajectory.GetCount());
	const ndBrainFloat invSigmaSquare = ndBrainFloat(1.0f) / (m_sigma * m_sigma);

	for (ndInt32 base = 0; base < steps; base += m_bashBufferSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, base, invSigmaSquare](ndInt32 threadIndex, ndInt32 threadCount)
		{
			class Loss : public ndBrainLossLeastSquaredError
			{
				public:
				Loss(ndBrainTrainer& trainer, ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>* const agent, ndInt32 index, ndBrainFloat invSigmaSquare)
					:ndBrainLossLeastSquaredError(trainer.GetBrain()->GetOutputSize())
					,m_trainer(trainer)
					,m_agent(agent)
					,m_invSigmaSquare(invSigmaSquare)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					const ndBrainVector& rewards = m_agent->m_rewards;
					const ndBrainVector& actions = m_agent->m_trajectory[m_index].m_actions;
					//ndBrainFloat logProbAdvantage = rewards[m_index] / (SIGMA * SIGMA);
					ndBrainFloat logProbAdvantage = rewards[m_index] * m_invSigmaSquare;
					for (ndInt32 i = actionDim - 1; i >= 0; --i)
					{
						loss[i] = logProbAdvantage * (actions[i] - output[i]);
					}
				}
	
				ndBrainTrainer& m_trainer;
				ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>* m_agent;
				const ndBrainFloat m_invSigmaSquare;
				ndInt32 m_index;
			};

			ndBrainFixSizeVector<statesDim> observations;
			const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBrainTrainer& trainer = *m_auxiliaryTrainers[i];
				Loss loss(trainer, this, base + i, invSigmaSquare);
				if ((base + i) < m_trajectory.GetCount())
				{
					trainer.BackPropagate(m_trajectory[base + i].m_observation, loss);
				}
				else
				{
					trainer.ClearGradients();
				}
			}
		});
		ndBrainThreadPool::ParallelExecute(CalculateGradients);

		auto AddGradients = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
		{
			const ndStartEnd startEnd(m_trainers.GetCount(), threadIndex, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBrainTrainer* const trainer = m_trainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_auxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}

	m_optimizer->AccumulateGradients(this, m_trainers);
	m_weightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectory.GetCount()));
	m_optimizer->Update(this, m_weightedTrainer, -m_learnRate);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave) const
{
	loadSave->Save(&m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainFloat ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::CalcucateRewards()
{
	const ndInt32 steps = m_trajectory.GetCount();

	// using the Bellman equation.
	m_rewards.SetCount(steps);
	m_rewards[steps - 1] = m_trajectory[steps - 1].m_reward;
	for (ndInt32 i = steps - 2; i >= 0; --i)
	{
		m_rewards[i] = m_trajectory[i].m_reward + m_gamma * m_rewards[i + 1];
	}

	m_averageQvalue.Update(ndReal(m_rewards[0]));
	m_averageFramesPerEpisodes.Update(ndReal(steps));
	m_rewards.GaussianNormalize();
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Optimize()
{
	CalcucateRewards();
	BackPropagate();
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::SelectAction(ndBrainVector& probabilities) const
{
	// for now use a constant deviations until the algorithm is stable 
	for (ndInt32 i = actionDim - 1; i >= 0; --i)
	{
		//ndBrainFloat sample = ndGaussianRandom(probabilities[i], SIGMA);
		ndBrainFloat sample = ndGaussianRandom(probabilities[i], m_sigma);
		ndBrainFloat squashSample = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		probabilities[i] = squashSample;
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Step()
{
	ndAssert(0);
	//ndTrajectoryStep trajectoryStep;
	//
	//GetObservation(&trajectoryStep.m_observation[0]);
	//m_actor.MakePrediction(trajectoryStep.m_observation, trajectoryStep.m_actions);
	//SelectAction(trajectoryStep.m_actions);
	//ApplyActions(&trajectoryStep.m_actions[0]);
	//trajectoryStep.m_reward = GetReward();
	//
	//ndAssert(m_trajectory.GetCount() < m_trajectory.GetCapacity());
	//m_trajectory.PushBack(trajectoryStep);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ResetModel();
	}

	bool isTeminal = IsTerminal() || (m_trajectory.GetCount() == (m_extraTrajectorySteps + m_maxTrajectorySteps));
	if (isTeminal)
	{
		Optimize();
		ResetModel();
		m_trajectory.SetCount(0);
		m_eposideCount++;
		m_framesAlive = 0;
	}
	
	m_frameCount++;
	m_framesAlive++;
}

#endif 