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

#ifndef _ND_BRAIN_AGENT_DESCRETE_VPG_TRAINER_H__
#define _ND_BRAIN_AGENT_DESCRETE_VPG_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainLayer.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainThreadPool.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerTanhActivation.h"
#include "ndBrainLossLeastSquaredError.h"

// this is an implementation of the vanilla policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/vpg.html

template<ndInt32 statesDim, ndInt32 actionDim> class ndBrainAgentDiscretePolicyGradient_TrainerMaster;

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDiscretePolicyGradient_Trainer : public ndBrainAgent
{
	public:
	class ndTrajectoryStep
	{
		public:
		ndTrajectoryStep()
			:m_reward(ndBrainFloat(0.0f))
			,m_action(ndBrainFloat(0.0f))
			,m_observation()
		{
		}

		ndTrajectoryStep(const ndTrajectoryStep& src)
			:m_reward(src.m_reward)
			,m_action(src.m_action)
		{
			ndMemCpy(m_observation, src.m_observation, statesDim);
		}

		ndTrajectoryStep& operator=(const ndTrajectoryStep& src)
		{
			new (this) ndTrajectoryStep(src);
			return *this;
		}

		ndBrainFloat m_reward;
		ndBrainFloat m_action;
		ndBrainFloat m_observation[statesDim];
	};

	ndBrainAgentDiscretePolicyGradient_Trainer(ndSharedPtr<ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>>& master);
	~ndBrainAgentDiscretePolicyGradient_Trainer();

	ndBrain* GetActor();
	ndBrainFloat SelectAction(const ndBrainVector& probabilities) const;

	void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	void Save(ndBrainSave* const) { ndAssert(0); }
	bool IsTrainer() const { ndAssert(0); return true; }
	void InitWeights(ndBrainFloat, ndBrainFloat) { ndAssert(0); }

	bool IsTerminal() const;
	ndInt32 GetEpisodeFrames() const;

	virtual void Step();

	ndBrainVector m_workingBuffer;
	ndArray<ndTrajectoryStep> m_trajectory;
	ndSharedPtr<ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>> m_master;

	mutable std::random_device m_rd;
	mutable std::mt19937 m_gen;
	mutable std::uniform_real_distribution<ndFloat32> m_d;

	friend class ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>;
};

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDiscretePolicyGradient_TrainerMaster : public ndBrainThreadPool
{
	public:
	class HyperParameters
	{
		public:
		HyperParameters()
		{
			m_randomSeed = 47;
			m_numberOfLayers = 4;
			m_bashBufferSize = 64;
			m_neuronPerLayers = 64;
			m_bashTrajectoryCount = 100;
			m_maxTrajectorySteps = 4096;
			m_extraTrajectorySteps = 1024;
			m_baseLineOptimizationPases = 4;

			m_criticLearnRate = ndBrainFloat(0.0005f);
			m_policyLearnRate = ndBrainFloat(0.0005f);
			m_regularizer = ndBrainFloat(1.0e-6f);
			m_discountFactor = ndBrainFloat(0.99f);
			m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize);
		}

		ndBrainFloat m_policyLearnRate;
		ndBrainFloat m_criticLearnRate;
		ndBrainFloat m_regularizer;
		ndBrainFloat m_discountFactor;

		ndInt32 m_threadsCount;
		ndInt32 m_numberOfLayers;
		ndInt32 m_bashBufferSize;
		ndInt32 m_neuronPerLayers;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_bashTrajectoryCount;
		ndInt32 m_extraTrajectorySteps;
		ndInt32 m_baseLineOptimizationPases;
		ndUnsigned32 m_randomSeed;
	};

	ndBrainAgentDiscretePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters);
	virtual ~ndBrainAgentDiscretePolicyGradient_TrainerMaster();

	ndBrain* GetActor();
	ndBrain* GetCritic();

	const ndString& GetName() const;
	void SetName(const ndString& name);

	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;

	bool IsSampling() const;
	ndFloat32 GetAverageScore() const;
	ndFloat32 GetAverageFrames() const;

	void OptimizeStep();

	private:
	void Optimize();
	void OptimizePolicy();
	void OptimizeCritic();
	void SaveTrajectory(ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>* const agent);

	ndBrain m_actor;
	ndBrain m_baseLineValue;
	ndBrainOptimizerAdam* m_optimizer;
	ndArray<ndBrainTrainer*> m_trainers;
	ndArray<ndBrainTrainer*> m_weightedTrainer;
	ndArray<ndBrainTrainer*> m_auxiliaryTrainers;
	ndBrainOptimizerAdam* m_baseLineValueOptimizer;
	ndArray<ndInt32> m_randomPermutation;
	ndArray<ndBrainTrainer*> m_baseLineValueTrainers;
	ndArray<typename ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::ndTrajectoryStep> m_trajectoryAccumulator;

	ndBrainFloat m_gamma;
	ndBrainFloat m_policyLearnRate;
	ndBrainFloat m_criticLearnRate;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_maxTrajectorySteps;
	ndInt32 m_extraTrajectorySteps;
	ndInt32 m_bashTrajectoryIndex;
	ndInt32 m_bashTrajectorySteps;
	ndInt32 m_baseLineOptimizationPases;
	ndInt32 m_baseValueWorkingBufferSize;
	ndUnsigned32 m_randomSeed;
	ndBrainVector m_workingBuffer;
	ndMovingAverage<8> m_averageScore;
	ndMovingAverage<8> m_averageFramesPerEpisodes;
	ndString m_name;
	ndList<ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>*> m_agents;

	friend class ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::ndBrainAgentDiscretePolicyGradient_Trainer(ndSharedPtr<ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>>& master)
	:ndBrainAgent()
	,m_workingBuffer()
	,m_trajectory()
	,m_master(master)
	,m_rd()
	,m_gen(m_rd())
	,m_d(ndFloat32(0.0f), ndFloat32(1.0f))
{
	m_gen.seed(m_master->m_randomSeed);
	m_master->m_randomSeed += 1;

	m_master->m_agents.Append(this);
	m_trajectory.SetCount(m_master->m_maxTrajectorySteps + m_master->m_extraTrajectorySteps);
	m_trajectory.SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::~ndBrainAgentDiscretePolicyGradient_Trainer()
{
	for (typename ndList<ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>*>::ndNode* node = m_master->m_agents.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo() == this)
		{
			m_master->m_agents.Remove(node);
			break;
		}
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrain* ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::GetActor()
{
	return m_master->GetActor();
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::IsTerminal() const
{
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::GetEpisodeFrames() const 
{ 
	return m_trajectory.GetCount();
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::SelectAction(const ndBrainVector& probabilities) const
{
	ndBrainFixSizeVector<actionDim + 1> pdf;

	pdf.SetCount(0);
	ndBrainFloat sum = ndBrainFloat(0.0f);
	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		pdf.PushBack(sum);
		sum += probabilities[i];
	}
	pdf.PushBack(sum);

	//ndFloat32 r = ndRand();
	ndFloat32 r = m_d(m_gen);
	ndInt32 index = actionDim - 1;
	for (ndInt32 i = actionDim - 1; i >= 0; --i)
	{
		index = i;
		if (pdf[i] < r)
		{
			break;
		}
	}
	return ndBrainFloat(index);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::Step()
{
	ndBrainFixSizeVector<actionDim> probability;
	ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::ndTrajectoryStep trajectoryStep;
	ndBrainMemVector observation(&trajectoryStep.m_observation[0], statesDim);

	GetObservation(&observation[0]);
	m_master->m_actor.MakePrediction(observation, probability, m_workingBuffer);

	trajectoryStep.m_action = SelectAction(probability);
	ApplyActions(&trajectoryStep.m_action);
	trajectoryStep.m_reward = CalculateReward();

	ndAssert(m_trajectory.GetCount() < m_trajectory.GetCapacity());
	m_trajectory.PushBack(trajectoryStep);
}

// ***************************************************************************************
//
// ***************************************************************************************
template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::ndBrainAgentDiscretePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainThreadPool()
	,m_actor()
	,m_baseLineValue()
	,m_optimizer(nullptr)
	,m_trainers()
	,m_weightedTrainer()
	,m_auxiliaryTrainers()
	,m_baseLineValueOptimizer(nullptr)
	,m_baseLineValueTrainers()
	,m_trajectoryAccumulator()
	,m_gamma(hyperParameters.m_discountFactor)
	,m_policyLearnRate(hyperParameters.m_policyLearnRate)
	,m_criticLearnRate(hyperParameters.m_criticLearnRate)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_bashTrajectoryIndex(0)
	,m_bashTrajectorySteps(hyperParameters.m_bashTrajectoryCount * m_maxTrajectorySteps)
	,m_baseLineOptimizationPases(hyperParameters.m_baseLineOptimizationPases)
	,m_baseValueWorkingBufferSize(0)
	,m_randomSeed(hyperParameters.m_randomSeed)
	,m_workingBuffer()
	,m_averageScore()
	,m_averageFramesPerEpisodes()
	,m_agents()
{
	ndSetRandSeed(m_randomSeed);

	// build policy neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_neuronPerLayers));
	layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, hyperParameters.m_neuronPerLayers));
		layers.PushBack(new ndBrainLayerTanhActivation(hyperParameters.m_neuronPerLayers));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, actionDim));
	layers.PushBack(new ndBrainLayerSoftmaxActivation(actionDim));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_actor.AddLayer(layers[i]);
	}
	m_actor.InitWeightsXavierMethod();
	ndAssert(!strcmp((m_actor[m_actor.GetCount() - 1])->GetLabelId(), "ndBrainLayerSoftmaxActivation"));

	m_trainers.SetCount(0);
	m_auxiliaryTrainers.SetCount(0);
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

	// build state value critic neural net
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_neuronPerLayers * 2));
	layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers * 2);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), hyperParameters.m_neuronPerLayers * 2));
		layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_baseLineValue.AddLayer(layers[i]);
	}
	m_baseLineValue.InitWeightsXavierMethod();

	ndAssert(m_baseLineValue.GetOutputSize() == 1);
	ndAssert(m_baseLineValue.GetInputSize() == m_actor.GetInputSize());
	ndAssert(!strcmp((m_baseLineValue[m_baseLineValue.GetCount() - 1])->GetLabelId(), "ndBrainLayerLinear"));

	m_baseLineValueTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_baseLineValue);
		m_baseLineValueTrainers.PushBack(trainer);
	}

	m_baseLineValueOptimizer = new ndBrainOptimizerAdam();
	//m_baseLineValueOptimizer->SetRegularizer(hyperParameters.m_regularizer);
	m_baseLineValueOptimizer->SetRegularizer(ndBrainFloat(1.0e-4f));

	m_baseValueWorkingBufferSize = m_baseLineValue.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::~ndBrainAgentDiscretePolicyGradient_TrainerMaster()
{
	for (ndInt32 i = 0; i < m_trainers.GetCount(); ++i)
	{
		delete m_trainers[i];
		delete m_auxiliaryTrainers[i];
	}
	delete m_optimizer;

	for (ndInt32 i = 0; i < m_baseLineValueTrainers.GetCount(); ++i)
	{
		delete m_baseLineValueTrainers[i];
	}
	delete m_baseLineValueOptimizer;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrain* ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::GetActor()
{
	return &m_actor;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrain* ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::GetCritic()
{
	return &m_baseLineValue;
}

template<ndInt32 statesDim, ndInt32 actionDim>
const ndString& ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::GetName() const
{
	return m_name;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::SetName(const ndString& name)
{
	m_name = name;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::IsSampling() const
{
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::GetEposideCount() const
{
	return m_eposideCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndFloat32 ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndFloat32 ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::GetAverageScore() const
{
	return m_averageScore.GetAverage();
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::SaveTrajectory(ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>* const agent)
{
	// remove last step because if it was a dead state, it will provide misleading feedback.
	agent->m_trajectory.SetCount(agent->m_trajectory.GetCount() - 1);

	// get the max trajectory steps
	const ndInt32 maxSteps = ndMin(agent->m_trajectory.GetCount(), m_maxTrajectorySteps);
	ndAssert(maxSteps > 0);

	// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
	for (ndInt32 i = agent->m_trajectory.GetCount() - 2; i >= 0; --i)
	{
		agent->m_trajectory[i].m_reward += m_gamma * agent->m_trajectory[i + 1].m_reward;
	}

	for (ndInt32 i = 0; i < maxSteps; ++i)
	{
		m_trajectoryAccumulator.PushBack(agent->m_trajectory[i]);
	}
	agent->m_trajectory.SetCount(0);
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::OptimizeCritic()
{
	m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount());
	for (ndInt32 i = m_trajectoryAccumulator.GetCount() - 1; i >= 0; --i)
	{
		m_randomPermutation[i] = i;
	}

	ndAtomic<ndInt32> iterator(0);
	const ndInt32 maxSteps = (m_trajectoryAccumulator.GetCount() & -m_bashBufferSize) - m_bashBufferSize;
	for (ndInt32 passes = 0; passes < m_baseLineOptimizationPases; ++passes)
	{
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
		for (ndInt32 base = 0; base < maxSteps; base += m_bashBufferSize)
		{
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				class ndPolicyLoss : public	ndBrainLossLeastSquaredError
				{
					public:
					ndPolicyLoss()
						:ndBrainLossLeastSquaredError(1)
					{
					}

					void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
					{
						ndBrainLossLeastSquaredError::GetLoss(output, loss);
					}
				};

				//class ndPolicyLoss : public ndBrainLossHuber
				//{
				//	public:
				//	ndPolicyLoss()
				//		:ndBrainLossHuber(1)
				//	{
				//	}
				//
				//	void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				//	{
				//		ndBrainLossHuber::GetLoss(output, loss);
				//	}
				//};

				ndPolicyLoss loss;
				ndBrainFixSizeVector<1> stateValue;
				ndBrainFixSizeVector<statesDim> zeroObservations;

				zeroObservations.Set(ndBrainFloat(0.0f));
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_baseLineValueTrainers[i];
					const ndBrainMemVector observation(&m_trajectoryAccumulator[index].m_observation[0], statesDim);
					stateValue[0] = m_trajectoryAccumulator[index].m_reward;
					loss.SetTruth(stateValue);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			ndBrainThreadPool::ParallelExecute(BackPropagateBash);
			m_baseLineValueOptimizer->Update(this, m_baseLineValueTrainers, m_criticLearnRate);
		}
	}

	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	for (ndInt32 i = m_trajectoryAccumulator.GetCount(); i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator[i].m_reward;
	}
	m_averageScore.Update(averageSum / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(m_trajectoryAccumulator.GetCount()) / ndBrainFloat(m_bashTrajectoryIndex));

	ndBrainFixSizeVector<D_MAX_THREADS_COUNT> rewardVariance;

	rewardVariance.Set(ndBrainFloat(0.0f));
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
	auto CalculateAdvantage = ndMakeObject::ndFunction([this, &iterator, &rewardVariance](ndInt32 threadIndex, ndInt32)
	{
		ndBrainFixSizeVector<1> actions;
		ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);

		ndInt32 const count = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation(&m_trajectoryAccumulator[i].m_observation[0], statesDim);
			m_baseLineValue.MakePrediction(observation, actions, workingBuffer);
			ndBrainFloat advantage = m_trajectoryAccumulator[i].m_reward - actions[0];
			m_trajectoryAccumulator[i].m_reward = advantage;
			rewardVariance[threadIndex] += advantage * advantage;
		}
	});

	iterator = 0;
	ndBrainThreadPool::ParallelExecute(CalculateAdvantage);

	ndBrainFloat rewardVarianceSum = ndBrainFloat(0.0f);
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		rewardVarianceSum += rewardVariance[i];
	}

	rewardVarianceSum /= ndBrainFloat(m_trajectoryAccumulator.GetCount());
	ndBrainFloat invVariance = ndBrainFloat(1.0f) / ndBrainFloat(ndSqrt(rewardVarianceSum + ndBrainFloat(1.0e-4f)));
	ndInt32 newCount = m_trajectoryAccumulator.GetCount();
	for (ndInt32 i = m_trajectoryAccumulator.GetCount() - 1; i >= 0; --i)
	{
		const ndBrainFloat normalizedAdvantage = m_trajectoryAccumulator[i].m_reward * invVariance;
		m_trajectoryAccumulator[i].m_reward = normalizedAdvantage;

		// actions within 0.1 standard deviation, are not changed
		if (ndAbs(normalizedAdvantage) < ndBrainFloat(0.01f))
		{
			m_trajectoryAccumulator[i] = m_trajectoryAccumulator[newCount - 1];
			newCount--;
		}
	}
	m_trajectoryAccumulator.SetCount(newCount);
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::OptimizePolicy()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_trainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = m_trajectoryAccumulator.GetCount();
	for (ndInt32 base = 0; base < steps; base += m_bashBufferSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class Loss : public ndBrainLossLeastSquaredError
			{
				public:
				Loss(ndBrainTrainer& trainer, ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>* const agent, ndInt32 index)
					:ndBrainLossLeastSquaredError(trainer.GetBrain()->GetOutputSize())
					,m_trainer(trainer)
					,m_agent(agent)
					,m_index(index)
				{
				}
	
				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					const typename ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>::ndTrajectoryStep& trajectoryStep = m_agent->m_trajectoryAccumulator[m_index];
					ndInt32 actionIndex = ndInt32(trajectoryStep.m_action);
	
					loss.Set(ndBrainFloat(0.0f));
					ndBrainFloat negLogProb = ndBrainFloat(-ndLog(output[actionIndex]));
	
					loss[actionIndex] = negLogProb * trajectoryStep.m_reward;
				}
	
				ndBrainTrainer& m_trainer;
				ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>* m_agent;
				ndInt32 m_index;
			};
	
			ndBrainFixSizeVector<statesDim> observations;
			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_auxiliaryTrainers[i];
				Loss loss(trainer, this, base + i);
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					const ndBrainMemVector observation(&m_trajectoryAccumulator[base + i].m_observation[0], statesDim);
					trainer.BackPropagate(observation, loss);
				}
				else
				{
					trainer.ClearGradients();
				}
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_trainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_auxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}
	
	m_optimizer->AccumulateGradients(this, m_trainers);
	m_weightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_optimizer->Update(this, m_weightedTrainer, -m_policyLearnRate);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::Optimize()
{
	OptimizeCritic();
	OptimizePolicy();
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscretePolicyGradient_TrainerMaster<statesDim, actionDim>::OptimizeStep()
{
	for (typename ndList<ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentDiscretePolicyGradient_Trainer<statesDim, actionDim>* const agent = node->GetInfo();

		bool isTeminal = agent->IsTerminal() || (agent->m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
		if (isTeminal)
		{
			SaveTrajectory(agent);
			m_bashTrajectoryIndex++;
			if (m_trajectoryAccumulator.GetCount() >= m_bashTrajectorySteps)
			{
				Optimize();
				for (node = m_agents.GetFirst(); node; node = node->GetNext())
				{
					node->GetInfo()->m_trajectory.SetCount(0);
				}
				node = m_agents.GetLast();

				m_eposideCount++;
				m_framesAlive = 0;
				m_bashTrajectoryIndex = 0;
				m_trajectoryAccumulator.SetCount(0);
			}
			agent->ResetModel();
		}

		m_frameCount++;
		m_framesAlive++;
	}
}

#endif 