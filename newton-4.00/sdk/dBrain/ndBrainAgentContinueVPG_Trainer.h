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
#include "ndBrainLayerLinear.h"
#include "ndBrainLayerTanhActivation.h"
#include "ndBrainLossLeastSquaredError.h"

// this is an implementation of the vanilla policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/vpg.html


#if 0
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
			m_bashTrajectoryCount = 100;
			m_numberOfHiddenLayers = 4;
			m_maxTrajectorySteps = 1024 * 2;
			m_extraTrajectorySteps = 1024 * 2;
			m_hiddenLayersNumberOfNeurons = 64;
			
			ndBrainFloat sigma2 = ndBrainFloat(0.05f);
			//ndBrainFloat sigma2 = ndBrainFloat(0.1f);
			//ndBrainFloat sigma2 = ndBrainFloat (0.15f);
			//ndBrainFloat sigma2 = ndBrainFloat(0.2f);
			
			m_sigma = ndSqrt(sigma2);

			m_learnRate = ndBrainFloat(0.0005f);
			m_regularizer = ndBrainFloat(1.0e-6f);
			m_discountFactor = ndBrainFloat(0.99f);
			m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), ndMin(m_bashBufferSize, 16));
		}

		ndBrainFloat m_sigma;
		ndBrainFloat m_learnRate;
		ndBrainFloat m_regularizer;
		ndBrainFloat m_discountFactor;

		ndInt32 m_threadsCount;
		ndInt32 m_bashBufferSize;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_bashTrajectoryCount;
		ndInt32 m_extraTrajectorySteps;
		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_hiddenLayersNumberOfNeurons;
	};

	class ndTrajectoryStep
	{
		public:
		ndTrajectoryStep()
			:m_reward(ndBrainFloat(0.0f))
			,m_action()
			,m_observation()
		{
		}

		ndTrajectoryStep(const ndTrajectoryStep& src)
			:m_reward(src.m_reward)
		{
			ndMemCpy(m_action, src.m_action, actionDim);
			ndMemCpy(m_observation, src.m_observation, statesDim);
		}

		ndTrajectoryStep& operator=(const ndTrajectoryStep& src)
		{
			new (this) ndTrajectoryStep(src);
			return*this;
		}

		ndBrainFloat m_reward;
		ndBrainFloat m_action[actionDim];
		ndBrainFloat m_observation[statesDim];
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

	void Save(ndBrainSave* const loadSave);

	void InitWeights();
	void InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance);

	bool IsSampling() const;
	bool IsTerminal() const;
	ndBrainFloat CalculateReward();

	private:
	void Optimize();
	void BackPropagate();
	void SaveTrajectory();
	void SelectAction(ndBrainVector& probabilities) const;

	protected:
	ndBrain m_actor;
	ndBrain m_baseLineValue;
	ndBrainOptimizerAdam* m_optimizer;
	ndArray<ndBrainTrainer*> m_trainers;
	ndArray<ndBrainTrainer*> m_weightedTrainer;
	ndArray<ndBrainTrainer*> m_auxiliaryTrainers;
	ndBrainOptimizerAdam* m_baseLineValueOptimizer;
	ndArray<ndBrainTrainer*> m_baseLineValueTrainers;

	ndArray<ndTrajectoryStep> m_trajectory;
	ndArray<ndTrajectoryStep> m_trajectoryAccumulator;

	ndBrainFloat m_sigma;
	ndBrainFloat m_gamma;
	ndBrainFloat m_learnRate;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_maxTrajectorySteps;
	ndInt32 m_extraTrajectorySteps;
	ndInt32 m_bashTrajectoryIndex;
	ndInt32 m_bashTrajectoryCount;
	ndInt32 m_baseValueWorkingBufferSize;
	ndBrainVector m_workingBuffer;
	ndMovingAverage<32> m_averageScore;
	ndMovingAverage<32> m_averageFramesPerEpisodes;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::ndBrainAgentContinueVPG_Trainer(const HyperParameters& hyperParameters)
	:ndBrainAgent()
	,ndBrainThreadPool()
	,m_actor()
	,m_baseLineValue()
	,m_optimizer(nullptr)
	,m_trainers()
	,m_weightedTrainer()
	,m_auxiliaryTrainers()
	,m_baseLineValueOptimizer(nullptr)
	,m_baseLineValueTrainers()
	,m_trajectory()
	,m_trajectoryAccumulator()
	,m_sigma(hyperParameters.m_sigma)
	,m_gamma(hyperParameters.m_discountFactor)
	,m_learnRate(hyperParameters.m_learnRate)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_bashTrajectoryIndex(0)
	,m_bashTrajectoryCount(hyperParameters.m_bashTrajectoryCount)
	,m_baseValueWorkingBufferSize(0)
	,m_workingBuffer()
	,m_averageScore()
	,m_averageFramesPerEpisodes()

	,m_rd()
	,m_gen(m_rd())
	,m_d(ndFloat32 (0.0f), ndFloat32(1.0f))
{
	// build neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, hyperParameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerTanhActivation(hyperParameters.m_hiddenLayersNumberOfNeurons));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, actionDim));
	layers.PushBack(new ndBrainLayerTanhActivation(actionDim));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_actor.AddLayer(layers[i]);
	}
	m_actor.InitWeightsXavierMethod();
	ndAssert(!strcmp((m_actor[m_actor.GetCount() - 1])->GetLabelId(), "ndBrainLayerTanhActivation"));

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

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, hyperParameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerTanhActivation(hyperParameters.m_hiddenLayersNumberOfNeurons));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, 1));
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

	m_baseValueWorkingBufferSize = m_baseLineValue.CalculateWorkingtBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);

	m_trajectory.SetCount(m_maxTrajectorySteps + m_extraTrajectorySteps);
	//m_trajectoryAccumulator.SetCount(m_bashTrajectoryCount * m_maxTrajectorySteps + 1024);
	m_trajectoryAccumulator.SetCount(m_maxTrajectorySteps + m_extraTrajectorySteps);
	m_trajectory.SetCount(0);
	m_trajectoryAccumulator.SetCount(0);
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

	for (ndInt32 i = 0; i < m_baseLineValueTrainers.GetCount(); ++i)
	{
		delete m_baseLineValueTrainers[i];
	}
	delete m_baseLineValueOptimizer;
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
	m_baseLineValue.InitWeightsXavierMethod();
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

//#pragma optimize( "", off )
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

	const ndInt32 steps = m_trajectoryAccumulator.GetCount();
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
					const ndTrajectoryStep& trajectoryStep = m_agent->m_trajectoryAccumulator[m_index];
					ndBrainFloat logProbAdvantage = trajectoryStep.m_reward * m_invSigmaSquare;
					for (ndInt32 i = actionDim - 1; i >= 0; --i)
					{
						loss[i] = logProbAdvantage * (trajectoryStep.m_action[i] - output[i]);
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
	m_weightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_optimizer->Update(this, m_weightedTrainer, -m_learnRate);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave)
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
ndBrainFloat ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::CalculateReward()
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Optimize()
{
	ndBrainFixSizeVector<128> average;
	ndBrainFixSizeVector<128> variance2;
	
	variance2.SetCount(GetThreadCount());
	auto CalculateAdavantage = ndMakeObject::ndFunction([this, &average, &variance2](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainFixSizeVector<1> stateValue;
		ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);

		average[threadIndex] = ndBrainFloat(0.0f);
		variance2[threadIndex] = ndBrainFloat(0.0f);
		const ndStartEnd startEnd(m_trajectoryAccumulator.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBrainMemVector observation(&m_trajectoryAccumulator[i].m_observation[0], statesDim);
			m_baseLineValue.MakePrediction(observation, stateValue, workingBuffer);
			ndBrainFloat advantage = m_trajectoryAccumulator[i].m_reward - stateValue[0];

			average[threadIndex] += m_trajectoryAccumulator[i].m_reward;
			m_trajectoryAccumulator[i].m_reward = advantage;
			variance2[threadIndex] += advantage * advantage;
		}
	});
	ndBrainThreadPool::ParallelExecute(CalculateAdavantage);
	
	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	ndBrainFloat varianceSum2 = ndBrainFloat(0.0f);
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		averageSum += average[i];
		varianceSum2 += variance2[i];
	}
	m_averageScore.Update(averageSum / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(m_trajectoryAccumulator.GetCount()) / ndBrainFloat(m_bashTrajectoryCount));

	varianceSum2 /= ndBrainFloat(m_trajectoryAccumulator.GetCount());
	ndBrainFloat invVariance = ndBrainFloat(1.0f) / ndBrainFloat(ndSqrt(varianceSum2));
	for (ndInt32 i = m_trajectoryAccumulator.GetCount() - 1; i >= 0; --i)
	{
		m_trajectoryAccumulator[i].m_reward *= invVariance;
	}

	BackPropagate();
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::SaveTrajectory()
{
	// remove last step because if it was a dead state, it will provide misleading feedback.
	m_trajectory.SetCount(m_trajectory.GetCount() - 1);

	// get the max trajectory steps
	const ndInt32 maxSteps = ndMin(m_trajectory.GetCount(), m_maxTrajectorySteps);
	ndAssert(maxSteps > 0);

	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
	for (ndInt32 base = 0; base < maxSteps; base += m_bashBufferSize)
	{
		auto BackPropagateBash = ndMakeObject::ndFunction([this, base, maxSteps](ndInt32 threadIndex, ndInt32 threadCount)
		{
			class ndPolicyLoss : public ndBrainLossHuber
			{
				public:
				ndPolicyLoss()
					:ndBrainLossHuber(1)
					, m_lossValid(true)
				{
				}

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					loss.Set(m_truth);
					if (!m_lossValid)
					{
						SetTruth(output);
					}
					ndBrainLossHuber::GetLoss(output, loss);
				}

				bool m_lossValid;
			};

			ndPolicyLoss loss;
			ndBrainFixSizeVector<1> stateValue;
			ndBrainFixSizeVector<actionDim> actions;
			ndBrainFixSizeVector<statesDim> zeroObservations;
			ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);
			
			const ndBrainFloat gamma1 = m_gamma;
			const ndBrainFloat gamma2 = gamma1 * m_gamma;
			const ndBrainFloat gamma3 = gamma2 * m_gamma;
			const ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / (ndBrainFloat(2.0f) * m_sigma * m_sigma);
			//const ndBrainFloat uniVariateGaussian = ndBrainFloat(1.0f) / (m_sigma * ndBrainFloat(ndSqrt(ndBrainFloat(2.0f) * ndPi)));
			//const ndBrainFloat gaussian = ndPow (uniVariateGaussian, ndBrainFloat(actionDim));

			zeroObservations.Set(ndBrainFloat(0.0f));
			const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBrainTrainer& trainer = *m_baseLineValueTrainers[i];
				ndInt32 index = base + i;
				if ((index + 3) < maxSteps)
				{
					const ndBrainMemVector observation(&m_trajectory[ndInt32(index)].m_observation[0], statesDim);
					m_actor.MakePrediction(observation, actions, workingBuffer);

					ndBrainFloat exponent = ndBrainFloat(0.0f);
					for (ndInt32 j = actions.GetCount() - 1; j >= 0; --j)
					{
						ndBrainFloat error = (actions[j] - m_trajectory[ndInt32(index)].m_action[j]);
						exponent += error * error * invSigma2;
					}
					//ndBrainFloat stateProb = ndBrainFloat (gaussian * ndExp(-exponent));
					ndBrainFloat stateProb = ndBrainFloat (ndExp(-exponent));

					const ndBrainMemVector baseObservation(&m_trajectory[ndInt32(index + 3)].m_observation[0], statesDim);
					m_baseLineValue.MakePrediction(baseObservation, stateValue, workingBuffer);
					ndBrainFloat q0 = m_trajectory[ndInt32(index + 0)].m_reward;
					ndBrainFloat q1 = m_trajectory[ndInt32(index + 1)].m_reward;
					ndBrainFloat q2 = m_trajectory[ndInt32(index + 2)].m_reward;
					ndBrainFloat q3 = stateValue[0];
					stateValue[0] = stateProb * (q0 + q1 * gamma1 + q2 * gamma2 + q3 * gamma3);
					loss.SetTruth(stateValue);
					trainer.BackPropagate(observation, loss);
				}
				else
				{
					loss.m_lossValid = false;
					trainer.BackPropagate(zeroObservations, loss);
				}
			}
		});

		ndBrainThreadPool::ParallelExecute(BackPropagateBash);
		m_baseLineValueOptimizer->Update(this, m_baseLineValueTrainers, m_learnRate);
	}

	// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
	for (ndInt32 i = m_trajectory.GetCount() - 2; i >= 0; --i)
	{
		m_trajectory[i].m_reward += m_gamma * m_trajectory[i + 1].m_reward;
	}

	for (ndInt32 i = 0; i < maxSteps; ++i)
	{
		m_trajectoryAccumulator.PushBack(m_trajectory[i]);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::SelectAction(ndBrainVector& actions) const
{
	for (ndInt32 i = actionDim - 1; i >= 0; --i)
	{
		ndBrainFloat sample = ndBrainFloat (ndGaussianRandom(actions[i], m_sigma));
		ndBrainFloat squashedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = squashedAction;
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Step()
{
	ndTrajectoryStep trajectoryStep;

	ndBrainMemVector actions(&trajectoryStep.m_action[0], actionDim);
	ndBrainMemVector observation(&trajectoryStep.m_observation[0], statesDim);

	GetObservation(&observation[0]);
	m_actor.MakePrediction(observation, actions, m_workingBuffer);

	SelectAction(actions);
	ApplyActions(&trajectoryStep.m_action[0]);
	trajectoryStep.m_reward = CalculateReward();
	
	ndAssert(m_trajectory.GetCount() < m_trajectory.GetCapacity());
	m_trajectory.PushBack(trajectoryStep);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ResetModel();
		m_trajectory.SetCount(0);
	}

	bool isTeminal = IsTerminal() || (m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
	if (isTeminal)
	{
		SaveTrajectory();
		m_bashTrajectoryIndex++;
		if (m_bashTrajectoryIndex >= m_bashTrajectoryCount)
		{
			Optimize();
			m_trajectoryAccumulator.SetCount(0);
			m_eposideCount++;
			m_framesAlive = 0;
			m_bashTrajectoryIndex = 0;
		}
		m_trajectory.SetCount(0);
		ResetModel();
	}
	
	m_frameCount++;
	m_framesAlive++;
}

#else

template<ndInt32 statesDim, ndInt32 actionDim> class ndBrainAgentContinueVPG_TrainerMaster;


template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentContinueVPG_Trainer : public ndBrainAgent
{
	public:

	class ndTrajectoryStep
	{
		public:
		ndTrajectoryStep()
			:m_reward(ndBrainFloat(0.0f))
			,m_action()
			,m_observation()
		{
		}

		ndTrajectoryStep(const ndTrajectoryStep& src)
			:m_reward(src.m_reward)
		{
			ndMemCpy(m_action, src.m_action, actionDim);
			ndMemCpy(m_observation, src.m_observation, statesDim);
		}

		ndTrajectoryStep& operator=(const ndTrajectoryStep& src)
		{
			new (this) ndTrajectoryStep(src);
			return*this;
		}

		ndBrainFloat m_reward;
		ndBrainFloat m_action[actionDim];
		ndBrainFloat m_observation[statesDim];
	};

	ndBrainAgentContinueVPG_Trainer(ndSharedPtr<ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>>& master);
	~ndBrainAgentContinueVPG_Trainer();

	ndBrain* GetActor();
	void SelectAction(ndBrainVector& actions) const;

	void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	void Save(ndBrainSave* const) { ndAssert(0); }
	bool IsTrainer() const { ndAssert(0); return true; }
	ndInt32 GetEpisodeFrames() const { ndAssert(0); return 0; }
	void InitWeights(ndBrainFloat, ndBrainFloat) { ndAssert(0); }

	virtual bool IsTerminal() const;
	virtual void Step();

	ndBrainVector m_workingBuffer;
	ndArray<ndTrajectoryStep> m_trajectory;
	ndSharedPtr<ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>> m_master;

	friend class ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>;
};

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentContinueVPG_TrainerMaster : public ndBrainThreadPool
{
	public:

	class HyperParameters
	{
		public:
		HyperParameters()
		{
			m_bashBufferSize = 32;
			m_bashTrajectoryCount = 100;
			m_numberOfHiddenLayers = 4;
			m_maxTrajectorySteps = 1024 * 2;
			m_extraTrajectorySteps = 1024 * 2;
			m_hiddenLayersNumberOfNeurons = 64;

			ndBrainFloat sigma2 = ndBrainFloat(0.05f);
			//ndBrainFloat sigma2 = ndBrainFloat(0.1f);
			//ndBrainFloat sigma2 = ndBrainFloat (0.15f);
			//ndBrainFloat sigma2 = ndBrainFloat(0.2f);

			m_sigma = ndSqrt(sigma2);

			m_learnRate = ndBrainFloat(0.0005f);
			m_regularizer = ndBrainFloat(1.0e-6f);
			m_discountFactor = ndBrainFloat(0.99f);
			m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), ndMin(m_bashBufferSize, 16));
		}

		ndBrainFloat m_sigma;
		ndBrainFloat m_learnRate;
		ndBrainFloat m_regularizer;
		ndBrainFloat m_discountFactor;

		ndInt32 m_threadsCount;
		ndInt32 m_bashBufferSize;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_bashTrajectoryCount;
		ndInt32 m_extraTrajectorySteps;
		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_hiddenLayersNumberOfNeurons;
	};

	ndBrainAgentContinueVPG_TrainerMaster(const HyperParameters& hyperParameters);
	virtual ~ndBrainAgentContinueVPG_TrainerMaster();

	ndBrain* GetActor();
	//ndInt32 GetFramesCount() const;
	//ndInt32 GetEposideCount() const;
	//ndInt32 GetEpisodeFrames() const;
	//
	//bool IsTrainer() const;
	//
	//protected:
	//void Step();
	//void OptimizeStep();
	//
	//void Save(ndBrainSave* const loadSave);
	
	void InitWeights();
	void InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance);
	
	//bool IsSampling() const;
	//bool IsTerminal() const;
	//ndBrainFloat CalculateReward();
	//
	//private:
	//void Optimize();
	//void BackPropagate();
	//void SelectAction(ndBrainVector& probabilities) const;
	void OptimizeStep();
	
	protected:
	void SaveTrajectory(ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>* const agent);

	ndBrain m_actor;
	ndBrain m_baseLineValue;
	ndBrainOptimizerAdam* m_optimizer;
	ndArray<ndBrainTrainer*> m_trainers;
	ndArray<ndBrainTrainer*> m_weightedTrainer;
	ndArray<ndBrainTrainer*> m_auxiliaryTrainers;
	ndBrainOptimizerAdam* m_baseLineValueOptimizer;
	ndArray<ndBrainTrainer*> m_baseLineValueTrainers;
	
	//ndArray<ndTrajectoryStep> m_trajectory;
	ndArray<typename ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::ndTrajectoryStep> m_trajectoryAccumulator;
	
	ndBrainFloat m_sigma;
	ndBrainFloat m_gamma;
	ndBrainFloat m_learnRate;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	//ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_maxTrajectorySteps;
	ndInt32 m_extraTrajectorySteps;
	ndInt32 m_bashTrajectoryIndex;
	ndInt32 m_bashTrajectoryCount;
	ndInt32 m_baseValueWorkingBufferSize;
	ndBrainVector m_workingBuffer;
	//ndMovingAverage<32> m_averageScore;
	//ndMovingAverage<32> m_averageFramesPerEpisodes;
	//
	//mutable std::random_device m_rd;
	//mutable std::mt19937 m_gen;
	//mutable std::normal_distribution<ndFloat32> m_d;
	ndList<ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>*> m_agents;
	friend class ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>;
};

/*

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::~ndBrainAgentContinueVPG_Trainer()
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
bool ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::IsTrainer() const
{
	return true;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::InitWeights()
{
	m_actor.InitWeightsXavierMethod();
	m_baseLineValue.InitWeightsXavierMethod();
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

//#pragma optimize( "", off )
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

	const ndInt32 steps = m_trajectoryAccumulator.GetCount();
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
						, m_trainer(trainer)
						, m_agent(agent)
						, m_invSigmaSquare(invSigmaSquare)
						, m_index(index)
					{
					}

					void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
					{
						const ndTrajectoryStep& trajectoryStep = m_agent->m_trajectoryAccumulator[m_index];
						ndBrainFloat logProbAdvantage = trajectoryStep.m_reward * m_invSigmaSquare;
						for (ndInt32 i = actionDim - 1; i >= 0; --i)
						{
							loss[i] = logProbAdvantage * (trajectoryStep.m_action[i] - output[i]);
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
	m_weightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_optimizer->Update(this, m_weightedTrainer, -m_learnRate);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave)
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
ndBrainFloat ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::CalculateReward()
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Optimize()
{
	ndBrainFixSizeVector<128> average;
	ndBrainFixSizeVector<128> variance2;

	variance2.SetCount(GetThreadCount());
	auto CalculateAdavantage = ndMakeObject::ndFunction([this, &average, &variance2](ndInt32 threadIndex, ndInt32 threadCount)
		{
			ndBrainFixSizeVector<1> stateValue;
			ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);

			average[threadIndex] = ndBrainFloat(0.0f);
			variance2[threadIndex] = ndBrainFloat(0.0f);
			const ndStartEnd startEnd(m_trajectoryAccumulator.GetCount(), threadIndex, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBrainMemVector observation(&m_trajectoryAccumulator[i].m_observation[0], statesDim);
				m_baseLineValue.MakePrediction(observation, stateValue, workingBuffer);
				ndBrainFloat advantage = m_trajectoryAccumulator[i].m_reward - stateValue[0];

				average[threadIndex] += m_trajectoryAccumulator[i].m_reward;
				m_trajectoryAccumulator[i].m_reward = advantage;
				variance2[threadIndex] += advantage * advantage;
			}
		});
	ndBrainThreadPool::ParallelExecute(CalculateAdavantage);

	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	ndBrainFloat varianceSum2 = ndBrainFloat(0.0f);
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		averageSum += average[i];
		varianceSum2 += variance2[i];
	}
	m_averageScore.Update(averageSum / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(m_trajectoryAccumulator.GetCount()) / ndBrainFloat(m_bashTrajectoryCount));

	varianceSum2 /= ndBrainFloat(m_trajectoryAccumulator.GetCount());
	ndBrainFloat invVariance = ndBrainFloat(1.0f) / ndBrainFloat(ndSqrt(varianceSum2));
	for (ndInt32 i = m_trajectoryAccumulator.GetCount() - 1; i >= 0; --i)
	{
		m_trajectoryAccumulator[i].m_reward *= invVariance;
	}

	BackPropagate();
}


template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::SelectAction(ndBrainVector& actions) const
{
	//for (ndInt32 i = 0; i < 2000; ++i)
	//{
	//	ndBrainFloat xxx0 = ndGaussianRandom(5.0f, 2.0f);
	//	ndBrainFloat xxx1 = 5.0f + ndBrainFloat(m_d(m_gen) * 2.0f);
	//	ndTrace (("%f, %f\n", xxx0, xxx1))
	//}

	for (ndInt32 i = actionDim - 1; i >= 0; --i)
	{
		ndBrainFloat sample = ndBrainFloat(ndGaussianRandom(actions[i], m_sigma));
		ndBrainFloat squashedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = squashedAction;
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Step()
{
	ndTrajectoryStep trajectoryStep;

	ndBrainMemVector actions(&trajectoryStep.m_action[0], actionDim);
	ndBrainMemVector observation(&trajectoryStep.m_observation[0], statesDim);

	GetObservation(&observation[0]);
	m_actor.MakePrediction(observation, actions, m_workingBuffer);

	SelectAction(actions);
	ApplyActions(&trajectoryStep.m_action[0]);
	trajectoryStep.m_reward = CalculateReward();

	ndAssert(m_trajectory.GetCount() < m_trajectory.GetCapacity());
	m_trajectory.PushBack(trajectoryStep);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ResetModel();
		m_trajectory.SetCount(0);
	}

	bool isTeminal = IsTerminal() || (m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
	if (isTeminal)
	{
		SaveTrajectory();
		m_bashTrajectoryIndex++;
		if (m_bashTrajectoryIndex >= m_bashTrajectoryCount)
		{
			Optimize();
			m_trajectoryAccumulator.SetCount(0);
			m_eposideCount++;
			m_framesAlive = 0;
			m_bashTrajectoryIndex = 0;
		}
		m_trajectory.SetCount(0);
		ResetModel();
	}

	m_frameCount++;
	m_framesAlive++;
}
*/

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::ndBrainAgentContinueVPG_Trainer(ndSharedPtr<ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>>& master)
	:ndBrainAgent()
	,m_workingBuffer()
	,m_trajectory()
	,m_master(master)
{
	m_master->m_agents.Append(this);
	m_trajectory.SetCount(m_master->m_maxTrajectorySteps + m_master->m_extraTrajectorySteps);
	m_trajectory.SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::~ndBrainAgentContinueVPG_Trainer()
{
	for (ndList<ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>*>::ndNode* node = m_master->m_agents.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo() == this)
		{
			m_master->m_agents.Remove(node);
			break;
		}
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrain* ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::GetActor()
{ 
	return m_master->GetActor(); 
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::IsTerminal() const
{
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::SelectAction(ndBrainVector& actions) const
{
	//for (ndInt32 i = 0; i < 2000; ++i)
	//{
	//	ndBrainFloat xxx0 = ndGaussianRandom(5.0f, 2.0f);
	//	ndBrainFloat xxx1 = 5.0f + ndBrainFloat(m_d(m_gen) * 2.0f);
	//	ndTrace (("%f, %f\n", xxx0, xxx1))
	//}

	for (ndInt32 i = actionDim - 1; i >= 0; --i)
	{
		ndBrainFloat sample = ndBrainFloat(ndGaussianRandom(actions[i], m_master->m_sigma));
		ndBrainFloat squashedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = squashedAction;
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>::Step()
{
	ndTrajectoryStep trajectoryStep;

	ndBrainMemVector actions(&trajectoryStep.m_action[0], actionDim);
	ndBrainMemVector observation(&trajectoryStep.m_observation[0], statesDim);

	GetObservation(&observation[0]);
	m_master->m_actor.MakePrediction(observation, actions, m_workingBuffer);

	SelectAction(actions);
	ApplyActions(&trajectoryStep.m_action[0]);
	trajectoryStep.m_reward = CalculateReward();

	ndAssert(m_trajectory.GetCount() < m_trajectory.GetCapacity());
	m_trajectory.PushBack(trajectoryStep);
}

// ***************************************************************************************
//
// ***************************************************************************************
template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>::ndBrainAgentContinueVPG_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainThreadPool()
	,m_actor()
	,m_baseLineValue()
	,m_optimizer(nullptr)
	,m_trainers()
	,m_weightedTrainer()
	,m_auxiliaryTrainers()
	,m_baseLineValueOptimizer(nullptr)
	,m_baseLineValueTrainers()
	//, m_trajectory()
	,m_trajectoryAccumulator()
	,m_sigma(hyperParameters.m_sigma)
	,m_gamma(hyperParameters.m_discountFactor)
	,m_learnRate(hyperParameters.m_learnRate)
	,m_frameCount(0)
	,m_framesAlive(0)
	//, m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_bashTrajectoryIndex(0)
	,m_bashTrajectoryCount(hyperParameters.m_bashTrajectoryCount)
	,m_baseValueWorkingBufferSize(0)
	,m_workingBuffer()
	,m_agents()
	//, m_averageScore()
	//, m_averageFramesPerEpisodes()
	//
	//, m_rd()
	//, m_gen(m_rd())
	//, m_d(ndFloat32(0.0f), ndFloat32(1.0f))
{
	// build neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, hyperParameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerTanhActivation(hyperParameters.m_hiddenLayersNumberOfNeurons));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, actionDim));
	layers.PushBack(new ndBrainLayerTanhActivation(actionDim));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_actor.AddLayer(layers[i]);
	}
	m_actor.InitWeightsXavierMethod();
	ndAssert(!strcmp((m_actor[m_actor.GetCount() - 1])->GetLabelId(), "ndBrainLayerTanhActivation"));
	
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
	
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(statesDim, hyperParameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerTanhActivation(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, hyperParameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerTanhActivation(hyperParameters.m_hiddenLayersNumberOfNeurons));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, 1));
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
	
	m_baseValueWorkingBufferSize = m_baseLineValue.CalculateWorkingtBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
	
	//m_trajectory.SetCount(m_maxTrajectorySteps + m_extraTrajectorySteps);
	////m_trajectoryAccumulator.SetCount(m_bashTrajectoryCount * m_maxTrajectorySteps + 1024);
	//m_trajectoryAccumulator.SetCount(m_maxTrajectorySteps + m_extraTrajectorySteps);
	//m_trajectory.SetCount(0);
	//m_trajectoryAccumulator.SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>::~ndBrainAgentContinueVPG_TrainerMaster()
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
ndBrain* ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>::GetActor()
{ 
	return &m_actor; 
}

//#pragma optimize( "", off )
template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>::SaveTrajectory(ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>* const agent)
{
	// remove last step because if it was a dead state, it will provide misleading feedback.
	agent->m_trajectory.SetCount(agent->m_trajectory.GetCount() - 1);

	// get the max trajectory steps
	const ndInt32 maxSteps = ndMin(agent->m_trajectory.GetCount(), m_maxTrajectorySteps);
	ndAssert(maxSteps > 0);

	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
	for (ndInt32 base = 0; base < maxSteps; base += m_bashBufferSize)
	{
		auto BackPropagateBash = ndMakeObject::ndFunction([this, agent, base, maxSteps](ndInt32 threadIndex, ndInt32 threadCount)
		{
			class ndPolicyLoss : public ndBrainLossHuber
			{
				public:
				ndPolicyLoss()
					:ndBrainLossHuber(1)
					,m_lossValid(true)
				{
				}

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					loss.Set(m_truth);
					if (!m_lossValid)
					{
						SetTruth(output);
					}
					ndBrainLossHuber::GetLoss(output, loss);
				}

				bool m_lossValid;
			};

			ndPolicyLoss loss;
			ndBrainFixSizeVector<1> stateValue;
			ndBrainFixSizeVector<actionDim> actions;
			ndBrainFixSizeVector<statesDim> zeroObservations;
			ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);

			const ndBrainFloat gamma1 = m_gamma;
			const ndBrainFloat gamma2 = gamma1 * m_gamma;
			const ndBrainFloat gamma3 = gamma2 * m_gamma;
			const ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / (ndBrainFloat(2.0f) * m_sigma * m_sigma);
			//const ndBrainFloat uniVariateGaussian = ndBrainFloat(1.0f) / (m_sigma * ndBrainFloat(ndSqrt(ndBrainFloat(2.0f) * ndPi)));
			//const ndBrainFloat gaussian = ndPow (uniVariateGaussian, ndBrainFloat(actionDim));

			zeroObservations.Set(ndBrainFloat(0.0f));
			const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBrainTrainer& trainer = *m_baseLineValueTrainers[i];
				ndInt32 index = base + i;
				if ((index + 3) < maxSteps)
				{
					const ndBrainMemVector observation(&agent->m_trajectory[ndInt32(index)].m_observation[0], statesDim);
					m_actor.MakePrediction(observation, actions, workingBuffer);

					ndBrainFloat exponent = ndBrainFloat(0.0f);
					for (ndInt32 j = actions.GetCount() - 1; j >= 0; --j)
					{
						ndBrainFloat error = (actions[j] - agent->m_trajectory[ndInt32(index)].m_action[j]);
						exponent += error * error * invSigma2;
					}
					//ndBrainFloat stateProb = ndBrainFloat (gaussian * ndExp(-exponent));
					ndBrainFloat stateProb = ndBrainFloat(ndExp(-exponent));

					const ndBrainMemVector baseObservation(&agent->m_trajectory[ndInt32(index + 3)].m_observation[0], statesDim);
					m_baseLineValue.MakePrediction(baseObservation, stateValue, workingBuffer);
					ndBrainFloat q0 = agent->m_trajectory[ndInt32(index + 0)].m_reward;
					ndBrainFloat q1 = agent->m_trajectory[ndInt32(index + 1)].m_reward;
					ndBrainFloat q2 = agent->m_trajectory[ndInt32(index + 2)].m_reward;
					ndBrainFloat q3 = stateValue[0];
					stateValue[0] = stateProb * (q0 + q1 * gamma1 + q2 * gamma2 + q3 * gamma3);
					loss.SetTruth(stateValue);
					trainer.BackPropagate(observation, loss);
				}
				else
				{
					loss.m_lossValid = false;
					trainer.BackPropagate(zeroObservations, loss);
				}
			}
		});

		ndBrainThreadPool::ParallelExecute(BackPropagateBash);
		m_baseLineValueOptimizer->Update(this, m_baseLineValueTrainers, m_learnRate);
	}

	// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
	for (ndInt32 i = agent->m_trajectory.GetCount() - 2; i >= 0; --i)
	{
		agent->m_trajectory[i].m_reward += m_gamma * agent->m_trajectory[i + 1].m_reward;
	}

	for (ndInt32 i = 0; i < maxSteps; ++i)
	{
		m_trajectoryAccumulator.PushBack(agent->m_trajectory[i]);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentContinueVPG_TrainerMaster<statesDim, actionDim>::OptimizeStep()
{
	for (ndList<ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentContinueVPG_Trainer<statesDim, actionDim>* const agent = node->GetInfo();

		//bool isTeminal = IsTerminal() || (m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
		bool isTeminal = agent->IsTerminal() || (agent->m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
		if (isTeminal)
		{
			SaveTrajectory(agent);
			m_bashTrajectoryIndex++;
			if (m_bashTrajectoryIndex >= m_bashTrajectoryCount)
			{
				ndAssert(0);
			//	Optimize();
			//	m_trajectoryAccumulator.SetCount(0);
			//	m_eposideCount++;
				m_framesAlive = 0;
				m_bashTrajectoryIndex = 0;
			}
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
		}

		m_frameCount++;
		m_framesAlive++;
	}
}

#endif

#endif 