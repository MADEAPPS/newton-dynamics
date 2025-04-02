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

#include "ndBrainStdafx.h"
#include "ndBrainTrainer.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainOptimizerSgd.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainAgentContinueProximaPolicyGradient_Trainer.h"

#define ND_CONTINUE_PROXIMA_POLICY_ITERATIONS		5
#define ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE	ndBrainFloat(0.1f)
#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON		ndBrainFloat(0.2f)
#define ND_CONTINUE_PROXIMA_POLICY_MIN_SIGMA		ndBrainFloat(1.0e-1f)

#if 0
// ***************************************************************************************
//
// ***************************************************************************************
ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainThreadPool()
	,m_policy()
	,m_critic()
	,m_oldPolicy()
	,m_tempPolicy()
	,m_criticTrainers()
	,m_policyTrainers()
	,m_policyWeightedTrainer()
	,m_policyAuxiliaryTrainers()
	,m_criticOptimizer(nullptr)
	,m_policyOptimizer(nullptr)
	,m_randomPermutation()
	,m_trajectoryAccumulator(hyperParameters.m_numberOfActions, hyperParameters.m_numberOfObservations)
	,m_gamma(hyperParameters.m_discountFactor)
	,m_policyLearnRate(hyperParameters.m_policyLearnRate)
	,m_criticLearnRate(hyperParameters.m_criticLearnRate)
	,m_numberOfActions(hyperParameters.m_numberOfActions)
	,m_numberOfObservations(hyperParameters.m_numberOfObservations)
	,m_framesAlive(0)
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_bashTrajectoryIndex(0)
	,m_bashTrajectoryCount(hyperParameters.m_bashTrajectoryCount)
	,m_bashTrajectorySteps(hyperParameters.m_bashTrajectoryCount * m_maxTrajectorySteps)
	,m_baseValueWorkingBufferSize(0)
	,m_randomSeed(hyperParameters.m_randomSeed)
	,m_workingBuffer()
	,m_averageScore()
	,m_averageFramesPerEpisodes()
	,m_agents()
{
	ndAssert(m_numberOfActions);
	ndAssert(m_numberOfObservations);
	ndSetRandSeed(m_randomSeed);

	m_randomGenerator = new ndBrainAgentContinueProximaPolicyGradient_Trainer::ndRandomGenerator[size_t(hyperParameters.m_bashTrajectoryCount)];
	for (ndInt32 i = 0; i < hyperParameters.m_bashTrajectoryCount; ++i)
	{
		m_randomSeed++;
		m_randomGenerator[i].m_gen.seed(m_randomSeed);
	}
	m_randomSeed = 0;

	// build policy neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, hyperParameters.m_neuronPerLayers));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, hyperParameters.m_neuronPerLayers));
		layers.PushBack(new ndBrainLayerActivationTanh(hyperParameters.m_neuronPerLayers));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, m_numberOfActions * 2));
	layers.PushBack(new ndPolicyGradientActivation(m_numberOfActions));

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_policy.AddLayer(layers[i]);
	}

	m_policy.InitWeights();
	NormalizePolicy();
	ndAssert(!strcmp((m_policy[m_policy.GetCount() - 1])->GetLabelId(), "ndBrainLayerActivationTanh"));

	m_oldPolicy = m_policy;
	m_tempPolicy = m_policy;

	m_policyTrainers.SetCount(0);
	m_policyAuxiliaryTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_policy);
		m_policyTrainers.PushBack(trainer);
	
		ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(&m_policy);
		m_policyAuxiliaryTrainers.PushBack(auxiliaryTrainer);
	}
	
	m_policyWeightedTrainer.PushBack(m_policyTrainers[0]);
	m_policyOptimizer = new ndBrainOptimizerAdam();
	m_policyOptimizer->SetRegularizer(hyperParameters.m_regularizer);

	// build state value critic neural net
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, hyperParameters.m_neuronPerLayers * 2));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers * 2);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), hyperParameters.m_neuronPerLayers * 2));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_critic.AddLayer(layers[i]);
	}
	m_critic.InitWeights();
	NormalizeCritic();
	
	ndAssert(m_critic.GetOutputSize() == 1);
	ndAssert(m_critic.GetInputSize() == m_policy.GetInputSize());
	ndAssert(!strcmp((m_critic[m_critic.GetCount() - 1])->GetLabelId(), "ndBrainLayerLinear"));
	
	m_criticTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic);
		m_criticTrainers.PushBack(trainer);
	}
	
	m_criticOptimizer = new ndBrainOptimizerAdam();
	m_criticOptimizer->SetRegularizer(ndBrainFloat(1.0e-3f));
	
	m_baseValueWorkingBufferSize = m_critic.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
}




void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizePolicy()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_policyTrainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount());
	for (ndInt32 base = 0; base < steps; base += m_bashBufferSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class MaxLikelihoodLoss : public ndBrainLoss
			{
				public:
				MaxLikelihoodLoss(ndBrainTrainer& trainer, ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* const agent, ndInt32 index)
					:ndBrainLoss()
					,m_trainer(trainer)
					,m_agent(agent)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					// basically this fits a multivariate Gaussian process with zero cross covariance to the actions.
					const ndInt32 numberOfActions = m_agent->m_numberOfActions;
					const ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					const ndBrainMemVector sampledProbability (m_agent->m_trajectoryAccumulator.GetActions(m_index), numberOfActions * 2);

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = probabilityDistribution[i];
						const ndBrainFloat sigma1 = probabilityDistribution[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat sigma3 = sigma2 * sigma1;
						const ndBrainFloat num = (sampledProbability[i] - mean);

						// this was a huge bug, it is gradient ascend
						ndBrainFloat meanGradient = -num / sigma2;
						ndBrainFloat sigmaGradient = num * num / sigma3 - ndBrainFloat(1.0f) / sigma1;

						loss[i] = -meanGradient * advantage;
						loss[i + numberOfActions] = -sigmaGradient * advantage;
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				MaxLikelihoodLoss loss(trainer, this, base + i);
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(base + i), m_numberOfObservations);
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
				ndBrainTrainer* const trainer = m_policyTrainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}

	m_policyOptimizer->AccumulateGradients(this, m_policyTrainers);
	m_policyWeightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, -m_policyLearnRate);
}

//#pragma optimize( "", off )
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::UpdateBaseLineValue()
{
	m_trajectoryAccumulator.SetTerminalState(m_trajectoryAccumulator.GetCount() - 2, true);

	m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount() - 1);
	for (ndInt32 i = ndInt32(m_randomPermutation.GetCount()) - 1; i >= 0; --i)
	{
		m_randomPermutation[i] = i;
	}
	m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());

	if (m_randomPermutation.GetCount() > ND_CONTINUE_PROXIMA_POLICY_BUFFER_SIZE)
	{
		m_randomPermutation.SetCount(ND_CONTINUE_PROXIMA_POLICY_BUFFER_SIZE);
	}
	else
	{
		ndInt32 smallSize = ndInt32(m_randomPermutation.GetCount()) & -m_bashBufferSize;
		m_randomPermutation.SetCount(smallSize);
	}

	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 i = 0; i < ND_CONTINUE_PROXIMA_POLICY_STATE_VALUE_ITERATIONS; ++i)
	{ 
		for (ndInt32 base = 0; base < m_randomPermutation.GetCount(); base += m_bashBufferSize)
		{
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainLossLeastSquaredError loss(1);
				ndBrainFixSizeVector<1> stateValue;
				ndBrainFixSizeVector<1> stateQValue;
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_criticTrainers[i];

					stateValue[0] = m_trajectoryAccumulator.GetReward(index);
					if (!m_trajectoryAccumulator.GetTerminalState(index))
					{
						const ndBrainMemVector qObservation(m_trajectoryAccumulator.GetObservations(index + 1), m_numberOfObservations);
						m_critic.MakePrediction(qObservation, stateQValue);
						stateValue[0] += m_gamma * stateQValue[0];
					}

					loss.SetTruth(stateValue);

					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			ndBrainThreadPool::ParallelExecute(BackPropagateBash);
			m_criticOptimizer->Update(this, m_criticTrainers, m_criticLearnRate);
		}
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
	}
}

void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizeCritic()
{
	UpdateBaseLineValue();

	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
	for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator.GetAdvantage(i);
	}
	m_averageScore.Update(averageSum / ndBrainFloat(stepNumber));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(stepNumber) / ndBrainFloat(m_bashTrajectoryIndex));
	
	ndAtomic<ndInt32> iterator(0);
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
	auto CalculateAdvantage = ndMakeObject::ndFunction([this, &iterator](ndInt32 threadIndex, ndInt32)
	{
		ndBrainFixSizeVector<1> actions;
		ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);
	
		ndInt32 const count = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			m_critic.MakePrediction(observation, actions, workingBuffer);
			ndBrainFloat baseLine = actions[0];
			ndBrainFloat reward = m_trajectoryAccumulator.GetAdvantage(i);
			ndBrainFloat advantage = reward - baseLine;
			m_trajectoryAccumulator.SetAdvantage(i, advantage);
		}
	});
	ndBrainThreadPool::ParallelExecute(CalculateAdvantage);
}

void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::Optimize()
{
	OptimizeCritic();

	m_oldPolicy.CopyFrom(m_policy);
	OptimizePolicy();
	for (ndInt32 i = 0; (i < ND_CONTINUE_PROXIMA_POLICY_ITERATIONS) && (CalculateKLdivergence() < ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE); ++i)
	{
		OptimizePolicyPPOstep();
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizeStep()
{
	for (ndList<ndBrainAgentContinueProximaPolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentContinueProximaPolicyGradient_Trainer* const agent = node->GetInfo();

		bool isTeminal = agent->m_trajectory.GetTerminalState(agent->m_trajectory.GetCount() - 1);
		isTeminal = isTeminal || (agent->m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
		if (isTeminal)
		{
			agent->SaveTrajectory();
			agent->ResetModel();
			agent->m_randomGenerator = GetRandomGenerator();
		}
		m_frameCount++;
		m_framesAlive++;
	}

	ndInt32 trajectoryAccumulatorCount = m_trajectoryAccumulator.GetCount();
	if ((m_bashTrajectoryIndex >= m_bashTrajectoryCount) && (trajectoryAccumulatorCount >= m_bashTrajectorySteps))
	{
		Optimize();

		m_eposideCount++;
		m_framesAlive = 0;
		m_bashTrajectoryIndex = 0;
		m_trajectoryAccumulator.SetCount(0);
		for (ndList<ndBrainAgentContinueProximaPolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentContinueProximaPolicyGradient_Trainer* const agent = node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
		}
	}
}

#endif

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters)
	,m_oldPolicy(m_policy)
	,m_tempPolicy(m_policy)
{
}

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::~ndBrainAgentContinueProximaPolicyGradient_TrainerMaster()
{
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::CalculateKLdivergence()
{
	//https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	// since I am using a diagonal sigma, I do not have to use cholesky 

	ndAtomic<ndInt32> iterator(0);
	ndFloat64 partialDivergence[256];

	auto ParcialDivergence = ndMakeObject::ndFunction([this, &iterator, &partialDivergence](ndInt32 threadIndex, ndInt32)
	{
		ndFloat64 totalDivergece = ndFloat32(0.0f);
		ndInt32 size = m_trajectoryAccumulator.GetCount();

		ndBrainFloat crossProbabilitiesBuffer[256];
		ndBrainMemVector crossProbabilities(&crossProbabilitiesBuffer[0], m_numberOfActions * 2);
		for (ndInt32 i = iterator++; i < size; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			m_policy.MakePrediction(observation, crossProbabilities);
			//m_tempPolicy.MakePrediction(observation, crossProbabilities);
			ndBrainMemVector entropyProbabilities(m_trajectoryAccumulator.GetProbabilityDistribution(i), m_numberOfActions * 2);

			// caculate t0 = trace(inv(Sigma_q) * Sigma_p
			// caculate t1 = trans(Uq - Up) * inv(Sigma_q) * (Uq - Up)
			// caculate t2 = log(det(Sigma_q) /det(Sigma_p))
			ndFloat64 t0 = 0.0f;
			ndFloat64 t1 = 0.0f;
			ndFloat64 det_p = 1.0f;
			ndFloat64 det_q = 1.0f;
			for (ndInt32 j = m_numberOfActions - 1; j >= 0; --j)
			{
				ndBrainFloat sigma_q = crossProbabilities[j + m_numberOfActions];
				ndBrainFloat sigma_p = entropyProbabilities[j + m_numberOfActions];
				ndBrainFloat invSigma_q = 1.0f / sigma_q;

				det_p *= sigma_p;
				det_q *= sigma_q;
				t0 += sigma_p * invSigma_q;
				ndBrainFloat meanError(crossProbabilities[j] - entropyProbabilities[j]);
				t1 += meanError * invSigma_q * meanError;
			}
			ndFloat64 t2 = ndLog(det_q / det_p);
			totalDivergece += ndBrainFloat(0.5f) * (t0 - ndBrainFloat(m_numberOfActions) + t1 + t2);
		}
		partialDivergence[threadIndex] = totalDivergece;
	});
	ndBrainThreadPool::ParallelExecute(ParcialDivergence);

	ndFloat64 divergence = ndFloat32(0.0f);
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		divergence += partialDivergence[i];
	}
	ndAssert(divergence >= 0.0f);
	divergence /= ndFloat64(m_trajectoryAccumulator.GetCount());
	//divergence = 0;
	return ndBrainFloat(divergence);
}

//#pragma optimize( "", off )
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizePolicyPPOstep()
{
	ndAtomic<ndInt32> iterator(0);

	m_tempPolicy.CopyFrom(m_policy);
	m_policy.CopyFrom(m_oldPolicy);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_policyTrainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount());
	for (ndInt32 base = 0; base < steps; base += m_bashBufferSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class MaxLikelihoodLoss : public ndBrainLoss
			{
				public:
				MaxLikelihoodLoss(ndBrainTrainer& trainer, ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* const agent, ndInt32 index)
					:ndBrainLoss()
					,m_trainer(trainer)
					,m_agent(agent)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					// basically this fits a multivariate Gaussian process with zero cross covariance to the actions.
					const ndInt32 numberOfActions = m_agent->m_numberOfActions;
					const ndBrainMemVector sampledProbability(m_agent->m_trajectoryAccumulator.GetActions(m_index), numberOfActions * 2);
					const ndBrainMemVector oldProbabilityDistribution(m_agent->m_trajectoryAccumulator.GetProbabilityDistribution(m_index), numberOfActions * 2);

					ndBrainMemVector newPolicyDistribution(&newPolicyDistributionBuffer[0], m_agent->m_numberOfActions * 2);
					const ndBrainMemVector observation(m_agent->m_trajectoryAccumulator.GetObservations(m_index), m_agent->m_numberOfObservations);
					m_agent->m_tempPolicy.MakePrediction(observation, newPolicyDistribution);

					ndFloat32 det_p = ndFloat32(1.0f);
					ndFloat32 det_q = ndFloat32(1.0f);
					ndFloat32 exp_p = ndFloat32(0.0f);
					ndFloat32 exp_q = ndFloat32(0.0f);
					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						ndFloat32 prob_p = sampledProbability[i] - oldProbabilityDistribution[i];
						ndFloat32 sigma_p = oldProbabilityDistribution[i + numberOfActions];
						det_p *= sigma_p;
						exp_p += prob_p * sigma_p * prob_p;

						ndFloat32 prob_q = sampledProbability[i] - newPolicyDistribution[i];
						ndFloat32 sigma_q = newPolicyDistribution[i + numberOfActions];
						det_q *= sigma_q;
						exp_q += prob_q * sigma_q * prob_q;
					}

					//ndFloat32 prob_p = ndExp(-exp_p * 0.5f) / ndSqrt(((2.0f * ndPi) ^ numberOfActions) * det_p);
					//the constant ((2.0f * ndPi) ^ numberOfActions) cancel each other in the ratio
					ndFloat32 prob_p = ndExp(-exp_p * ndFloat32(0.5f)) / ndSqrt(det_p);
					ndFloat32 prob_q = ndExp(-exp_q * ndFloat32(0.5f)) / ndSqrt(det_q);

					ndBrainFloat rawAdvantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					ndBrainFloat r = prob_p / prob_q;
					//ndBrainFloat r = prob_q / prob_p; // this is fucking wrong
					ndBrainFloat g = (rawAdvantage > ndBrainFloat(0.0f)) ? ndBrainFloat(1.0 + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON) : ndBrainFloat(1.0 - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
					// calculate clipped advantage
					ndBrainFloat advantage = ndMin(r * rawAdvantage, g * rawAdvantage);

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = probabilityDistribution[i];
						const ndBrainFloat sigma1 = probabilityDistribution[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat sigma3 = sigma2 * sigma1;
						const ndBrainFloat num = (sampledProbability[i] - mean);

						// this was a huge bug, it is gradient ascend
						ndBrainFloat meanGradient = -num / sigma2;
						ndBrainFloat sigmaGradient = num * num / sigma3 - ndBrainFloat(1.0f) / sigma1;

						loss[i] = -meanGradient * advantage;
						loss[i + numberOfActions] = -sigmaGradient * advantage;
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
				ndBrainFloat newPolicyDistributionBuffer[256];
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				MaxLikelihoodLoss loss(trainer, this, base + i);
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(base + i), m_numberOfObservations);
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
				ndBrainTrainer* const trainer = m_policyTrainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}

	m_policy.CopyFrom(m_tempPolicy);
	m_policyOptimizer->AccumulateGradients(this, m_policyTrainers);
	m_policyWeightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, -m_policyLearnRate);
}


void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::Optimize()
{
	m_oldPolicy.CopyFrom(m_policy);

	OptimizeCritic();
	OptimizePolicy();
	for (ndInt32 i = 0; (i < ND_CONTINUE_PROXIMA_POLICY_ITERATIONS) && (CalculateKLdivergence() < ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE); ++i)
	{
		OptimizePolicyPPOstep();
	}
}
