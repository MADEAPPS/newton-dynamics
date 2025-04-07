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

#define ND_CONTINUE_PROXIMA_POLICY_ITERATIONS			10
#define ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE		ndBrainFloat(0.001f)
#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON			ndBrainFloat(0.2f)

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters)
	,m_oldPolicy(m_policy)
	,m_tempPolicy(m_policy)
{
	m_policyLearnRate *= 0.25f;
	m_criticLearnRate *= 0.25f;
}

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::~ndBrainAgentContinueProximaPolicyGradient_TrainerMaster()
{
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::CalculateKLdivergence()
{
	//https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	// since I am using a diagonal sigma, I do not have to use Cholesky 

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
	return ndBrainFloat(divergence);
}

#pragma optimize( "", off )
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

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount() - 1) & -m_bashBufferSize;
	//m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount());
	//for (ndInt32 i = ndInt32(m_randomPermutation.GetCount()) - 1; i >= 0; --i)
	//{
	//	m_randomPermutation[i] = i;
	//}
	//m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
	//ndInt32 steps = ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE;
	//if (m_randomPermutation.GetCount() < steps)
	//{
	//	steps = ndInt32(m_randomPermutation.GetCount()) & -m_bashBufferSize;
	//}

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

					ndFloat32 oldDet = ndFloat32(1.0f);
					ndFloat32 oldExp = ndFloat32(0.0f);

					ndFloat32 newDet = ndFloat32(1.0f);
					ndFloat32 newExp = ndFloat32(0.0f);
					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						ndFloat32 oldProb = sampledProbability[i] - oldProbabilityDistribution[i];
						ndFloat32 oldSigma = oldProbabilityDistribution[i + numberOfActions];
						oldDet *= oldSigma;
						oldExp += oldProb * oldSigma * oldProb;

						ndFloat32 newProb = sampledProbability[i] - newPolicyDistribution[i];
						ndFloat32 newSigma = newPolicyDistribution[i + numberOfActions];
						newDet *= newSigma;
						newExp += newProb * newSigma * newProb;
					}

					//ndFloat32 prob_p = [1.0 / ndSqrt(((2.0f * ndPi) ^ numberOfActions)] * (1.0 / det) * ndExp(-newExp * 0.5f) ;
					//the constant ((2.0f * ndPi) ^ numberOfActions) cancel each other in the ratio
					ndFloat32 newProb = ndExp(-newExp * ndFloat32(0.5f)) / ndSqrt(newDet);
					ndFloat32 oldProb = ndExp(-oldExp * ndFloat32(0.5f)) / ndSqrt(oldDet);

					ndBrainFloat rawAdvantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					ndBrainFloat r = newProb / oldProb;
					ndBrainFloat g = (rawAdvantage > ndBrainFloat(0.0f)) ? ndBrainFloat(1.0 + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON) : ndBrainFloat(1.0 - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
					// calculate clipped advantage
					ndBrainFloat advantage = ndMin(r * rawAdvantage, g * rawAdvantage);

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = probabilityDistribution[i];
						const ndBrainFloat sigma1 = probabilityDistribution[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat num = sampledProbability[i] - mean;

						ndBrainFloat meanGradient = num / sigma1;
						ndBrainFloat sigmaGradient = ndBrainFloat(0.5f) * (num * num / sigma2 - ndBrainFloat(1.0f) / sigma1);

						loss[i] = meanGradient * advantage;
						loss[i + numberOfActions] = sigmaGradient * advantage;
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
				//ndInt32 index = m_randomPermutation[base + i];
				ndInt32 index = base + i;
				MaxLikelihoodLoss loss(trainer, this, index);
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
				trainer.BackPropagate(observation, loss);
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

//#pragma optimize( "", off )
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizePolicyPPO()
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

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount() - 1) & -m_bashBufferSize;

	//m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount());
	//for (ndInt32 i = ndInt32(m_randomPermutation.GetCount()) - 1; i >= 0; --i)
	//{
	//	m_randomPermutation[i] = i;
	//}
	//m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
	//ndInt32 steps = ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE;
	//if (m_randomPermutation.GetCount() < steps)
	//{
	//	steps = ndInt32(m_randomPermutation.GetCount()) & -m_bashBufferSize;
	//}

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
					// calculate the log of prob of a multivariate Gaussian
					const ndInt32 numberOfActions = m_agent->m_numberOfActions;
					const ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					const ndBrainMemVector sampledProbability(m_agent->m_trajectoryAccumulator.GetActions(m_index), numberOfActions * 2);

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = probabilityDistribution[i];
						const ndBrainFloat sigma1 = probabilityDistribution[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat num = sampledProbability[i] - mean;

						ndBrainFloat meanGradient = num / sigma1;
						ndBrainFloat sigmaGradient = ndBrainFloat(0.5f) * (num * num / sigma2 - ndBrainFloat(1.0f) / sigma1);

						loss[i] = meanGradient * advantage;
						loss[i + numberOfActions] = sigmaGradient * advantage;
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				//ndInt32 index = m_randomPermutation[base + i];
				ndInt32 index = base + i;
				MaxLikelihoodLoss loss(trainer, this, index);
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
				trainer.BackPropagate(observation, loss);
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
	m_policyWeightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(steps));
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, -m_policyLearnRate);
}

void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::Optimize()
{
	m_oldPolicy.CopyFrom(m_policy);

	CalculateAdvange();
	OptimizePolicyPPO();
	for (ndInt32 i = ND_CONTINUE_PROXIMA_POLICY_ITERATIONS; (i >= 0) && (CalculateKLdivergence() < ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE); --i)
	{
		OptimizePolicyPPOstep();
	}
	OptimizeCritic();
}
