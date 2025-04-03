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
