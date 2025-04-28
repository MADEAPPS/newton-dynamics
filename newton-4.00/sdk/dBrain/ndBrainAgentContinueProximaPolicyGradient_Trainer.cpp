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

#define ND_CONTINUE_PROXIMA_POLICY_ITERATIONS			100
#define ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE		ndBrainFloat(0.001f)
//#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON			ndBrainFloat(0.2f)
#define ND_CONTINUE_PROXIMA_POLICY_LEARN_SCALE			ndBrainFloat(0.125f)

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters)
	,m_policyActions()
{
}

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::~ndBrainAgentContinueProximaPolicyGradient_TrainerMaster()
{
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::CalculateKLdivergence()
{
	// https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	// since I am using a diagonal sigma, I do not have to use Cholesky 

	ndAtomic<ndInt32> iterator(0);
	ndFloat64 partialDivergence[256];

	auto ParcialDivergence = ndMakeObject::ndFunction([this, &iterator, &partialDivergence](ndInt32 threadIndex, ndInt32)
	{
		ndFloat64 totalDivergence = ndFloat32(0.0f);
		ndInt32 size = m_trajectoryAccumulator.GetCount();
		
		ndBrainFloat crossProbabilitiesBuffer[256];
		ndBrainMemVector crossProbabilities(&crossProbabilitiesBuffer[0], m_parameters.m_numberOfActions * 2);
		for (ndInt32 i = iterator++; i < size; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
			m_policy.MakePrediction(observation, crossProbabilities);
			const ndBrainMemVector probabilities(&m_policyActions[i * m_parameters.m_numberOfActions * 2], m_parameters.m_numberOfActions * 2);
		
			// calculate t0 = trace(inv(Sigma_q) * Sigma_p
			// calculate t1 = numberOfActions
			// calculate t2 = trans(Uq - Up) * inv(Sigma_q) * (Uq - Up)
			// calculate t3 = log(det(Sigma_q) /det(Sigma_p))
			ndFloat32 t0 = 0.0f;
			ndFloat32 t2 = 0.0f;
			ndFloat32 log_det_p = 0.0f;
			ndFloat32 log_det_q = 0.0f;
			for (ndInt32 j = m_parameters.m_numberOfActions - 1; j >= 0; --j)
			{
				ndBrainFloat sigma_p = probabilities[j + m_parameters.m_numberOfActions];
				ndBrainFloat sigma_q = crossProbabilities[j + m_parameters.m_numberOfActions];
				ndBrainFloat invSigma_q = 1.0f / sigma_q;
		
				log_det_p += log(sigma_p);
				log_det_q += log(sigma_q);
				t0 += sigma_p * invSigma_q;
				ndBrainFloat meanError(crossProbabilities[j] - probabilities[j]);
				t2 += meanError * invSigma_q * meanError;
			}
			//ndFloat64 t3 = ndLog(det_q / det_p);
			ndFloat64 t3 = log_det_q - log_det_p;
			ndFloat64 t1 = ndBrainFloat(m_parameters.m_numberOfActions);
			totalDivergence += ndBrainFloat(0.5f) * (t0 - t1 + t2 + t3);
		}
		partialDivergence[threadIndex] = totalDivergence;
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

//#pragma optimize( "", off )
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::CalculateGradients()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_policyTrainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_advantage.GetCount() + m_parameters.m_miniBatchSize - 1) & -m_parameters.m_miniBatchSize;
	for (ndInt32 base = 0; base < steps; base += m_parameters.m_miniBatchSize)
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
					const ndInt32 numberOfActions = m_agent->m_policy.GetOutputSize();
					const ndBrainFloat advantage = m_agent->m_advantage[m_index];
					const ndBrainMemVector sampledProbability(m_agent->m_trajectoryAccumulator.GetActions(m_index), numberOfActions);

					//negate the gradient for gradient ascend?
					ndBrainFloat ascend = ndBrainFloat(-1.0f);
					ndBrainFloat sigmaGrad2Z = ndBrainFloat(0.0f);
					ndBrainFloat sigma2 = probabilityDistribution[numberOfActions - 1];
					ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / sigma2;
					for (ndInt32 i = numberOfActions - 2; i >= 0; --i)
					{
						// as I understand, this is just a special case of maximum likelihood optimization.
						// given a multivariate Gaussian process with zero cross covariance to the actions.
						// calculate the log of prob of a multivariate Gaussian

						const ndBrainFloat mean = probabilityDistribution[i];
						ndBrainFloat confidence = sampledProbability[i] - mean;
						sigmaGrad2Z += confidence * confidence;

						ndBrainFloat meanGradient = confidence * invSigma2;
						loss[i] = meanGradient * advantage * ascend;
					}
					ndBrainFloat sigmaGrad = ndBrainFloat(0.5f) * invSigma2 * (sigmaGrad2Z * invSigma2 - ndBrainFloat(numberOfActions - 1));
					loss[numberOfActions - 1] = sigmaGrad * advantage * ascend;
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndInt32 index = base + i;
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				MaxLikelihoodLoss loss(trainer, this, index);
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
				trainer.BackPropagate(observation, loss);
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
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
	m_policyTrainers[1]->CopyGradients(m_policyWeightedTrainer[0]);
}

void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::CalculateAdvange()
{
	ndBrainAgentContinuePolicyGradient_TrainerMaster::CalculateAdvange();

	ndInt32 numberOfActions = m_policy.GetOutputSize();
	if (m_policyActions.GetCount() < m_parameters.m_miniBatchSize * numberOfActions)
	{
		ndInt32 start = 0;
		for (ndInt32 i = ndInt32(m_policyActions.GetCount()); i < m_parameters.m_miniBatchSize * numberOfActions; ++i)
		{
			ndBrainFloat advantage = m_policyActions[start];
			m_policyActions.PushBack(advantage);
			start++;
		}
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::Optimize()
{
	ndAtomic<ndInt32> iterator(0);
	ndInt32 numberOfActions = m_policy.GetOutputSize();
	m_policyActions.SetCount(m_trajectoryAccumulator.GetCount() * numberOfActions);
	auto CalculateActionsDistribution = ndMakeObject::ndFunction([this, &iterator, numberOfActions](ndInt32, ndInt32)
	{
		const ndInt32 size = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = iterator++; i < size; i = iterator++)
		{
			ndBrainMemVector probabilities(&m_policyActions[i * numberOfActions], numberOfActions);
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
			m_policy.MakePrediction(observation, probabilities);
		}
	});
	ndBrainThreadPool::ParallelExecute(CalculateActionsDistribution);

	OptimizeCritic();
	CalculateAdvange();
	CalculateGradients();
	ndBrainFloat learnRate = m_parameters.m_policyLearnRate * ND_CONTINUE_PROXIMA_POLICY_LEARN_SCALE;
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, learnRate);
	for (ndInt32 i = ND_CONTINUE_PROXIMA_POLICY_ITERATIONS; (i >= 0) && (CalculateKLdivergence() < ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE); --i)
	{
		m_policyWeightedTrainer[0]->CopyGradients(m_policyTrainers[1]);
		m_policyOptimizer->Update(this, m_policyWeightedTrainer, learnRate);
	}
}
