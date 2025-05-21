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

#define ND_CONTINUE_PROXIMA_POLICY_ENTROPY_CONFICIENT	ndBrainFloat (2.0e-5f)

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters)
	,m_policyActions()
	,m_referenceProbability()
{
	ndBrainFloat unitEntropy = ndClamp(m_parameters.m_entropyRegularizerCoef, ndBrainFloat(0.0f), ndBrainFloat(1.0f));
	m_parameters.m_entropyRegularizerCoef = ND_CONTINUE_PROXIMA_POLICY_ENTROPY_CONFICIENT * unitEntropy;
}

ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::~ndBrainAgentContinueProximaPolicyGradient_TrainerMaster()
{
}

ndBrainFloat ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution)
{
	ndBrainFloat z2 = ndBrainFloat(0.0f);
	ndBrainFloat invSigma2Det = ndBrainFloat(1.0f);
	ndBrainFloat invSqrtPi = ndBrainFloat(1.0f) / ndSqrt(2.0f * ndPi);

	ndBrainFloat prob = 1.0f;
	if (m_parameters.m_usePerActionSigmas)
	{
		const ndInt32 size = ndInt32(distribution.GetCount()) / 2;

		const ndBrainMemVector sampledProbabilities(m_trajectoryAccumulator.GetActions(index), m_policy.GetOutputSize());
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			ndBrainFloat sigma = distribution[i + size];
			ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
			ndBrainFloat z = (sampledProbabilities[i] - distribution[i]) * invSigma;

			z2 += (z * z);
			invSigma2Det *= (invSqrtPi * invSigma);
		}
		ndBrainFloat exponent = ndBrainFloat(0.5f) * z2;
		prob = invSigma2Det * ndBrainFloat(ndExp(-exponent));
	}
	else
	{
		const ndInt32 count = ndInt32(distribution.GetCount());

		ndBrainFloat sigma = m_parameters.m_actionFixSigma;
		ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
		const ndBrainMemVector sampledProbabilities(m_trajectoryAccumulator.GetActions(index), m_policy.GetOutputSize());
		for (ndInt32 i = count - 1; i >= 0; --i)
		{
			ndBrainFloat z = (sampledProbabilities[i] - distribution[i]) * invSigma;
			z2 += z * z;
			invSigma2Det *= (invSqrtPi * invSigma);
		}
		ndBrainFloat exponent = ndBrainFloat(0.5f) * z2;
		prob = invSigma2Det * ndBrainFloat(ndExp(-exponent));
	}
	return ndMax(prob, ndBrainFloat(1.0e-4f));
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::CalculateKLdivergence()
{
	// https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	// since I am using a diagonal sigma, I do not have to use Cholesky 

	ndAtomic<ndInt32> iterator(0);
	ndFloat64 partialDivergence[256];
	
	m_policyDivergeActions.SetCount(m_trajectoryAccumulator.GetCount() * m_policy.GetOutputSize());
	auto ParcialDivergence = ndMakeObject::ndFunction([this, &iterator, &partialDivergence](ndInt32 threadIndex, ndInt32)
	{
		ndFloat64 totalDivergence = ndFloat32(0.0f);
		ndInt32 count = m_trajectoryAccumulator.GetCount();
		
		ndInt32 numberOfActions = m_policy.GetOutputSize();
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
			ndBrainMemVector crossProbabilities(&m_policyDivergeActions[i * numberOfActions], numberOfActions);
			m_policy.MakePrediction(observation, crossProbabilities);
			const ndBrainMemVector probabilities(&m_policyActions[i * numberOfActions], numberOfActions);
			
			// calculate t0 = trace(inv(Sigma_q) * Sigma_p
			// calculate t1 = numberOfActions
			// calculate t2 = trans(Uq - Up) * inv(Sigma_q) * (Uq - Up)
			// calculate t3 = log(det(Sigma_q) /det(Sigma_p))
			ndFloat32 t0 = ndFloat32(0.0f);
			ndFloat32 t2 = ndFloat32(0.0f);
			ndFloat32 log_det_p = ndFloat32(0.0f);
			ndFloat32 log_det_q = ndFloat32(0.0f);
			
			const ndInt32 size = numberOfActions / 2;
			for (ndInt32 j = size - 1; j >= 0; --j)
			{
				ndBrainFloat sigma_p = probabilities[size + j];
				ndBrainFloat sigma_q = crossProbabilities[size + j];
				ndBrainFloat invSigma_q = 1.0f / sigma_q;
			
				log_det_p += ndLog(sigma_p);
				log_det_q += ndLog(sigma_q);
				t0 += sigma_p * invSigma_q;
				ndBrainFloat meanError(crossProbabilities[j] - probabilities[j]);
				t2 += meanError * invSigma_q * meanError;
			}

			// it does not really matter  the ratio is inverted since KLis a distance, 
			// the only problem is that KL(p/q) is different that KL(q/p)
			// but the distance still represent how close are the two distributions.
			//ndFloat64 t3 = ndLog(det_q / det_p);
			ndFloat64 t3 = log_det_q - log_det_p;
			ndFloat64 t1 = ndBrainFloat(size);
			ndFloat64 divergence = t0 - t1 + t2 + t3;
			totalDivergence += divergence;
		}
		partialDivergence[threadIndex] = ndBrainFloat(0.5f) * totalDivergence;
	});

	auto ParcialDivergenceFixSigma = ndMakeObject::ndFunction([this, &iterator, &partialDivergence](ndInt32 threadIndex, ndInt32)
	{
		ndFloat64 totalDivergence = ndFloat32(0.0f);
		ndInt32 count = m_trajectoryAccumulator.GetCount();

		ndInt32 numberOfActions = m_policy.GetOutputSize();
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
			ndBrainMemVector crossProbabilities(&m_policyDivergeActions[i * numberOfActions], numberOfActions);
			m_policy.MakePrediction(observation, crossProbabilities);
			const ndBrainMemVector probabilities(&m_policyActions[i * numberOfActions], numberOfActions);

			// calculate t0 = trace(inv(Sigma_q) * Sigma_p
			// calculate t1 = numberOfActions
			// calculate t2 = trans(Uq - Up) * inv(Sigma_q) * (Uq - Up)
			// calculate t3 = log(det(Sigma_q) /det(Sigma_p))
			ndFloat32 t0 = ndFloat32(0.0f);
			ndFloat32 t2 = ndFloat32(0.0f);
			ndFloat32 log_det_p = ndFloat32(0.0f);
			ndFloat32 log_det_q = ndFloat32(0.0f);

			ndBrainFloat sigma_p = m_parameters.m_actionFixSigma;
			ndBrainFloat sigma_q = m_parameters.m_actionFixSigma;
			ndBrainFloat invSigma_q = 1.0f / sigma_q;
			ndFloat32 logSigmap = ndLog(sigma_p);
			ndFloat32 logSigmaq = ndLog(sigma_q);

			const ndInt32 size = numberOfActions;
			for (ndInt32 j = size - 1; j >= 0; --j)
			{
				log_det_p += logSigmap;
				log_det_q += logSigmaq;
				t0 += sigma_p * invSigma_q;
				ndBrainFloat meanError(crossProbabilities[j] - probabilities[j]);
				t2 += meanError * invSigma_q * meanError;
			}

			// it does not really matter  the ratio is inverted since KLis a distance, 
			// the only problem is that KL(p/q) is different that KL(q/p)
			// but the distance still represent how close are the two distributions.
			//ndFloat64 t3 = ndLog(det_q / det_p);
			ndFloat64 t3 = log_det_q - log_det_p;
			ndFloat64 t1 = ndBrainFloat(size);
			ndFloat64 divergence = t0 - t1 + t2 + t3;
			totalDivergence += divergence;
		}
		partialDivergence[threadIndex] = ndBrainFloat(0.5f) * totalDivergence;
	});

	if (m_parameters.m_usePerActionSigmas)
	{
		ndBrainThreadPool::ParallelExecute(ParcialDivergence);
	}
	else
	{
		ndBrainThreadPool::ParallelExecute(ParcialDivergenceFixSigma);
	}
	
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
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizedSurrogate()
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

	const ndInt32 steps = m_trajectoryAccumulator.GetCount() & -m_parameters.m_miniBatchSize;
	ndBrainFloat gradientScale = ndBrainFloat(m_parameters.m_miniBatchSize) / ndBrainFloat(steps);
	for (ndInt32 base = 0; base < steps; base += m_parameters.m_miniBatchSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class MaxLikelihoodLoss : public ndBrainLoss
			{
				public:
				MaxLikelihoodLoss(ndBrainTrainer& trainer, ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* const owner, ndInt32 index)
					:ndBrainLoss()
					,m_trainer(trainer)
					,m_owner(owner)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					// as I understand it, this is just a special case of maximum likelihood optimization.
					// given a multivariate Gaussian process with zero cross covariance to the actions.
					// calculate the log of prob of a multivariate Gaussian

					const ndInt32 numberOfActions = m_owner->m_policy.GetOutputSize();
					// calculate clipped surrogate advantage loss.
					const ndBrainMemVector newProbabilityDistribution(&m_owner->m_policyDivergeActions[m_index * numberOfActions], numberOfActions);
					ndBrainFloat prob = m_owner->CalculatePolicyProbability(m_index, newProbabilityDistribution);
					const ndBrainFloat referenceAdvantage = m_owner->m_advantage[m_index];
					ndBrainFloat r = prob / m_owner->m_referenceProbability[m_index];
					if (referenceAdvantage >= 0.0f)
					{
						r = ndMin (r, ndBrainFloat(1.0f) + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
					}
					else if (referenceAdvantage < 0.0f)
					{
						r = ndMax(r, ndBrainFloat(1.0f) - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
					}

					// calculate grad of probability	
					const ndBrainMemVector sampledProbability(m_owner->m_trajectoryAccumulator.GetActions(m_index), numberOfActions);

					if (m_owner->m_parameters.m_usePerActionSigmas)
					{
						const ndInt32 size = numberOfActions / 2;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat sigma = probabilityDistribution[size + i];
							ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma * invSigma;
							ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

							loss[i] = meanGrad;
							loss[size + i] = sigmaGrad;
						}
					}
					else
					{
						ndFloat32 fixSigma = m_owner->m_parameters.m_actionFixSigma;
						ndFloat32 sigma2 = fixSigma * fixSigma;
						ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / sigma2;
						const ndInt32 size = numberOfActions;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma2;
							loss[i] = meanGrad;
						}
					}

					if (m_owner->m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
					{
						// calculate and add the Gradient of entropy (grad of log probability)
						if (m_owner->m_parameters.m_usePerActionSigmas)
						{
							ndBrainFloat entropyRegularizerCoef = m_owner->m_parameters.m_entropyRegularizerCoef;
							const ndInt32 size = numberOfActions / 2;
							for (ndInt32 i = size - 1; i >= 0; --i)
							{
								ndBrainFloat sigma = probabilityDistribution[size + i];
								ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;;
								ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
								ndBrainFloat meanGrad = z * invSigma * invSigma;
								ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

								loss[i] -= entropyRegularizerCoef * meanGrad;
								loss[size + i] -= entropyRegularizerCoef * sigmaGrad;
							}
						}
					}

					//negate the gradient for gradient ascend?
					const ndBrainFloat advantage = r * referenceAdvantage;
					ndBrainFloat ascend = ndBrainFloat(-1.0f) * advantage;
					loss.Scale(ascend);
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* m_owner;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				ndInt32 index = base + i;
				MaxLikelihoodLoss loss(trainer, this, index);
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
				trainer.BackPropagate(observation, loss);
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator, gradientScale](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_policyTrainers[i];
				ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
				auxiliaryTrainer->ScaleWeights(gradientScale);
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}
	m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
}

void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizePolicy()
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

	if (m_trajectoryAccumulator.GetCount() < m_parameters.m_miniBatchSize)
	{
		ndInt32 padTransitions = m_parameters.m_miniBatchSize - m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = 0; i < padTransitions; ++i)
		{
			ndInt32 index = m_trajectoryAccumulator.GetCount();
			m_trajectoryAccumulator.SetCount(index + 1);
			m_trajectoryAccumulator.CopyFrom(index, m_trajectoryAccumulator, i);

			ndBrainFloat advantage = m_advantage[i];
			m_advantage.PushBack(advantage);
		}
	}

	const ndInt32 steps = m_trajectoryAccumulator.GetCount() & -m_parameters.m_miniBatchSize;
	ndBrainFloat gradientScale = ndBrainFloat(m_parameters.m_miniBatchSize) / ndBrainFloat(steps);

	m_referenceProbability.SetCount(steps);
	m_policyActions.SetCount(m_trajectoryAccumulator.GetCount() * m_policy.GetOutputSize());

	for (ndInt32 base = 0; base < steps; base += m_parameters.m_miniBatchSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class MaxLikelihoodLoss : public ndBrainLoss
			{
				public:
				MaxLikelihoodLoss(ndBrainTrainer& trainer, ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* const owner, ndInt32 index)
					:ndBrainLoss()
					,m_trainer(trainer)
					,m_owner(owner)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					// calculate the prob and save the base action for future KL iterations
					const ndInt32 numberOfActions = m_owner->m_policy.GetOutputSize();
					ndMemCpy(&m_owner->m_policyActions[m_index * numberOfActions], &probabilityDistribution[0], numberOfActions);
					m_owner->m_referenceProbability[m_index] = m_owner->CalculatePolicyProbability(m_index, probabilityDistribution);

					// as I understand it, this is just a special case of maximum likelihood optimization.
					// given a multivariate Gaussian process with zero cross covariance to the actions.

					const ndBrainMemVector sampledProbability(m_owner->m_trajectoryAccumulator.GetActions(m_index), numberOfActions);
					if (m_owner->m_parameters.m_usePerActionSigmas)
					{
						const ndInt32 size = numberOfActions / 2;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat sigma = probabilityDistribution[size + i];
							ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma * invSigma;
							ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

							loss[i] = meanGrad;
							loss[size + i] = sigmaGrad;
						}
					}
					else
					{
						ndFloat32 fixSigma = m_owner->m_parameters.m_actionFixSigma;
						ndFloat32 sigma2 = fixSigma * fixSigma;
						ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / sigma2;
						
						const ndInt32 size = numberOfActions;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma2;
							loss[i] = meanGrad;
						}
					}
					
					const ndBrainFloat advantage = m_owner->m_advantage[m_index];

					if (m_owner->m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
					{
						if (m_owner->m_parameters.m_usePerActionSigmas)
						{
							// calculate and add the Gradient of entropy (grad of log probability)
							const ndInt32 size = numberOfActions / 2;
							ndBrainFloat entropyRegularizerCoef = m_owner->m_parameters.m_entropyRegularizerCoef;
							for (ndInt32 i = size - 1; i >= 0; --i)
							{
								ndBrainFloat sigma = probabilityDistribution[size + i];
								ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;;
								ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
								ndBrainFloat meanGrad = z * invSigma * invSigma;
								ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

								loss[i] -= entropyRegularizerCoef * meanGrad;
								loss[size + i] -= entropyRegularizerCoef * sigmaGrad;
							}
						}
					}

					//negate the gradient for gradient ascend?
					ndBrainFloat ascend = ndBrainFloat(-1.0f) * advantage;
					loss.Scale(ascend);
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinueProximaPolicyGradient_TrainerMaster* m_owner;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				ndInt32 index = base + i;
				MaxLikelihoodLoss loss(trainer, this, index);
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
				trainer.BackPropagate(observation, loss);
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator, gradientScale](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_policyTrainers[i];
				ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
				auxiliaryTrainer->ScaleWeights(gradientScale);
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}
	m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
}

void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::OptimizeCritic()
{
	m_randomPermutation.SetCount(0);
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount()) - 1; i >= 0; --i)
	{
		m_randomPermutation.PushBack(i);
	}

	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 iter = m_parameters.m_criticVelueIterations - 1; iter >= 0; --iter)
	{
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
		for (ndInt32 base = 0; base < m_randomPermutation.GetCount(); base += m_parameters.m_miniBatchSize)
		{
			auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainLossLeastSquaredError loss(1);
				ndBrainFixSizeVector<1> stateValue;
				ndBrainFixSizeVector<1> stateQValue;
	
				// calculate GAE(l, 1) // very, very noisy
				// calculate GAE(l, 0) // too smooth, and does not work either
				ndBrainFloat gamma = m_parameters.m_discountRewardFactor * m_parameters.m_generalizedAdvangeDiscount;
	
				for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_criticTrainers[i];
	
					stateValue[0] = m_trajectoryAccumulator.GetReward(index);
					if (!m_trajectoryAccumulator.GetTerminalState(index))
					{
						const ndBrainMemVector nextObservation(m_trajectoryAccumulator.GetNextObservations(index), m_parameters.m_numberOfObservations);
						m_critic.MakePrediction(nextObservation, stateQValue);
						stateValue[0] += gamma * stateQValue[0];
					}
	
					if (m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
					{
						ndBrainFixSizeVector<256> entropyActions;
						entropyActions.SetCount(m_policy.GetOutputSize());

						const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
						m_policy.MakePrediction(observation, entropyActions);
						ndBrainFloat prob = CalculatePolicyProbability(index, entropyActions);
						ndBrainFloat logProb = ndBrainFloat(ndLog(prob));
						stateValue[0] -= m_parameters.m_entropyRegularizerCoef * logProb;
					}
	
					loss.SetTruth(stateValue);
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});

			auto BackPropagateMontecarlo = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainLossLeastSquaredError loss(1);
				ndBrainFixSizeVector<1> stateValue;
				for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_criticTrainers[i];
					stateValue[0] = m_trajectoryAccumulator.GetExpectedReward(index);

					if (m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
					{
						ndBrainFixSizeVector<256> entropyActions;
						entropyActions.SetCount(m_policy.GetOutputSize());

						const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
						m_policy.MakePrediction(observation, entropyActions);
						ndBrainFloat prob = CalculatePolicyProbability(index, entropyActions);
						ndBrainFloat logProb = ndBrainFloat(ndLog(prob));
						stateValue[0] -= m_parameters.m_entropyRegularizerCoef * logProb;
					}

					loss.SetTruth(stateValue);
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});
	
			iterator = 0;
			//ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
			ndBrainThreadPool::ParallelExecute(BackPropagateMontecarlo);
			m_criticOptimizer->Update(this, m_criticTrainers, m_parameters.m_criticLearnRate);
		}
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinueProximaPolicyGradient_TrainerMaster::Optimize()
{
	CalculateAdvange();
	OptimizePolicy();
	ndBrainFloat divergence = CalculateKLdivergence();
	for (ndInt32 i = ND_CONTINUE_PROXIMA_POLICY_ITERATIONS; (i >= 0) && (divergence < ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE); --i)
	{
		OptimizedSurrogate();
		divergence = CalculateKLdivergence();
	}
	OptimizeCritic();
}
