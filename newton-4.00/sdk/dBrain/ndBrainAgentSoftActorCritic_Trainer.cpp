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

#include "ndBrainLayer.h"
#include "ndBrainTrainer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainAgentSoftActorCritic_Trainer.h"

#define ND_MAX_SAC_ENTROPY_COEFFICIENT	ndBrainFloat (2.0e-5f)

ndBrainAgentSoftActorCritic_Trainer::ndBrainAgentSoftActorCritic_Trainer(const HyperParameters& parameters)
	:ndBrainAgentDeterministicPolicyGradient_Trainer(parameters)
{
	ndBrainFloat unitEntropy = ndClamp(m_parameters.m_entropyRegularizerCoef, ndBrainFloat(0.0f), ndBrainFloat(1.0f));
	m_parameters.m_entropyRegularizerCoef = ND_MAX_SAC_ENTROPY_COEFFICIENT * unitEntropy;
}

ndBrainAgentSoftActorCritic_Trainer::~ndBrainAgentSoftActorCritic_Trainer()
{
}

//#pragma optimize( "", off )
//ndBrainFloat ndBrainAgentSoftActorCritic_Trainer::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& sampledActions)
ndBrainFloat ndBrainAgentSoftActorCritic_Trainer::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution)
{
	ndAssert(0);
	ndBrainFloat z2 = ndBrainFloat(0.0f);
	ndBrainFloat invSigma2Det = ndBrainFloat(1.0f);
	ndBrainFloat invSqrtPi = ndBrainFloat(1.0f) / ndSqrt(2.0f * ndPi);

	ndBrainFloat prob = 1.0f;
	if (m_parameters.m_useSigmasPerActions)
	{
		const ndInt32 size = ndInt32(distribution.GetCount()) / 2;

		const ndBrainMemVector sampledProbabilities(m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
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
		const ndBrainMemVector sampledProbabilities(m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
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
ndBrainFloat ndBrainAgentSoftActorCritic_Trainer::CalculatePolicyProbability(ndInt32 index)
{
	const ndBrainMemVector sampledProbabilities(m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
	return CalculatePolicyProbability(index, sampledProbabilities);
}

//#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::CalculateExpectedRewards()
{
	ndInt32 count = m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize;
	m_expectedRewards.SetCount(count);

	m_miniBatchIndexBuffer.SetCount(0);
	for (ndInt32 i = 0; i < count; ++i)
	{
		m_miniBatchIndexBuffer.PushBack(m_shuffleBuffer[m_shuffleBatchIndex]);
		m_shuffleBatchIndex = (m_shuffleBatchIndex + 1) % ndInt32(m_shuffleBuffer.GetCount());
	}

	ndAtomic<ndInt32> iterator(0);
	auto ExpectedRewards = ndMakeObject::ndFunction([this, count, &iterator](ndInt32, ndInt32)
	{
		const ndInt32 batchSize = 128;
		ndBrainFixSizeVector<256> policyEntropyAction;
		ndBrainFixSizeVector<256> criticNextObservationAction;
		
		ndAssert(count % batchSize == 0);
		policyEntropyAction.SetCount(m_policy.GetOutputSize());
		criticNextObservationAction.SetCount(m_policy.GetOutputSize() + m_policy.GetInputSize());
		for (ndInt32 base = iterator.fetch_add(batchSize); base < count; base = iterator.fetch_add(batchSize))
		{
			for (ndInt32 j = 0; j < batchSize; ++j)
			{
				ndBrainFixSizeVector<ND_NUMBER_OF_CRITICS> rewards;
				rewards.SetCount(0);
				const ndInt32 index = m_miniBatchIndexBuffer[j + base];
				ndBrainFloat r = m_replayBuffer.GetReward(index);
				for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
				{
					rewards.PushBack(r);
				}
				if (!m_replayBuffer.GetTerminalState(index))
				{
					ndBrainMemVector nextAction(&criticNextObservationAction[0], m_policy.GetOutputSize());
					const ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(index), m_policy.GetInputSize());
					m_policy.MakePrediction(nextObservation, nextAction);
					ndMemCpy(&criticNextObservationAction[m_policy.GetOutputSize()], &nextObservation[0], nextObservation.GetCount());

					ndBrainFixSizeVector<1> criticQvalue;
					for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
					{
						m_referenceCritic[i].MakePrediction(criticNextObservationAction, criticQvalue);
						rewards[i] += m_parameters.m_discountRewardFactor * criticQvalue[0];
					}
				}

				ndBrainFloat minQ = ndBrainFloat(1.0e10f);
				for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
				{
					minQ = ndMin(minQ, rewards[i]);
				}

				// calculate entropy
				if (m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
				{
					ndBrainFloat prob = CalculatePolicyProbability(index);
					ndBrainFloat logProb = ndBrainFloat (ndLog(prob));
					minQ -= m_parameters.m_entropyRegularizerCoef * logProb;
				}
				m_expectedRewards[j + base] = minQ;
			}
		}
	});
	ndBrainThreadPool::ParallelExecute(ExpectedRewards);
}

//#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::LearnPolicyFunction()
{
	ndAtomic<ndInt32> iterator(0);
	ndInt32 base = 0;
	for (ndInt32 n = m_parameters.m_policyUpdatesCount - 1; n >= 0; --n)
	{
		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class ndLoss : public ndBrainLossLeastSquaredError
			{
				public:
				ndLoss(ndBrainAgentSoftActorCritic_Trainer* const owner, const ndInt32 index)
					:ndBrainLossLeastSquaredError(1)
					,m_criticLoss(owner, index)
					,m_owner(owner)
					,m_combinedActionObservation()
					,m_index(index)
				{
					m_combinedInputGradients.SetCount(m_owner->m_policy.GetInputSize() + m_owner->m_policy.GetOutputSize());
					m_tempCombinedInputGradients.SetCount(m_owner->m_policy.GetInputSize() + m_owner->m_policy.GetOutputSize());
					m_combinedActionObservation.SetCount(m_owner->m_policy.GetInputSize() + m_owner->m_policy.GetOutputSize());
				}
	
				class ndCriticLoss : public ndBrainLossLeastSquaredError
				{
					public:
					ndCriticLoss(ndBrainAgentSoftActorCritic_Trainer* const owner, ndInt32 index)
						:ndBrainLossLeastSquaredError(1)
						,m_owner(owner)
						,m_index(index)
						,m_saveQValue(ndBrainFloat(0.0f))
					{
					}
	
					void GetLoss(const ndBrainVector& qValue, ndBrainVector& loss)
					{
						loss[0] = 1.0f;
						m_saveQValue = qValue[0];
					}
	
					ndBrainAgentSoftActorCritic_Trainer* m_owner;
					ndInt32 m_index;
					ndBrainFloat m_saveQValue;
				};
	
				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					ndMemCpy(&m_combinedActionObservation[0], &probabilityDistribution[0], m_owner->m_policy.GetOutputSize());
					ndMemCpy(&m_combinedActionObservation[m_owner->m_policy.GetOutputSize()], m_owner->m_replayBuffer.GetObservations(m_index), m_owner->m_policy.GetInputSize());
					
					ndBrainFloat minReward = ndBrainFloat(1.0e10f);
					for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
					{
						m_owner->m_critic[i].CalculateInputGradient(m_combinedActionObservation, m_tempCombinedInputGradients, m_criticLoss);
						if (m_criticLoss.m_saveQValue < minReward)
						{
							minReward = m_criticLoss.m_saveQValue;
							m_combinedInputGradients.Set(m_tempCombinedInputGradients);
						}
					}
					ndMemCpy(&loss[0], &m_combinedInputGradients[0], loss.GetCount());
					
					if (m_owner->m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
					{
						if (m_owner->m_parameters.m_useSigmasPerActions)
						{
							// calculate and add the Gradient of entropy (grad of log probability)
							ndBrainMemVector sampledActions(m_owner->m_replayBuffer.GetActions(m_index), m_owner->m_policy.GetOutputSize());
							ndBrainFloat entropyRegularizerCoef = m_owner->m_parameters.m_entropyRegularizerCoef;

							const ndInt32 size = ndInt32(m_owner->m_policy.GetOutputSize()) / 2;
							for (ndInt32 i = size - 1; i >= 0; --i)
							{
								ndBrainFloat sigma = probabilityDistribution[size + i];
								ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
								ndBrainFloat z = sampledActions[i] - probabilityDistribution[i];
								ndBrainFloat meanGrad = z * invSigma * invSigma;
								ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

								loss[i] -= entropyRegularizerCoef * meanGrad;
								loss[size + i] -= entropyRegularizerCoef * sigmaGrad;
							}
						}
					}
					
					// gradient ascend
					loss.Scale(ndBrainFloat (-1.0f));
				}
	
				ndCriticLoss m_criticLoss;
				ndBrainAgentSoftActorCritic_Trainer* m_owner;
				ndBrainFixSizeVector<256> m_combinedInputGradients;
				ndBrainFixSizeVector<256> m_combinedActionObservation;
				ndBrainFixSizeVector<256> m_tempCombinedInputGradients;
				ndInt32 m_index;
				ndInt32 m_batchIndex;
			};
	
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				const ndInt32 index = m_miniBatchIndexBuffer[i + base];
				ndBrainTrainer& trainer = *m_policyTrainers[i];
				const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), m_policy.GetInputSize());
	
				ndLoss loss(this, index);
				trainer.BackPropagate(observation, loss);
			}
		});
	
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
		m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);

		base += m_parameters.m_miniBatchSize;
	}
}