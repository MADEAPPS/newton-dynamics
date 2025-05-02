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

ndBrainAgentSoftActorCritic_Trainer::ndBrainAgentSoftActorCritic_Trainer(const HyperParameters& parameters)
	:ndBrainAgentDeterministicPolicyGradient_Trainer(parameters)
{
}

ndBrainAgentSoftActorCritic_Trainer::~ndBrainAgentSoftActorCritic_Trainer()
{
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentSoftActorCritic_Trainer::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& sampledActions)
{
	ndBrainFixSizeVector<256> distribution;
	distribution.SetCount(m_policy.GetOutputSize());
	const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), m_policy.GetInputSize());
	m_policy.MakePrediction(observation, distribution);

	ndBrainFloat z2 = 0.0f;
	ndBrainFloat invSigma2Det = ndBrainFloat(1.0f);

	const ndInt32 count = ndInt32(distribution.GetCount()) / 2;
	const ndInt32 start = ndInt32(distribution.GetCount()) / 2;
	ndFloat32 invSqrtPi = ndSqrt(1.0f / (2.0f * ndPi));
	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		ndFloat32 sigma = distribution[start + i];
		ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
		ndBrainFloat z = (sampledActions[i] - distribution[i]) * invSigma;

		z2 += z * z;
		invSigma2Det *= (invSqrtPi * invSigma);
	}
	ndBrainFloat exponent = ndBrainFloat(0.5f) * z2;
	ndBrainFloat prob = invSigma2Det * ndExp(-exponent);
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

	m_expectedRewardsIndexBuffer.SetCount(0);
	for (ndInt32 i = 0; i < count; ++i)
	{
		m_expectedRewardsIndexBuffer.PushBack(m_shuffleBuffer[m_shuffleIndexBuffer]);
		m_shuffleIndexBuffer = (m_shuffleIndexBuffer + 1) % ndInt32(m_shuffleBuffer.GetCount());
	}

	ndAtomic<ndInt32> iterator(0);
	auto ExpectedRewards = ndMakeObject::ndFunction([this, count, &iterator](ndInt32, ndInt32)
	{
		ndBrainFixSizeVector<256> policyEntropyAction;
		ndBrainFixSizeVector<256> criticObservationAction;
		
		const ndInt32 batchSize = 128;
		policyEntropyAction.SetCount(m_policy.GetOutputSize());
		criticObservationAction.SetCount(m_policy.GetOutputSize() + m_policy.GetInputSize());
		for (ndInt32 k = iterator.fetch_add(batchSize); k < count; k = iterator.fetch_add(batchSize))
		{
			const ndInt32 size = (k + batchSize) < count ? k + batchSize : count;
			for (ndInt32 j = k; j < size; ++j)
			{
				ndBrainFixSizeVector<ND_NUMBER_OF_CRITICS> rewards;
				rewards.SetCount(0);
				const ndInt32 index = m_expectedRewardsIndexBuffer[j];
				for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
				{
					rewards.PushBack(m_replayBuffer.GetReward(index));
				}
				if (!m_replayBuffer.GetTerminalState(index))
				{
					ndBrainMemVector nextAction(&criticObservationAction[0], m_policy.GetOutputSize());
					const ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(index), m_policy.GetInputSize());
					m_policy.MakePrediction(nextObservation, nextAction);
					ndMemCpy(&criticObservationAction[m_policy.GetOutputSize()], &nextObservation[0], nextObservation.GetCount());

					ndBrainFixSizeVector<1> criticQvalue;
					for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
					{
						m_referenceCritic[i].MakePrediction(criticObservationAction, criticQvalue);
						rewards[i] += m_parameters.m_discountRewardFactor * criticQvalue[0];
					}
				}

				ndFloat32 minQ = ndFloat32(1.0e10f);
				for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
				{
					minQ = ndMin(minQ, rewards[i]);
				}

				// calculate entropy
				ndBrainFloat prob = CalculatePolicyProbability(index);
				ndBrainFloat logProb = ndLog(prob);
				ndBrainFloat entropyRegularizedReward = minQ - m_parameters.m_entropyRegularizerCoef * logProb;
				m_expectedRewards[j] = entropyRegularizedReward;
			}
		}
	});
	ndBrainThreadPool::ParallelExecute(ExpectedRewards);

	ndFloat32 rewardSum = 0.0f;
	for (ndInt32 i = ndInt32(m_expectedRewards.GetCount()) - 1; i >= 0; --i)
	{
		rewardSum += m_expectedRewards[i];
	}
	ndFloat32 averageReward = rewardSum / ndFloat32(m_expectedRewards.GetCount());
	m_averageExpectedRewards.Update(averageReward);
}

//#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::LearnPolicyFunction()
{
	ndAtomic<ndInt32> iterator(0);
	m_reparametizedActions.SetCount(m_policy.GetOutputSize() * m_parameters.m_miniBatchSize);
	for (ndInt32 n = m_parameters.m_policyUpdatesCount - 1; n >= 0; --n)
	{
		ndFixSizeArray<ndInt32, 1024> indirectBuffer;
		for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
		{
			indirectBuffer.PushBack(m_shuffleBuffer[m_shuffleIndexBuffer]);
			m_shuffleIndexBuffer = (m_shuffleIndexBuffer + 1) % ndInt32(m_shuffleBuffer.GetCount());
		}
	
		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, &indirectBuffer](ndInt32, ndInt32)
		{
			class ndLoss : public ndBrainLossLeastSquaredError
			{
				public:
				ndLoss(ndBrainAgentSoftActorCritic_Trainer* const owner, const ndInt32 index, ndInt32 batchIndex)
					:ndBrainLossLeastSquaredError(1)
					,m_criticLoss(owner, index)
					,m_owner(owner)
					,m_combinedActionObservation()
					,m_index(index)
					,m_batchIndex(batchIndex)
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
	
				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					ndMemCpy(&m_tempCombinedInputGradients[0], &m_owner->m_reparametizedActions[m_batchIndex * m_owner->m_policy.GetOutputSize()], m_owner->m_policy.GetOutputSize());
					ndMemCpy(&m_combinedActionObservation[m_owner->m_policy.GetOutputSize()], m_owner->m_replayBuffer.GetNextObservations(m_index), m_owner->m_policy.GetInputSize());
	
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
					
					// calculate and add the Gradient of entropy (grad of log probability)
					const ndInt32 count = ndInt32(m_owner->m_policy.GetOutputSize()) / 2;
					const ndInt32 start = ndInt32(m_owner->m_policy.GetOutputSize()) / 2;
					const ndBrainMemVector reparametizedActions(&m_owner->m_reparametizedActions[m_batchIndex], count);
					for (ndInt32 i = count - 1; i >= 0; --i)
					{
						ndBrainFloat sigma = output[i + start];
						ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;;
						ndBrainFloat z = output[i] - reparametizedActions[i];
						ndBrainFloat meanGrad = z * invSigma * invSigma;
						ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

						loss[i] -= m_owner->m_parameters.m_entropyRegularizerCoef * meanGrad;
						loss[i + start] -= m_owner->m_parameters.m_entropyRegularizerCoef * sigmaGrad;
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
				const ndInt32 index = indirectBuffer[i];
				ndBrainTrainer& trainer = *m_policyTrainers[i];
				const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), m_policy.GetInputSize());
	
				ndLoss loss(this, index, i);
				trainer.BackPropagate(observation, loss);
			}
		});

		auto CalculateReparametizedSamples = ndMakeObject::ndFunction([this, &iterator, &indirectBuffer](ndInt32, ndInt32)
		{
			ndInt32 actiontSize = m_policy.GetOutputSize();
			ndInt32 observationSize = m_policy.GetInputSize();
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				const ndInt32 index = indirectBuffer[i];
				const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), observationSize);
				ndBrainMemVector reparametizedAction(&m_reparametizedActions[i * actiontSize], actiontSize);
				m_policy.MakePrediction(observation, reparametizedAction);
				m_agent->SampleActions(reparametizedAction);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateReparametizedSamples);
	
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
		m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
	}
}