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

#ifndef _ND_BRAIN_AGENT_TD3_TRAINER_H__
#define _ND_BRAIN_AGENT_TD3_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrainAgentDDPG_Trainer.h"

// this is an implementation of more stable Policy gradoint
// Continuous control with deep re enforcement learning (ddpg agent)
// trainer as described in: https://arxiv.org/pdf/1802.09477.pdf

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentTD3_Trainer: public ndBrainAgentDDPG_Trainer<statesDim, actionDim>
{
	public:
	ndBrainAgentTD3_Trainer(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic);

	protected:
	virtual void BackPropagate();
	void BackPropagateCritic(const ndUnsigned32* const bashIndex);

	ndBrain m_critic2;
	ndBrain m_target2Critic;
	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_critic2Optimizer;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentTD3_Trainer<statesDim, actionDim>::ndBrainAgentTD3_Trainer(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic)
	:ndBrainAgentDDPG_Trainer<statesDim, actionDim>(actor, critic)
	,m_critic2(*(*critic))
	,m_target2Critic(*(*critic))
{
	m_critic2.InitGaussianWeights(ndReal (0.25f));
	m_target2Critic.CopyFrom(m_critic2);

	for (ndInt32 i = 0; i < ndBrainThreadPool::GetThreadCount(); ++i)
	{
		m_critic2Optimizer.PushBack(new ndBrainTrainer(&m_critic2));
		m_critic2Optimizer[m_critic2Optimizer.GetCount() - 1]->SetRegularizer(D_DDPG_REGULARIZER);
	}
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::BackPropagateCritic(const ndUnsigned32* const shuffleBuffer)
{
	auto PropagateBash = ndMakeObject::ndFunction([this, &shuffleBuffer](ndInt32 threadIndex, ndInt32 threadCount)
	{
		class Loss: public ndBrainLeastSquareErrorLoss
		{
			public:
			Loss(ndBrainTrainer& trainer0, ndBrainTrainer& trainer1
				,ndBrain* const targetActor, ndBrain* const targetCritic0, ndBrain* const targetCritic1
				,const ndBrainReplayBuffer<ndReal, statesDim, actionDim>& replayBuffer
				,ndReal gamma, ndFloat32 actionNoiseVariance, ndRandom& randomGenerator)
				:ndBrainLeastSquareErrorLoss(trainer0.GetBrain()->GetOutputSize())
				,m_randomGenerator(randomGenerator)
				,m_criticTrainer0(trainer0)
				,m_criticTrainer1(trainer1)
				,m_targetActor(targetActor)
				,m_targetCritic0(targetCritic0)
				,m_targetCritic1(targetCritic1)
				,m_replayBuffer(replayBuffer)
				,m_gamma(gamma)
				,m_targetValue(0)
				,m_actionNoiseVariance(actionNoiseVariance)
				,m_index(0)
				,m_pass(0)
			{
			}

			void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(loss.GetCount() == 1);
				ndAssert(output.GetCount() == 1);
				ndAssert(m_truth.GetCount() == m_criticTrainer0.GetBrain()->GetOutputSize());
				ndAssert(m_truth.GetCount() == m_criticTrainer1.GetBrain()->GetOutputSize());

				ndReal criticOutputBuffer[2];
				ndDeepBrainMemVector criticOutput(criticOutputBuffer, 1);

				if (m_pass == 0)
				{
					const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_replayBuffer[m_index];
					//criticOutput[0] = transition.m_reward + m_gamma * m_targetValue;
					m_targetValue = transition.m_reward;
					if (!transition.m_terminalState)
					{
						ndReal actorInputBuffer[statesDim * 2];
						ndReal actorOutputBuffer[actionDim * 2];
						ndDeepBrainMemVector actorInput(actorInputBuffer, statesDim);
						ndDeepBrainMemVector actorOutput(actorOutputBuffer, actionDim);

						for (ndInt32 i = 0; i < statesDim; ++i)
						{
							actorInput[i] = transition.m_nextState[i];
						}
						m_targetActor->MakePrediction(actorInput, actorOutput);
						for (ndInt32 i = 0; i < actionDim; ++i)
						{
							ndReal noiseAction = actorOutput[i] + ndReal(m_randomGenerator.GetGaussianRandom(ndFloat32(0.0f), ndFloat32(m_actionNoiseVariance)));
							actorOutput[i] = ndClamp(noiseAction, ndReal(-1.0f), ndReal(1.0f));
						}

						ndReal criticInputBuffer[(statesDim + actionDim) * 2];
						ndDeepBrainMemVector criticInput(criticInputBuffer, statesDim + actionDim);
						for (ndInt32 i = 0; i < statesDim; ++i)
						{
							criticInput[i] = actorInput[i];
						}
						for (ndInt32 i = 0; i < actionDim; ++i)
						{
							criticInput[i + statesDim] = actorOutput[i];
						}
						m_targetCritic0->MakePrediction(criticInput, criticOutput);
						ndReal reward0 = criticOutput[0];

						m_targetCritic1->MakePrediction(criticInput, criticOutput);
						ndReal reward1 = criticOutput[0];

						// get minimum reward from the two target critics, and use that at the targte loss
						m_targetValue = transition.m_reward + m_gamma * ndMin(reward0, reward1);
					}
				}
				m_pass++;
				criticOutput[0] = m_targetValue;

				SetTruth(criticOutput);
				ndBrainLeastSquareErrorLoss::GetLoss(output, loss);
			}

			ndRandom& m_randomGenerator;
			ndBrainTrainer& m_criticTrainer0;
			ndBrainTrainer& m_criticTrainer1;
			ndBrain* m_targetActor;
			ndBrain* m_targetCritic0;
			ndBrain* m_targetCritic1;
			const ndBrainReplayBuffer<ndReal, statesDim, actionDim>& m_replayBuffer;
			ndFloat32 m_gamma;
			ndFloat32 m_targetValue;
			ndFloat32 m_actionNoiseVariance;
			ndInt32 m_index;
			ndInt32 m_pass;
		};

		ndBrainTrainer& trainer0 = *(*ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_criticOptimizer[threadIndex]);
		ndBrainTrainer& trainer1 = *(*m_critic2Optimizer[threadIndex]);
		trainer0.ClearGradientsAcc();
		trainer1.ClearGradientsAcc();

		ndReal inputBuffer[(statesDim + actionDim) * 2];
		ndDeepBrainMemVector input(inputBuffer, statesDim + actionDim);
		//Loss loss(trainer0, trainer1, &m_targetActor, &m_targetCritic, &m_target2Critic, m_replayBuffer, m_gamma, m_actionNoiseVariance, GetRandomGenerator(threadIndex));

		Loss loss(trainer0, trainer1,
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetTargetActor(),
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetTargetCritic(),
			&m_target2Critic,
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_replayBuffer,
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_gamma,
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_actionNoiseVariance,
			ndBrainThreadPool::GetRandomGenerator(threadIndex));
		
		const ndStartEnd startEnd(ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_bashBufferSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = ndInt32(shuffleBuffer[i]);
			const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_replayBuffer[index];

			for (ndInt32 j = 0; j < statesDim; ++j)
			{
				input[j] = transition.m_state[j];
			}
			for (ndInt32 j = 0; j < actionDim; ++j)
			{
				input[j + statesDim] = transition.m_action[j];
			}

			loss.m_pass = 0;
			loss.m_index = index;
			trainer0.BackPropagate(input, loss);
			trainer1.BackPropagate(input, loss);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		//ndBrainTrainer& trainer0 = *(*m_criticOptimizer[0]);
		ndBrainTrainer* const trainer0 = ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetCriticTrainer(0);
		ndBrainTrainer* const trainer1 = *m_critic2Optimizer[0];
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer* const srcTrainer0 = ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetCriticTrainer(i);
			ndBrainTrainer* const srcTrainer1 = *m_critic2Optimizer[i];
			trainer0->AcculumateGradients(*srcTrainer0, threadIndex, threadCount);
			trainer1->AcculumateGradients(*srcTrainer1, threadIndex, threadCount);
		}
	});

	ParallelExecute(PropagateBash);
	ParallelExecute(AccumulateWeight);
	ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetCriticTrainer(0)->UpdateWeights(ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_criticLearnRate, ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_bashBufferSize);
	m_critic2Optimizer[0]->UpdateWeights(ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_criticLearnRate, ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_bashBufferSize);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::BackPropagate()
{
	ndUnsigned32 shuffleBuffer[1024];
	for (ndInt32 i = 0; i < ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_bashBufferSize; ++i)
	{
		shuffleBuffer[i] = ndRandInt() % ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_replayBuffer.GetCount();
	}

	ndBrainAgentTD3_Trainer<statesDim, actionDim>::BackPropagateCritic(shuffleBuffer);
	if (m_frameCount & 1)
	{
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::BackPropagateActor(shuffleBuffer);
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetTargetActor()->SoftCopy(*ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetActor(), ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_softTargetFactor);
		m_target2Critic.SoftCopy(m_critic2, ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_softTargetFactor);
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetTargetCritic()->SoftCopy(*ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetCritic(), ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_softTargetFactor);
	}
}

#endif 
