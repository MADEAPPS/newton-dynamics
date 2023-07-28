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
	m_critic2.InitGaussianWeights();
	m_target2Critic.CopyFrom(m_critic2);

	for (ndInt32 i = 0; i < GetThreadCount(); ++i)
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
				Loss(ndBrainTrainer& trainer, ndBrain* const targetActor, ndBrain* const targetCritic, 
					const ndBrainReplayBuffer<ndReal, statesDim, actionDim>& replayBuffer, ndReal gamma)
				:ndBrainLeastSquareErrorLoss(trainer.GetBrain()->GetOutputSize())
				,m_targetActor(targetActor)
				,m_targetCritic(targetCritic)
				,m_criticTrainer(trainer)
				,m_replayBuffer(replayBuffer)
				,m_gamma(gamma)
				,m_index(0)
			{
			}

			void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(loss.GetCount() == 1);
				ndAssert(output.GetCount() == 1);
				ndAssert(m_truth.GetCount() == m_criticTrainer.GetBrain()->GetOutputSize());
				//const ndBrainReplayBuffer<ndReal, statesDim, actionDim>& replayBuffer = m_agent->GetReplayBuffer();
				const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_replayBuffer[m_index];

				ndReal criticOutputBuffer[2];
				ndDeepBrainMemVector criticOutput(criticOutputBuffer, 1);
				criticOutput[0] = transition.m_reward;
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
					m_targetCritic->MakePrediction(criticInput, criticOutput);
					criticOutput[0] = transition.m_reward + m_gamma * criticOutput[0];
				}

				SetTruth(criticOutput);
				ndBrainLeastSquareErrorLoss::GetLoss(output, loss);
			}

			ndBrain* m_targetActor;
			ndBrain* m_targetCritic;
			ndBrainTrainer& m_criticTrainer;
			const ndBrainReplayBuffer<ndReal, statesDim, actionDim>& m_replayBuffer;
			ndFloat32 m_gamma;
			ndInt32 m_index;
		};

		ndBrainTrainer& trainer0 = *(*m_criticOptimizer[threadIndex]);
		ndBrainTrainer& trainer1 = *(*m_critic2Optimizer[threadIndex]);
		trainer0.ClearGradientsAcc();
		trainer1.ClearGradientsAcc();

		ndReal inputBuffer[(statesDim + actionDim) * 2];
		ndDeepBrainMemVector input(inputBuffer, statesDim + actionDim);
		Loss loss0(trainer0, &m_targetActor, &m_targetCritic, m_replayBuffer, m_gamma);
		Loss loss1(trainer1, &m_targetActor, &m_target2Critic, m_replayBuffer, m_gamma);

		const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = ndInt32(shuffleBuffer[i]);
			const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_replayBuffer[index];

			for (ndInt32 j = 0; j < statesDim; ++j)
			{
				input[j] = transition.m_state[j];
			}
			for (ndInt32 j = 0; j < actionDim; ++j)
			{
				input[j + statesDim] = transition.m_action[j];
			}

			loss0.m_index = index;
			loss1.m_index = index;
			trainer0.BackPropagate(input, loss0);
			trainer1.BackPropagate(input, loss1);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer& trainer = *(*m_criticOptimizer[0]);
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer& srcTrainer0 = *(*m_criticOptimizer[i]);
			ndBrainTrainer& srcTrainer1 = *(*m_critic2Optimizer[i]);
			trainer.AcculumateGradients(srcTrainer0, threadIndex, threadCount);
			trainer.AcculumateGradients(srcTrainer1, threadIndex, threadCount);
		}
	});

	ParallelExecute(PropagateBash);
	ParallelExecute(AccumulateWeight);
	m_criticOptimizer[0]->UpdateWeights(m_criticLearnRate, m_bashBufferSize);
}


template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentTD3_Trainer<statesDim, actionDim>::BackPropagate()
{
	ndUnsigned32 shuffleBuffer[1024];
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		shuffleBuffer[i] = ndRandInt() % m_replayBuffer.GetCount();
	}

	BackPropagateCritic(shuffleBuffer);
	BackPropagateActor(shuffleBuffer);
	m_targetActor.SoftCopy(*(*m_actor), m_softTargetFactor);
	m_targetCritic.SoftCopy(*(*m_critic), m_softTargetFactor);
}


#endif 
