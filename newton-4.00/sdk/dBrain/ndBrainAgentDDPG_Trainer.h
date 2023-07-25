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

#ifndef _ND_BRAIN_AGENT_DDPG_TRAINER_H__
#define _ND_BRAIN_AGENT_DDPG_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"

// this is an implementation of the vanilla 
// Continuous control with deep re enforcement learning (ddpg agent)
// trainer as described in: https://arxiv.org/pdf/1509.02971.pdf

// default hyper parameters defaults
#define D_DDPG_CRITIC_LEARN_RATE		ndReal(5.0e-3f)
#define D_DDPG_ACTOR_LEARN_RATE			(D_DDPG_CRITIC_LEARN_RATE * ndReal(0.125f))
#define D_DDPG_DISCOUNT_FACTOR			ndReal (0.99f)
//#define D_DDPG_REPLAY_BUFFERSIZE		(1024 * 512)
#define D_DDPG_REPLAY_BUFFERSIZE		(1024)
#define D_DDPG_REPLAY_BASH_SIZE			32
#define D_DDPG_REGULARIZER				ndReal (2.0e-6f)
#define D_DDPG_SOFT_TARGET_FACTOR		ndReal (1.0e-3f)
//#define D_DDPG_ACTION_NOISE_DEVIATION	ndReal (0.01f)
#define D_DDPG_ACTION_NOISE_DEVIATION	ndReal (0.0f)

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDDPG_Trainer: public ndBrainAgent, public ndBrainThreadPool
{
	public:
	ndBrainAgentDDPG_Trainer(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic);
	virtual ~ndBrainAgentDDPG_Trainer();

	ndReal GetCurrentValue() const;
	ndInt32 GetFramesCount() const;
	ndInt32 GetEposideCount() const;
	ndInt32 GetEpisodeFrames() const;

	protected:
	void Step();
	void OptimizeStep();
	void Save(ndBrainSave* const loadSave) const;

	bool IsSampling() const;
	bool IsTerminal() const;
	ndReal GetReward() const;
	ndReal PerturbeAction(ndReal action) const;

	private:
	void Optimize();
	void BackPropagate();
	void PopulateReplayBuffer();
	void SetBufferSize(ndInt32 size);
	void BackPropagateActor(const ndUnsigned32* const bashIndex);
	void BackPropagateCritic(const ndUnsigned32* const bashIndex);

	ndSharedPtr<ndBrain> m_actor;
	ndSharedPtr<ndBrain> m_critic;
	ndBrain m_targetActor;
	ndBrain m_targetCritic;
	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_actorOptimizer;
	ndFixSizeArray<ndSharedPtr<ndBrainTrainer>, D_MAX_THREADS_COUNT> m_criticOptimizer;

	ndBrainVector m_state;
	ndBrainVector m_actions;
	ndArray<ndInt32> m_bashSamples;
	ndBrainReplayBuffer<ndReal, statesDim, actionDim> m_replayBuffer;
	ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim> m_currentTransition;

	ndReal m_gamma;
	ndReal m_currentQValue;
	ndReal m_actorLearnRate;
	ndReal m_criticLearnRate;
	ndReal m_softTargetFactor;
	ndReal m_actionNoiseDeviation;
	ndInt32 m_frameCount;
	ndInt32 m_framesAlive;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	bool m_collectingSamples;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDDPG_Trainer<statesDim, actionDim>::ndBrainAgentDDPG_Trainer(const ndSharedPtr<ndBrain>& actor, const ndSharedPtr<ndBrain>& critic)
	:ndBrainAgent()
	,m_actor(actor)
	,m_critic(critic)
	,m_targetActor(*(*m_actor))
	,m_targetCritic(*(*m_critic))
	,m_replayBuffer()
	,m_gamma(D_DDPG_DISCOUNT_FACTOR)
	,m_currentQValue(ndReal(0.0f))
	,m_actorLearnRate(D_DDPG_ACTOR_LEARN_RATE)
	,m_criticLearnRate(D_DDPG_CRITIC_LEARN_RATE)
	,m_softTargetFactor(D_DDPG_SOFT_TARGET_FACTOR)
	,m_actionNoiseDeviation(D_DDPG_ACTION_NOISE_DEVIATION)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(D_DDPG_REPLAY_BASH_SIZE)
	,m_collectingSamples(true)
{
	ndAssert(m_critic->GetOutputSize() == 1);
	ndAssert(((*(*m_actor))[m_actor->GetCount() - 1])->GetActivationType() == m_tanh);
	ndAssert(m_critic->GetInputSize() == (m_actor->GetInputSize() + m_actor->GetOutputSize()));

	ndInt32 threadCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize / 4);
threadCount = 1;
	SetThreadCount(threadCount);
	for (ndInt32 i = 0; i < GetThreadCount(); ++i)
	{
		m_actorOptimizer.PushBack(new ndBrainTrainer(*m_actor));
		m_criticOptimizer.PushBack(new ndBrainTrainer(*m_critic));
		m_actorOptimizer[m_actorOptimizer.GetCount() - 1]->SetRegularizer(D_DQN_REGULARIZER);
		m_criticOptimizer[m_criticOptimizer.GetCount() - 1]->SetRegularizer(D_DQN_REGULARIZER);
	}

	m_state.SetCount(statesDim);
	m_actions.SetCount(actionDim);
	m_state.Set(ndReal(0.0f));
	m_actions.Set(ndReal(0.0f));

	SetBufferSize(D_DDPG_REPLAY_BUFFERSIZE);
	m_targetActor.CopyFrom(*(*m_actor));
	m_targetCritic.CopyFrom(*(*m_critic));
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDDPG_Trainer<statesDim, actionDim>::~ndBrainAgentDDPG_Trainer()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetCurrentValue() const
{
	return m_currentQValue;
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDDPG_Trainer<statesDim, actionDim>::IsSampling() const
{
	return m_collectingSamples;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetFramesCount() const
{
	return m_frameCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetEposideCount() const
{
	return m_eposideCount;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetEpisodeFrames() const
{
	return m_framesAlive;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::SetBufferSize(ndInt32 size)
{
	m_replayBuffer.SetSize(size);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::BackPropagateCritic(const ndUnsigned32* const shuffleBuffer)
{
	auto PropagateBash = ndMakeObject::ndFunction([this, &shuffleBuffer](ndInt32 threadIndex, ndInt32 threadCount)
	{
		class Loss : public ndBrainLeastSquareErrorLoss
		{
			public:
			Loss(ndBrainTrainer& trainer, ndBrainAgentDDPG_Trainer<statesDim, actionDim>* const agent)
				:ndBrainLeastSquareErrorLoss(trainer.GetBrain()->GetOutputSize())
				,m_criticTrainer(trainer)
				,m_agent(agent)
				,m_index(0)
			{
			}

			void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(loss.GetCount() == 1);
				ndAssert(output.GetCount() == 1);
				ndAssert(m_truth.GetCount() == m_criticTrainer.GetBrain()->GetOutputSize());
				const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_agent->m_replayBuffer[m_index];

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
					m_agent->m_actor->MakePrediction(actorInput, actorOutput);

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
					m_agent->m_targetCritic.MakePrediction(criticInput, criticOutput);
					criticOutput[0] = transition.m_reward + m_agent->m_gamma * criticOutput[0];
				}

				SetTruth(criticOutput);
				ndBrainLeastSquareErrorLoss::GetLoss(output, loss);
			}

			ndBrainTrainer& m_criticTrainer;
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>* m_agent;
			ndInt32 m_index;
		};

		ndBrainTrainer& trainer = *(*m_criticOptimizer[threadIndex]);
		trainer.ClearGradientsAcc();

		Loss loss(trainer, this);
		ndReal inputBuffer[(statesDim + actionDim) * 2];
		ndDeepBrainMemVector input(inputBuffer, statesDim + actionDim);

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

			loss.m_index = index;
			trainer.BackPropagate(input, loss);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer& trainer = *(*m_criticOptimizer[0]);
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer& srcTrainer = *(*m_criticOptimizer[i]);
			trainer.AcculumateGradients(srcTrainer, threadIndex, threadCount);
		}
	});

	ParallelExecute(PropagateBash);
	ParallelExecute(AccumulateWeight);
	m_criticOptimizer[0]->UpdateWeights(m_criticLearnRate, m_bashBufferSize);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::BackPropagateActor(const ndUnsigned32* const shuffleBuffer)
{
	auto PropagateBash = ndMakeObject::ndFunction([this, &shuffleBuffer](ndInt32 threadIndex, ndInt32 threadCount)
	{
		class Loss: public ndBrainLoss
		{
			public:
			Loss(ndBrainTrainer& trainer, ndBrainAgentDDPG_Trainer<statesDim, actionDim>* const agent)
				:ndBrainLoss()
				,m_actorTrainer(trainer)
				,m_agent(agent)
				,m_index(0)
			{
			}

			void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
			{
				ndAssert(loss.GetCount() == actionDim);
				ndAssert(output.GetCount() == actionDim);
				const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_agent->m_replayBuffer[m_index];

				ndReal criticInputBuffer[(statesDim + actionDim) * 2];
				ndDeepBrainMemVector input(criticInputBuffer, statesDim + actionDim);
				for (ndInt32 i = 0; i < statesDim; ++i)
				{
					input[i] = transition.m_state[i];
				}
				for (ndInt32 i = 0; i < actionDim; ++i)
				{
					input[i + statesDim] = output[i];
				}
				m_agent->m_critic->CalculateInputGradients(input, input);

				for (ndInt32 i = 0; i < actionDim; ++i)
				{
					loss[i] = input[statesDim + i];
				}
			}

			ndBrainTrainer& m_actorTrainer;
			ndBrainAgentDDPG_Trainer<statesDim, actionDim>* m_agent;
			ndInt32 m_index;
		};

		ndBrainTrainer& trainer = *(*m_actorOptimizer[threadIndex]);
		trainer.ClearGradientsAcc();

		Loss loss(trainer, this);
		ndReal inputBuffer[statesDim * 2];
		ndDeepBrainMemVector input(inputBuffer, statesDim);

		const ndStartEnd startEnd(m_bashBufferSize, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = ndInt32(shuffleBuffer[i]);
			const ndBrainReplayTransitionMemory<ndReal, statesDim, actionDim>& transition = m_replayBuffer[index];

			for (ndInt32 j = 0; j < statesDim; ++j)
			{
				input[j] = transition.m_state[j];
			}
			loss.m_index = index;
			trainer.BackPropagate(input, loss);
		}
	});

	auto AccumulateWeight = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		ndBrainTrainer& trainer = *(*m_actorOptimizer[0]);
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			ndBrainTrainer& srcTrainer = *(*m_actorOptimizer[i]);
			trainer.AcculumateGradients(srcTrainer, threadIndex, threadCount);
		}
	});

	ParallelExecute(PropagateBash);
	ParallelExecute(AccumulateWeight);
	m_actorOptimizer[0]->UpdateWeights(-m_actorLearnRate, m_bashBufferSize);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::BackPropagate()
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

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::Save(ndBrainSave* const loadSave) const
{
	loadSave->Save(*m_actor);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDDPG_Trainer<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDDPG_Trainer<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndReal(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDDPG_Trainer<statesDim, actionDim>::PerturbeAction(ndReal action) const
{
	ndReal actionNoise = ndReal(ndGaussianRandom(ndFloat32(action), ndFloat32(m_actionNoiseDeviation)));
	return ndClamp(actionNoise, ndReal(-1.0f), ndReal(1.0f));
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::PopulateReplayBuffer()
{
	GetObservation(&m_currentTransition.m_nextState[0]);
	m_currentTransition.m_reward = GetReward();
	m_currentTransition.m_terminalState = IsTerminal();
	m_replayBuffer.AddTransition(m_currentTransition);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::Optimize()
{
	BackPropagate();
	if (IsSampling())
	{
		ndExpandTraceMessage("%d star training: episode %d\n", m_frameCount, m_eposideCount);
	}
	m_collectingSamples = false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::Step()
{
	GetObservation(&m_currentTransition.m_state[0]);
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_state[i] = m_currentTransition.m_state[i];
	}
	m_actor->MakePrediction(m_state, m_actions);

	// explore environment
	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		m_actions[i] = PerturbeAction(m_actions[i]);
	}

	ApplyActions(&m_actions[0]);
	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		m_currentTransition.m_action[i] = m_actions[i];
	}

	// Get Q vale from Critic
	ndReal buffer[256];
	ndDeepBrainMemVector criticInput(buffer, statesDim + actionDim);;
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		criticInput[i] = m_state[i];
	}
	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		criticInput[i + statesDim] = m_actions[i];
	}
	ndDeepBrainMemVector criticOutput(&m_currentQValue, 1);
	m_critic->MakePrediction(criticInput, criticOutput);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG_Trainer<statesDim, actionDim>::OptimizeStep()
{
	if (!m_frameCount)
	{
		ResetModel();
	}

	PopulateReplayBuffer();
	if (m_replayBuffer.GetCount() == m_replayBuffer.GetCapacity())
	{
		Optimize();
	}

	if (m_currentTransition.m_terminalState)
	{
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_state.Set(ndReal(0.0f));
		ndBrainAgentDDPG_Trainer<statesDim, actionDim>::m_actions.Set(ndReal(0.0f));
		ResetModel();
		m_currentTransition.Clear();
		if (IsSampling() && (m_eposideCount % 100 == 0))
		{
			ndExpandTraceMessage("collecting samples: frame %d out of %d, episode %d \n", m_frameCount, m_replayBuffer.GetCapacity(), m_eposideCount);
		}
		m_eposideCount++;
		m_framesAlive = 0;
	}

	m_frameCount++;
	m_framesAlive++;
}

#endif 
