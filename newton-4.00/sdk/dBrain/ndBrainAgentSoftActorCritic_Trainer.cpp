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
#include "ndBrainLayerLinear.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainAgentSoftActorCritic_Trainer.h"

#if 0
ndBrainAgentSoftActorCritic_Trainer::HyperParameters::HyperParameters()
{
	m_actionNoiseSigma = ndBrainFloat(0.01f);
	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = ndBrainFloat(1.0e-4f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_discountRewardFactor = ndBrainFloat(0.99f);
	m_policyMovingAverageFactor = ndBrainFloat(0.005f);
	m_criticMovingAverageFactor = ndBrainFloat(0.005f);

	m_policyRegularizerType = ndBrainOptimizer::m_ridge;
	m_criticRegularizerType = ndBrainOptimizer::m_ridge;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_actorHiddenLayers = 3;
	m_policyUpdatesCount = 16;
	m_criticUpdatesCount = 16;
	m_maxTrajectorySteps = 1024 * 4;
	m_hiddenLayersNumberOfNeurons = 64;
	m_replayBufferStartOptimizeSize = m_maxTrajectorySteps * 4;

	m_replayBufferSize = 1024 * 1024;
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);

//m_threadsCount = 1;
//m_replayBufferSize /= 10;
}

ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::ndTrajectoryTransition(ndInt32 actionsSize, ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_actionsSize(actionsSize)
	,m_obsevationsSize(obsevationsSize)
{
}

void ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	ndMemSet(&me[stride * entry], ndBrainFloat(0.0f), stride);
}

void ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry)
{
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	ndTrajectoryTransition& me = *this;
	ndMemCpy(&me[stride * entry], &src[stride * srcEntry], stride);
}

ndInt32 ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetCount() const
{
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return me[stride * entry + m_reward];
}

void ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	me[stride * entry + m_reward] = reward;
}

bool ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return (me[stride * entry + m_isterminalState] == ndBrainFloat(999.0f)) ? true : false;
}

void ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	me[stride * entry + m_isterminalState] = isTernimal ? ndBrainFloat(999.0f) : ndBrainFloat(-999.0f);
}

ndBrainFloat* ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize];
}

ndBrainFloat* ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize + m_actionsSize];
}

const ndBrainFloat* ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize + m_actionsSize];
}

ndBrainFloat* ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize + m_actionsSize + m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentSoftActorCritic_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize + m_actionsSize + m_obsevationsSize];
}

ndBrainAgentSoftActorCritic_Agent::ndBrainAgentSoftActorCritic_Agent(const ndSharedPtr<ndBrainAgentSoftActorCritic_Trainer>& master)
	:ndBrainAgent()
	,m_owner(master)
	,m_trajectory(m_owner->m_parameters.m_numberOfActions, m_owner->m_parameters.m_numberOfObservations)
	,m_workingBuffer()
	,m_trajectoryBaseCount(0)
{
	m_owner->m_agent = this;
	//m_trajectory.SetCount(0);
	//m_randomGenerator = m_master->GetRandomGenerator();
}

ndBrainAgentSoftActorCritic_Agent::~ndBrainAgentSoftActorCritic_Agent()
{
}

ndInt32 ndBrainAgentSoftActorCritic_Agent::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

ndReal ndBrainAgentSoftActorCritic_Agent::SampleAction(ndReal action) const
{
	ndReal actionNoise = ndReal(ndGaussianRandom(ndFloat32(action), m_owner->m_parameters.m_actionNoiseSigma));
	return ndClamp(actionNoise, ndReal(-1.0f), ndReal(1.0f));
}

void ndBrainAgentSoftActorCritic_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), m_owner->m_parameters.m_numberOfActions);
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), m_owner->m_parameters.m_numberOfObservations);
	GetObservation(&observation[0]);
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());

	m_owner->m_policy.MakePrediction(observation, actions, m_workingBuffer);
	// explore environment
	for (ndInt32 i = m_owner->m_parameters.m_numberOfActions - 1; i >= 0; --i)
	{
		actions[i] = SampleAction(actions[i]);
	}
	ApplyActions(&actions[0]);
}

ndBrainAgentSoftActorCritic_Trainer::ndBrainAgentSoftActorCritic_Trainer(const HyperParameters& parameters)
	:ndBrainThreadPool()
	,m_name()
	,m_parameters(parameters)
	,m_policy()
	,m_referencePolicy()
	,m_policyTrainers()
	,m_policyOptimizer()
	,m_expectedRewards()
	,m_expectedRewardsIndexBuffer()
	,m_replayBuffer(m_parameters.m_numberOfActions, m_parameters.m_numberOfObservations)
	,m_agent(nullptr)
	,m_shuffleBuffer()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_replayBufferIndex(0)
	,ndPolycyDelayMod(0)
	,m_shuffleIndexBuffer(0)
	,m_startOptimization(false)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);
	//ndSetRandSeed(m_randomSeed);

	// create actor class
	SetThreadCount(m_parameters.m_threadsCount);

	BuildPolicyClass();
	BuildCriticClass();
}

ndBrainAgentSoftActorCritic_Trainer::~ndBrainAgentSoftActorCritic_Trainer()
{
	for (ndInt32 i = 0; i < m_policyTrainers.GetCount(); ++i)
	{
		delete m_policyTrainers[i];
	}

	for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
	{
		for (ndInt32 i = 0; i < m_criticTrainers[k].GetCount(); ++i)
		{
			delete m_criticTrainers[k][i];
		}
	}
}

ndBrain* ndBrainAgentSoftActorCritic_Trainer::GetPolicyNetwork()
{
	return &m_policy;
}

const ndString& ndBrainAgentSoftActorCritic_Trainer::GetName() const
{
	return m_name;
}

void ndBrainAgentSoftActorCritic_Trainer::SetName(const ndString& name)
{
	m_name = name;
}

void ndBrainAgentSoftActorCritic_Trainer::BuildPolicyClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < m_parameters.m_actorHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_policy.AddLayer(layers[i]);
	}
	m_policy.InitWeights();

	m_policyOptimizer = ndSharedPtr<ndBrainOptimizerAdam> (new ndBrainOptimizerAdam());
	m_policyOptimizer->SetRegularizer(m_parameters.m_policyRegularizer);
	m_policyOptimizer->SetRegularizerType(m_parameters.m_policyRegularizerType);

	m_policyTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_policy);
		m_policyTrainers.PushBack(trainer);
	}
	m_referencePolicy = m_policy;
}

void ndBrainAgentSoftActorCritic_Trainer::BuildCriticClass()
{
	for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
	{
		ndFixSizeArray<ndBrainLayer*, 32> layers;

		layers.SetCount(0);
		layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations + m_parameters.m_numberOfActions, m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
		for (ndInt32 i = 0; i < m_parameters.m_actorHiddenLayers; ++i)
		{
			ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
		}
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));

		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			m_critic[k].AddLayer(layers[i]);
		}
		m_critic[k].InitWeights();

		m_criticOptimizer[k] = ndSharedPtr<ndBrainOptimizerAdam>(new ndBrainOptimizerAdam());
		m_criticOptimizer[k]->SetRegularizer(m_parameters.m_criticRegularizer);
		m_criticOptimizer[k]->SetRegularizerType(m_parameters.m_criticRegularizerType);

		m_criticTrainers[k].SetCount(0);
		for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
		{
			ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic[k]);
			m_criticTrainers[k].PushBack(trainer);
		}
		m_referenceCritic[k] = m_critic[k];
	}
}

bool ndBrainAgentSoftActorCritic_Trainer::IsSampling() const
{
	return !m_startOptimization;
}

ndUnsigned32 ndBrainAgentSoftActorCritic_Trainer::GetFramesCount() const
{
	return m_frameCount;
}

ndUnsigned32 ndBrainAgentSoftActorCritic_Trainer::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentSoftActorCritic_Trainer::GetAverageScore() const
{
	return m_averageExpectedRewards.GetAverage();
}

ndFloat32 ndBrainAgentSoftActorCritic_Trainer::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::SaveTrajectory()
{
	// remove all dead states except the last 
	while (m_agent->m_trajectory.GetTerminalState(m_agent->m_trajectory.GetCount() - 2))
	{
		m_agent->m_trajectory.SetCount(m_agent->m_trajectory.GetCount() - 1);
	}

	auto AddTransition = [this](ndInt32 dstIndex, ndInt32 srcIndex)
	{
		m_shuffleBuffer.PushBack(dstIndex);
		m_replayBuffer.SetCount(dstIndex + 1);
		m_replayBuffer.CopyFrom(dstIndex, m_agent->m_trajectory, srcIndex);

		ndBrainFloat reward = m_agent->m_trajectory.GetReward(srcIndex + 1);
		m_replayBuffer.SetReward(dstIndex, reward);

		bool terminalState = m_agent->m_trajectory.GetTerminalState(srcIndex + 1);
		m_replayBuffer.SetTerminalState(dstIndex, terminalState);

		ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(dstIndex), m_parameters.m_numberOfObservations);
		nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(srcIndex + 1), m_parameters.m_numberOfObservations));
	};

	ndInt32 replayBufferCount = m_replayBuffer.GetCount();
	ndInt32 numberOfNewTranstitions = m_agent->m_trajectory.GetCount() - m_agent->m_trajectoryBaseCount;
	if ((replayBufferCount + numberOfNewTranstitions) < m_parameters.m_replayBufferStartOptimizeSize)
	{
		const ndInt32 numOfTransitions = m_agent->m_trajectory.GetCount() - 1;
		for (ndInt32 i = m_agent->m_trajectoryBaseCount; i < numOfTransitions; ++i)
		{
			AddTransition(replayBufferCount, i);
			replayBufferCount++;
		}
		m_agent->m_trajectoryBaseCount = numOfTransitions;
	}
	else if (!m_startOptimization)
	{
		m_startOptimization = true;
	}
	else if ((replayBufferCount + numberOfNewTranstitions) < m_parameters.m_replayBufferSize)
	{
		const ndInt32 numOfTransitions = m_agent->m_trajectory.GetCount() - 1;
		for (ndInt32 i = m_agent->m_trajectoryBaseCount; i < numOfTransitions; ++i)
		{
			AddTransition(replayBufferCount, i);
			replayBufferCount++;
		}
		m_agent->m_trajectoryBaseCount = numOfTransitions;
	}
	else if (replayBufferCount < m_parameters.m_replayBufferSize)
	{
		const ndInt32 numOfTransitions = m_parameters.m_replayBufferSize - replayBufferCount;
		for (ndInt32 i = 0; i < numOfTransitions; ++i)
		{
			AddTransition(replayBufferCount, m_agent->m_trajectoryBaseCount + i);
		}
		m_agent->m_trajectoryBaseCount += numOfTransitions;
	}
	else
	{
		const ndInt32 numOfTransitions = m_agent->m_trajectory.GetCount() - 1;
		for (ndInt32 i = m_agent->m_trajectoryBaseCount; i < numOfTransitions; ++i)
		{
			m_replayBuffer.CopyFrom(m_replayBufferIndex, m_agent->m_trajectory, i);
			
			ndBrainFloat reward = m_agent->m_trajectory.GetReward(i + 1);
			m_replayBuffer.SetReward(m_replayBufferIndex, reward);
			
			bool terminalState = m_agent->m_trajectory.GetTerminalState(i + 1);
			m_replayBuffer.SetTerminalState(m_replayBufferIndex, terminalState);
			
			ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(m_replayBufferIndex), m_parameters.m_numberOfObservations);
			nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations));

			m_replayBufferIndex = (m_replayBufferIndex + 1) % m_parameters.m_replayBufferSize;
		}
		m_agent->m_trajectoryBaseCount = numOfTransitions;
	}

	bool terminalState = m_agent->m_trajectory.GetTerminalState(m_agent->m_trajectory.GetCount() - 1);
	if (terminalState || (m_agent->m_trajectoryBaseCount >= m_parameters.m_maxTrajectorySteps))
	{
		if (m_startOptimization)
		{
			m_eposideCount++;
			m_framesAlive = 0;
			m_averageFramesPerEpisodes.Update(ndFloat32 (m_agent->m_trajectoryBaseCount));
		}
		m_agent->ResetModel();
		m_agent->m_trajectory.SetCount(0);
		m_agent->m_trajectoryBaseCount = 0;
		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	}
}

#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::LearnQvalueFunction(ndInt32 criticIndex)
{
	ndInt32 base = 0;
	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 n = m_parameters.m_criticUpdatesCount - 1; n >= 0; --n)
	{
		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base, criticIndex](ndInt32, ndInt32)
		{
			ndBrainFixSizeVector<256> criticObservationAction;
			criticObservationAction.SetCount(m_parameters.m_numberOfActions + m_parameters.m_numberOfObservations);

			ndBrainLossLeastSquaredError loss(1);
			ndBrainFixSizeVector<1> groundTruth;
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				const ndInt32 index = m_expectedRewardsIndexBuffer[i + base];
				ndBrainTrainer& trainer = *m_criticTrainers[criticIndex][i];

				ndMemCpy(&criticObservationAction[0], m_replayBuffer.GetActions(index), m_parameters.m_numberOfActions);
				ndMemCpy(&criticObservationAction[m_parameters.m_numberOfActions], m_replayBuffer.GetObservations(index), m_parameters.m_numberOfObservations);

				groundTruth[0] = m_expectedRewards[i + base];
				loss.SetTruth(groundTruth);
				trainer.BackPropagate(criticObservationAction, loss);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
		m_criticOptimizer[criticIndex]->Update(this, m_criticTrainers[criticIndex], m_parameters.m_criticLearnRate);
		base += m_parameters.m_miniBatchSize;
	}
	m_referenceCritic[criticIndex].SoftCopy(m_critic[criticIndex], m_parameters.m_criticMovingAverageFactor);
}

#pragma optimize( "", off )
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
		ndBrainFixSizeVector<256> criticObservationAction;
		criticObservationAction.SetCount(m_parameters.m_numberOfActions + m_parameters.m_numberOfObservations);
		for (ndInt32 i = iterator.fetch_add(32); i < count; i = iterator.fetch_add(32))
		{
			const ndInt32 size = (i + 32) < count ? i + 32 : count;
			for (ndInt32 j = i; j < size; ++j)
			{
				ndBrainFixSizeVector<ND_NUMBER_OF_CRITICS> rewards;
				rewards.SetCount(0);
				const ndInt32 index = m_expectedRewardsIndexBuffer[j];
				for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
				{
					rewards.PushBack(m_replayBuffer.GetReward(index));
				}
				if (!m_replayBuffer.GetTerminalState(index))
				{
					ndBrainMemVector nextAction(&criticObservationAction[0], m_parameters.m_numberOfActions);
					const ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(index), m_parameters.m_numberOfObservations);
					m_referencePolicy.MakePrediction(nextObservation, nextAction);
					ndMemCpy(&criticObservationAction[m_parameters.m_numberOfActions], &nextObservation[0], nextObservation.GetCount());

					ndBrainFixSizeVector<1> criticQvalue;
					for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
					{
						m_referenceCritic[k].MakePrediction(criticObservationAction, criticQvalue);
						rewards[k] += m_parameters.m_discountRewardFactor * criticQvalue[0];
					}
				}

				ndFloat32 minQ = ndFloat32(1.0e10f);
				for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
				{
					minQ = ndMin(minQ, rewards[k]);
				}
				m_expectedRewards[j] = minQ;
			}
		}
	});
	ndBrainThreadPool::ParallelExecute(ExpectedRewards);

	ndFloat32 rewardSum = 0.0f;
	for (ndInt32 i = ndInt32 (m_expectedRewards.GetCount()) - 1; i >= 0; --i)
	{
		rewardSum += m_expectedRewards[i];
	}
	ndFloat32 averageReward = rewardSum / ndFloat32(m_expectedRewards.GetCount());
	m_averageExpectedRewards.Update(averageReward);
}


#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::LearnPolicyFunction()
{
	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 n = m_parameters.m_policyUpdatesCount - 1; n >= 0; --n)
	{
		ndFixSizeArray<ndInt32, 1024> indirectBuffer;
		for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
		{
			indirectBuffer.PushBack(m_shuffleBuffer[m_shuffleIndexBuffer]);
			m_shuffleIndexBuffer = (m_shuffleIndexBuffer + 1) % ndInt32(m_shuffleBuffer.GetCount());
		}
		
		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, &indirectBuffer](ndInt32 threadIndex, ndInt32)
		{
			class ndLoss : public ndBrainLossLeastSquaredError
			{
				public:
				ndLoss(ndBrainAgentSoftActorCritic_Trainer* const owner, const ndInt32 index, ndInt32 m_threadIndex)
					:ndBrainLossLeastSquaredError(1)
					,m_criticLoss()
					,m_owner(owner)
					,m_combinedActionObservation()
					,m_index(index)
					,m_threadIndex(m_threadIndex)
				{
					m_combinedInputGradients.SetCount(m_owner->m_parameters.m_numberOfObservations + m_owner->m_parameters.m_numberOfActions);
					m_combinedActionObservation.SetCount(m_owner->m_parameters.m_numberOfObservations + m_owner->m_parameters.m_numberOfActions);
				}

				class ndCriticLoss: public ndBrainLossLeastSquaredError
				{
					public:
					ndCriticLoss()
						:ndBrainLossLeastSquaredError(1)
					{
					}

					void GetLoss(const ndBrainVector&, ndBrainVector& loss)
					{
						loss[0] = -1.0f;
					}
				};

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					ndMemCpy(&m_combinedActionObservation[0], &output[0], m_owner->m_parameters.m_numberOfActions);
					ndMemCpy(&m_combinedActionObservation[m_owner->m_parameters.m_numberOfActions], m_owner->m_replayBuffer.GetNextObservations(m_index), m_owner->m_parameters.m_numberOfObservations);
					m_owner->m_referenceCritic[0].CalculateInputGradient(m_combinedActionObservation, m_combinedInputGradients, m_criticLoss);
					
					//ndBrainTrainer& trainer = *m_owner->m_criticTrainers[m_threadIndex];
					//trainer.CalculateInputGradient(m_combinedActionObservation, m_combinedInputGradients, m_criticLoss);
					ndMemCpy(&loss[0], &m_combinedInputGradients[0], loss.GetCount());
				}

				ndCriticLoss m_criticLoss;
				ndBrainAgentSoftActorCritic_Trainer* m_owner;
				ndBrainFixSizeVector<256> m_combinedInputGradients;
				ndBrainFixSizeVector<256> m_combinedActionObservation;
				ndInt32 m_index;
				ndInt32 m_threadIndex;
			};

			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				const ndInt32 index = indirectBuffer[i];
				ndBrainTrainer& trainer = *m_policyTrainers[i];
				const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), m_parameters.m_numberOfObservations);

				ndLoss loss(this, index, threadIndex);
				trainer.BackPropagate(observation, loss);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
		m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
	}
	m_referencePolicy.SoftCopy(m_policy, m_parameters.m_policyMovingAverageFactor);
}

#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::Optimize()
{
	CalculateExpectedRewards();
	for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
	{
		LearnQvalueFunction(k);
	}

	if (!ndPolycyDelayMod)
	{
		LearnPolicyFunction();
	}
	ndPolycyDelayMod = (ndPolycyDelayMod + 1) % ND_POLICY_DELAY_MOD;
}

#pragma optimize( "", off )
void ndBrainAgentSoftActorCritic_Trainer::OptimizeStep()
{
	SaveTrajectory();
	if (m_startOptimization)
	{
		Optimize();
	}
	m_frameCount++;
	m_framesAlive++;
}

#endif


ndBrainAgentSoftActorCritic_Agent::ndBrainAgentSoftActorCritic_Agent(const HyperParameters& parameters)
	:ndBrainAgentDeterministicPolicyGradient_Trainer(parameters)
{
}

ndBrainAgentSoftActorCritic_Agent::~ndBrainAgentSoftActorCritic_Agent()
{
}
