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
#include "ndBrainAgentDDPG_Trainer.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"

ndBrainAgentDDPG_Trainer::HyperParameters::HyperParameters()
{
	m_discountRewardFactor = ndBrainFloat(0.99f);
	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = ndBrainFloat(1.0e-4f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_policyMovingAverageFactor = ndBrainFloat(0.005f);
	m_criticMovingAverageFactor = ndBrainFloat(0.005f);

	m_policyRegularizerType = ndBrainOptimizer::m_ridge;
	m_criticRegularizerType = ndBrainOptimizer::m_ridge;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_actorHiddenLayers = 3;
	m_criticUpdatesCount = 1;
	m_policyUpdatesCount = 1;
	m_hiddenLayersNumberOfNeurons = 64;
	m_replayBufferStartOptimizeSize = 1024 * 32;

	m_replayBufferSize = 1024 * 1024;
	m_maxTrajectorySteps = 1024 * 32;

//m_replayBufferSize = 128 * 4 + 100;
//m_replayBufferStartOptimizeSize = 128 * 3;

	//m_threadsCount = 1;
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);
}

ndBrainAgentDDPG_Agent::ndTrajectoryTransition::ndTrajectoryTransition(ndInt32 actionsSize, ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_actionsSize(actionsSize)
	,m_obsevationsSize(obsevationsSize)
{
}

void ndBrainAgentDDPG_Agent::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	ndMemSet(&me[stride * entry], ndBrainFloat(0.0f), stride);
}

void ndBrainAgentDDPG_Agent::ndTrajectoryTransition::CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry)
{
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	ndTrajectoryTransition& me = *this;
	ndMemCpy(&me[stride * entry], &src[stride * srcEntry], stride);
}

ndInt32 ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetCount() const
{
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentDDPG_Agent::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return me[stride * entry + m_reward];
}

bool ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return (me[stride * entry + m_isterminalState] == ndBrainFloat(999.0f)) ? true : false;
}

void ndBrainAgentDDPG_Agent::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	me[stride * entry + m_isterminalState] = isTernimal ? ndBrainFloat(999.0f) : ndBrainFloat(-999.0f);
}

void ndBrainAgentDDPG_Agent::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	me[stride * entry + m_reward] = reward;
}

ndBrainFloat* ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_obsevationsSize + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_obsevationsSize + m_transitionSize];
}

ndBrainFloat* ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize];
}

ndBrainFloat* ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize + m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentDDPG_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
	return &me[stride * entry + m_transitionSize + m_obsevationsSize];
}

ndBrainAgentDDPG_Agent::ndBrainAgentDDPG_Agent(const ndSharedPtr<ndBrainAgentDDPG_Trainer>& master)
	:ndBrainAgent()
	,m_owner(master)
	,m_trajectory(m_owner->m_parameters.m_numberOfActions, m_owner->m_parameters.m_numberOfObservations)
	,m_workingBuffer()
{
	//m_owner->m_agents.Append(this);
	//m_trajectory.SetCount(0);
	//m_randomGenerator = m_master->GetRandomGenerator();

	m_owner->m_agent = this;
}

ndBrainAgentDDPG_Agent::~ndBrainAgentDDPG_Agent()
{
}

ndInt32 ndBrainAgentDDPG_Agent::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

ndReal ndBrainAgentDDPG_Agent::PerturbeAction(ndReal action) const
{
	//ndReal actionNoise = ndReal(ndGaussianRandom(ndFloat32(action), ndFloat32(m_actionNoiseVariance)));
	ndReal actionNoise = ndReal(ndGaussianRandom(ndFloat32(action), ndFloat32(0.01f)));
	return ndClamp(actionNoise, ndReal(-1.0f), ndReal(1.0f));
}

void ndBrainAgentDDPG_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), m_owner->m_parameters.m_numberOfActions);
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), m_owner->m_parameters.m_numberOfObservations);
	GetObservation(&observation[0]);
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);

	m_owner->m_policy.MakePrediction(observation, actions, m_workingBuffer);
	// explore environment
	for (ndInt32 i = m_owner->m_parameters.m_numberOfActions - 1; i >= 0; --i)
	{
		actions[i] = PerturbeAction(actions[i]);
	}
	
	ApplyActions(&actions[0]);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());
	
	//// Get Q vale from Critic
	//ndReal buffer[256];
	//ndDeepBrainMemVector criticInput(buffer, statesDim + actionDim);;
	//for (ndInt32 i = 0; i < statesDim; ++i)
	//{
	//	criticInput[i] = m_state[i];
	//}
	//for (ndInt32 i = 0; i < actionDim; ++i)
	//{
	//	criticInput[i + statesDim] = m_actions[i];
	//}
	//ndDeepBrainMemVector criticOutput(&m_currentQValue, 1);
	//m_critic->MakePrediction(criticInput, criticOutput);
}

ndBrainAgentDDPG_Trainer::ndBrainAgentDDPG_Trainer(const HyperParameters& parameters)
	:ndBrainThreadPool()
	,m_parameters(parameters)
	,m_policy()
	,m_critic()
	,m_referencePolicy()
	,m_referenceCritic()
	,m_name()
	,m_replayBuffer(m_parameters.m_numberOfActions, m_parameters.m_numberOfObservations)
	,m_agent(nullptr)
	,m_policyOptimizer()
	,m_criticOptimizer()
	,m_policyTrainers()
	,m_criticTrainers()
	,m_shuffleBuffer()
	,m_averageScore()
	,m_averageFramesPerEpisodes()
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_replayBufferIndex(0)
	,m_startOptimization(false)
	,m_shuffleIndexBuffer(0)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);
	//ndSetRandSeed(m_randomSeed);

	// create actor class
	SetThreadCount(m_parameters.m_threadsCount);

	BuildPolicyClass();
	BuildCriticClass();
}

ndBrainAgentDDPG_Trainer::~ndBrainAgentDDPG_Trainer()
{
}

ndBrain* ndBrainAgentDDPG_Trainer::GetPolicyNetwork()
{
	return &m_policy;
}

const ndString& ndBrainAgentDDPG_Trainer::GetName() const
{
	return m_name;
}

void ndBrainAgentDDPG_Trainer::SetName(const ndString& name)
{
	m_name = name;
}

void ndBrainAgentDDPG_Trainer::BuildPolicyClass()
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

void ndBrainAgentDDPG_Trainer::BuildCriticClass()
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
		m_critic.AddLayer(layers[i]);
	}
	m_critic.InitWeights();

	m_criticOptimizer = ndSharedPtr<ndBrainOptimizerAdam> (new ndBrainOptimizerAdam());
	m_criticOptimizer->SetRegularizer(m_parameters.m_criticRegularizer);
	m_criticOptimizer->SetRegularizerType(m_parameters.m_criticRegularizerType);

	m_criticTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic);
		m_criticTrainers.PushBack(trainer);
	}
	m_referenceCritic = m_critic;
}

ndUnsigned32 ndBrainAgentDDPG_Trainer::GetFramesCount() const
{
	return m_frameCount;
}

ndUnsigned32 ndBrainAgentDDPG_Trainer::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentDDPG_Trainer::GetAverageScore() const
{
	return m_averageScore.GetAverage();
}

ndFloat32 ndBrainAgentDDPG_Trainer::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}


#pragma optimize( "", off )
void ndBrainAgentDDPG_Trainer::SaveTrajectory()
{
	// remove all dead states except the last 
	while (m_agent->m_trajectory.GetTerminalState(m_agent->m_trajectory.GetCount() - 2))
	{
		m_agent->m_trajectory.SetCount(m_agent->m_trajectory.GetCount() - 1);
	}

	// if it has enough states, just add the to the replay buffer
	if (m_agent->m_trajectory.GetCount() >= 4)
	{
		// make sure last state is terminal
		m_agent->m_trajectory.SetTerminalState(m_agent->m_trajectory.GetCount() - 1, true);

		m_averageFramesPerEpisodes.Update(ndBrainFloat(m_agent->m_trajectory.GetCount()));
		if (m_replayBuffer.GetCount() < m_parameters.m_replayBufferSize)
		{
			// populate replay buffer
			ndInt32 index = m_replayBuffer.GetCount();

			// add last state, since it is unique
			m_replayBuffer.SetCount(m_replayBuffer.GetCount() + 1);
			m_replayBuffer.CopyFrom(index, m_agent->m_trajectory, m_agent->m_trajectory.GetCount() - 1);
			m_shuffleBuffer.PushBack(index);
			index++;
			for (ndInt32 i = m_agent->m_trajectory.GetCount() - 2; (i >= 0) && (index < m_parameters.m_replayBufferSize); --i)
			{
				m_shuffleBuffer.PushBack(index);
				m_replayBuffer.SetCount(m_replayBuffer.GetCount() + 1);
				m_replayBuffer.CopyFrom(index, m_agent->m_trajectory, i);
				ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(index), m_parameters.m_numberOfObservations);
				nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations));
				index++;
			}

			// we now have enough transition to start optimization.
			if (!m_startOptimization && (m_replayBuffer.GetCount() >= m_parameters.m_replayBufferStartOptimizeSize))
			{
				m_replayBufferIndex = 0;
				m_shuffleIndexBuffer = 0;
				m_startOptimization = true;
			}
		}
		else
		{
			// overide old transitions.
			m_replayBuffer.CopyFrom(m_replayBufferIndex, m_agent->m_trajectory, m_agent->m_trajectory.GetCount() - 1);
			m_replayBufferIndex = (m_replayBufferIndex + 1) % m_parameters.m_replayBufferSize;
			for (ndInt32 i = m_agent->m_trajectory.GetCount() - 2; i >= 0; --i)
			{
				m_replayBuffer.CopyFrom(m_replayBufferIndex, m_agent->m_trajectory, i);
				ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(m_replayBufferIndex), m_parameters.m_numberOfObservations);
				nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations));
				m_replayBufferIndex = (m_replayBufferIndex + 1) % m_parameters.m_replayBufferSize;
			}
		}
		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	}

	m_agent->m_trajectory.SetCount(0);
	m_agent->ResetModel();
}

#pragma optimize( "", off )
void ndBrainAgentDDPG_Trainer::LearnQvalueFunction()
{
	ndBrainFixSizeVector<256> expectedRewards;
	expectedRewards.SetCount(GetThreadCount());
	expectedRewards.Set(ndBrainFloat(0.0f));
	for (ndInt32 n = m_parameters.m_criticUpdatesCount - 1; n >= 0; --n)
	{
		ndFixSizeArray<ndInt32, 1024> indirectBuffer;
		for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
		{
			indirectBuffer.PushBack(m_shuffleBuffer[m_shuffleIndexBuffer]);
			m_shuffleIndexBuffer = (m_shuffleIndexBuffer + 1) % ndInt32(m_shuffleBuffer.GetCount());
		}

		ndAtomic<ndInt32> iterator(0);
		auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, &indirectBuffer, &expectedRewards](ndInt32 threadIndex, ndInt32)
			{
				class ndLoss : public ndBrainLossLeastSquaredError
				{
				public:
				ndLoss(ndBrainAgentDDPG_Trainer* const owner, const ndInt32 index, ndInt32 threadIndex, ndBrainFixSizeVector<256>& expectedRewards)
					:ndBrainLossLeastSquaredError(1)
					,m_owner(owner)
					,m_criticQvalue()
					,m_nexActionObservation()
					,m_expectedRewards(expectedRewards)
					,m_gamma(m_owner->m_parameters.m_discountRewardFactor)
					,m_threadIndex(threadIndex)
					,m_index(index)
					{
						m_criticQvalue.SetCount(1);
						m_nexActionObservation.SetCount(m_owner->m_parameters.m_numberOfObservations + m_owner->m_parameters.m_numberOfActions);
					}

					void GetLoss(const ndBrainVector& criticQvalue, ndBrainVector& loss)
					{
						ndBrainFloat reward = m_owner->m_replayBuffer.GetReward(m_index);
						if (!m_owner->m_replayBuffer.GetTerminalState(m_index))
						{
							ndBrainMemVector nextAction(&m_nexActionObservation[0], m_owner->m_parameters.m_numberOfActions);
							const ndBrainMemVector nextObservation(m_owner->m_replayBuffer.GetNextObservations(m_index), m_owner->m_parameters.m_numberOfObservations);
							m_owner->m_referencePolicy.MakePrediction(nextObservation, nextAction);
							ndMemCpy(&m_nexActionObservation[m_owner->m_parameters.m_numberOfActions], &nextObservation[0], nextObservation.GetCount());
							m_owner->m_referenceCritic.MakePrediction(m_nexActionObservation, m_criticQvalue);
							reward += m_gamma * m_criticQvalue[0];
						}
						m_criticQvalue[0] = reward;
						m_expectedRewards[m_threadIndex] += reward;

						SetTruth(m_criticQvalue);
						ndBrainLossLeastSquaredError::GetLoss(criticQvalue, loss);
					}

					ndBrainAgentDDPG_Trainer* m_owner;
					ndBrainFixSizeVector<1> m_criticQvalue;
					ndBrainFixSizeVector<256> m_nexActionObservation;
					ndBrainFixSizeVector<256>& m_expectedRewards;
					ndBrainFloat m_gamma;
					ndInt32 m_threadIndex;
					ndInt32 m_index;
				};

				ndBrainFixSizeVector<256> criticObservationAction;
				criticObservationAction.SetCount(m_parameters.m_numberOfActions + m_parameters.m_numberOfObservations);
				for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
				{
					const ndInt32 index = indirectBuffer[i];
					ndBrainTrainer& trainer = *m_criticTrainers[i];

					ndMemCpy(&criticObservationAction[0], m_replayBuffer.GetActions(index), m_parameters.m_numberOfActions);
					ndMemCpy(&criticObservationAction[m_parameters.m_numberOfActions], m_replayBuffer.GetObservations(index), m_parameters.m_numberOfObservations);

					ndLoss loss(this, index, threadIndex, expectedRewards);
					trainer.BackPropagate(criticObservationAction, loss);
				}
			});

		ndBrainThreadPool::ParallelExecute(BackPropagateBash);
		m_criticOptimizer->Update(this, m_criticTrainers, m_parameters.m_criticLearnRate);
	}

	ndFloat32 expectedRewardSum = 0.0f;
	for (ndInt32 i = ndInt32 (expectedRewards.GetCount()) - 1; i >= 0; --i)
	{
		expectedRewardSum += expectedRewards[i];
	}
	ndFloat32 averageExpectedReward = expectedRewardSum / ndFloat32(m_parameters.m_miniBatchSize * m_parameters.m_criticUpdatesCount);
	m_averageScore.Update(averageExpectedReward);
}

#pragma optimize( "", off )
void ndBrainAgentDDPG_Trainer::LearnPolicyFunction()
{
	for (ndInt32 n = m_parameters.m_policyUpdatesCount - 1; n >= 0; --n)
	{
		ndFixSizeArray<ndInt32, 1024> indirectBuffer;
		for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
		{
			indirectBuffer.PushBack(m_shuffleBuffer[m_shuffleIndexBuffer]);
			m_shuffleIndexBuffer = (m_shuffleIndexBuffer + 1) % ndInt32(m_shuffleBuffer.GetCount());
		}

		ndAtomic<ndInt32> iterator(0);
		auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, &indirectBuffer](ndInt32, ndInt32)
		{
			class ndLoss : public ndBrainLossLeastSquaredError
			{
				public:
				ndLoss(ndBrainAgentDDPG_Trainer* const owner, const ndInt32 index)
					:ndBrainLossLeastSquaredError(1)
					,m_owner(owner)
					,m_combinedActionObservation()
					,m_index(index)
				{
					m_combinedInputGradients.SetCount(m_owner->m_parameters.m_numberOfObservations + m_owner->m_parameters.m_numberOfActions);
					m_combinedActionObservation.SetCount(m_owner->m_parameters.m_numberOfObservations + m_owner->m_parameters.m_numberOfActions);
				}

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					ndMemCpy(&m_combinedActionObservation[0], &output[0], m_owner->m_parameters.m_numberOfActions);
					ndMemCpy(&m_combinedActionObservation[m_owner->m_parameters.m_numberOfActions], m_owner->m_replayBuffer.GetNextObservations(m_index), m_owner->m_parameters.m_numberOfObservations);
					m_owner->m_critic.CalculateInputGradient(output, m_combinedInputGradients);
					ndMemCpy(&loss[0], &m_combinedInputGradients[0], loss.GetCount());
					loss.Scale(-1.0f);
				}

				ndBrainAgentDDPG_Trainer* m_owner;
				ndBrainFixSizeVector<256> m_combinedInputGradients;
				ndBrainFixSizeVector<256> m_combinedActionObservation;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				const ndInt32 index = indirectBuffer[i];
				ndBrainTrainer& trainer = *m_policyTrainers[i];
				const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), m_parameters.m_numberOfObservations);

				ndLoss loss(this, index);
				trainer.BackPropagate(observation, loss);
			}
		});

		ndBrainThreadPool::ParallelExecute(BackPropagateBash);
		m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
	}
}

void ndBrainAgentDDPG_Trainer::Optimize()
{
	LearnQvalueFunction();
	LearnPolicyFunction();

	m_referenceCritic.SoftCopy(m_critic, m_parameters.m_criticMovingAverageFactor);
	m_referencePolicy.SoftCopy(m_policy, m_parameters.m_policyMovingAverageFactor);
}

#pragma optimize( "", off )
void ndBrainAgentDDPG_Trainer::OptimizeStep()
{
	bool isTeminal = m_agent->m_trajectory.GetTerminalState(m_agent->m_trajectory.GetCount() - 1);
	isTeminal = isTeminal || (m_agent->m_trajectory.GetCount() >= m_parameters.m_maxTrajectorySteps);
	if (isTeminal)
	{
		SaveTrajectory();
		if (m_startOptimization)
		{
			Optimize();
			m_eposideCount++;
			m_framesAlive = 0;
		}
	}
	m_frameCount++;
	m_framesAlive++;
}