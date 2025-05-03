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
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainLayerActivationPolicyGradientMeanSigma.h"
#include "ndBrainAgentDeterministicPolicyGradient_Trainer.h"

#define ND_DETERMINISTIC_POLICY_FIX_SIGMA							ndBrainFloat(0.5f)

#define ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationRelu
//#define ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationTanh
//#define ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationLeakyRelu

ndBrainAgentDeterministicPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = ndBrainFloat(1.0e-4f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_discountRewardFactor = ndBrainFloat(0.99f);

	m_entropyRegularizerCoef = ndBrainFloat(0.01f);
	m_policyMovingAverageFactor = ndBrainFloat(0.005f);
	m_criticMovingAverageFactor = ndBrainFloat(0.005f);

	//m_useFixSigma = true;
	m_useFixSigma = false;
	m_actionFixSigma = ND_DETERMINISTIC_POLICY_FIX_SIGMA;

	m_policyRegularizerType = ndBrainOptimizer::m_ridge;
	m_criticRegularizerType = ndBrainOptimizer::m_ridge;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_randomSeed = 47;
	m_actorHiddenLayers = 3;
	m_policyUpdatesCount = 16;
	m_criticUpdatesCount = 16;
	m_maxTrajectorySteps = 1024 * 4;
	m_hiddenLayersNumberOfNeurons = 64;
	m_replayBufferStartOptimizeSize = m_maxTrajectorySteps * 4;

	m_replayBufferSize = 1024 * 1024;
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);

//m_threadsCount = 1;
//m_policyUpdatesCount = 2;
//m_criticUpdatesCount = 2;
//m_maxTrajectorySteps = 100;
}

ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::ndTrajectoryTransition()
	:ndBrainVector()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize;
	m_obsevationsSize = obsevationsSize;
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::CalculateStride() const
{
	return m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	ndMemSet(&me[stride * entry], ndBrainFloat(0.0f), stride);
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	ndMemCpy(&me[stride * entry], &src[stride * srcEntry], stride);
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetCount() const
{
	const ndInt64 stride = CalculateStride();
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	const ndInt64 stride = CalculateStride();
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return me[stride * entry + m_reward];
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	me[stride * entry + m_reward] = reward;
}

bool ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return (me[stride * entry + m_isterminalState] == ndBrainFloat(999.0f)) ? true : false;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	me[stride * entry + m_isterminalState] = isTernimal ? ndBrainFloat(999.0f) : ndBrainFloat(-999.0f);
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize];
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize];
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize + m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize + m_obsevationsSize];
}

ndBrainAgentDeterministicPolicyGradient_Agent::ndBrainAgentDeterministicPolicyGradient_Agent(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master)
	:ndBrainAgent()
	,m_owner(master)
	,m_trajectory()
	,m_workingBuffer()
	,m_trajectoryBaseCount(0)
	,m_randomeGenerator()
{
	m_owner->m_agent = this;
	m_trajectory.Init(m_owner->m_policy.GetOutputSize(), m_owner->m_parameters.m_numberOfObservations);
	m_randomeGenerator.m_gen.seed(m_owner->m_parameters.m_randomSeed);
}

ndBrainAgentDeterministicPolicyGradient_Agent::~ndBrainAgentDeterministicPolicyGradient_Agent()
{
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::SampleActions(ndBrainVector& actions)
{
	const ndInt32 count = ndInt32(actions.GetCount()) / 2;
	const ndInt32 start = ndInt32(actions.GetCount()) / 2;
	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		ndFloat32 sigma = actions[start + i];
		ndBrainFloat unitVarianceSample = m_randomeGenerator.m_d(m_randomeGenerator.m_gen);
		ndBrainFloat sample = ndBrainFloat(actions[i]) + unitVarianceSample * sigma;
		ndBrainFloat clippedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = clippedAction;
	}
}

void ndBrainAgentDeterministicPolicyGradient_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	const ndBrainAgentDeterministicPolicyGradient_Trainer* const owner = *m_owner;
	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), owner->m_policy.GetOutputSize());
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), owner->m_parameters.m_numberOfObservations);

	GetObservation(&observation[0]);
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());
	
	owner->m_policy.MakePrediction(observation, actions, m_workingBuffer);

	SampleActions(actions);
	ApplyActions(&actions[0]);
}

ndBrainAgentDeterministicPolicyGradient_Trainer::ndBrainAgentDeterministicPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndBrainThreadPool()
	,m_name()
	,m_parameters(parameters)
	,m_policy()
	,m_referencePolicy()
	,m_policyTrainers()
	,m_policyOptimizer()
	,m_expectedRewards()
	,m_miniBatchIndexBuffer()
	,m_replayBuffer()
	,m_agent(nullptr)
	,m_shuffleBuffer()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_replayBufferIndex(0)
	,ndPolycyDelayMod(0)
	,m_shuffleBatchIndex(0)
	,m_startOptimization(false)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);

	m_parameters.m_criticUpdatesCount = ndMax(m_parameters.m_criticUpdatesCount, 2);
	m_parameters.m_policyUpdatesCount = ndMax(m_parameters.m_policyUpdatesCount, 2);

	// create actor class
	SetThreadCount(m_parameters.m_threadsCount);

	BuildPolicyClass();
	BuildCriticClass();
}

ndBrainAgentDeterministicPolicyGradient_Trainer::~ndBrainAgentDeterministicPolicyGradient_Trainer()
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

ndBrain* ndBrainAgentDeterministicPolicyGradient_Trainer::GetPolicyNetwork()
{
	return &m_policy;
}

const ndString& ndBrainAgentDeterministicPolicyGradient_Trainer::GetName() const
{
	return m_name;
}

void ndBrainAgentDeterministicPolicyGradient_Trainer::SetName(const ndString& name)
{
	m_name = name;
}

void ndBrainAgentDeterministicPolicyGradient_Trainer::BuildPolicyClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < m_parameters.m_actorHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions + m_parameters.m_numberOfActions));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	layers.PushBack(new ndBrainLayerActivationPolicyGradientMeanSigma(layers[layers.GetCount() - 1]->GetOutputSize()));

	ndBrainFixSizeVector<256> bias;
	ndBrainFixSizeVector<256> slope;
	bias.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
	slope.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
	bias.Set(ndBrainFloat(0.0f));
	slope.Set(ndBrainFloat(1.0f));
	if (m_parameters.m_useFixSigma)
	{
		ndInt32 size = layers[layers.GetCount() - 1]->GetOutputSize() / 2;

		ndMemSet(&slope[size], ndBrainFloat(0.0f), size);
		ndMemSet(&bias[size], m_parameters.m_actionFixSigma, size);
	}
	layers.PushBack(new ndBrainLayerActivationLinear(slope, bias));

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
	m_replayBuffer.Init(m_policy.GetOutputSize(), m_policy.GetInputSize());
}

void ndBrainAgentDeterministicPolicyGradient_Trainer::BuildCriticClass()
{
	for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
	{
		ndFixSizeArray<ndBrainLayer*, 32> layers;

		layers.SetCount(0);
		layers.PushBack(new ndBrainLayerLinear(m_policy.GetOutputSize() + m_policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

		for (ndInt32 i = 0; i < m_parameters.m_actorHiddenLayers; ++i)
		{
			ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
		}
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
		layers.PushBack(new ndBrainLayerActivationLeakyRelu(layers[layers.GetCount() - 1]->GetOutputSize()));

		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			m_critic[k].AddLayer(layers[i]);
		}
		m_critic[k].InitWeights();

		m_criticTrainers[k].SetCount(0);
		for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
		{
			ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic[k]);
			m_criticTrainers[k].PushBack(trainer);
		}
		m_referenceCritic[k] = m_critic[k];

		m_criticOptimizer[k] = ndSharedPtr<ndBrainOptimizerAdam>(new ndBrainOptimizerAdam());
		m_criticOptimizer[k]->SetRegularizer(m_parameters.m_criticRegularizer);
		m_criticOptimizer[k]->SetRegularizerType(m_parameters.m_criticRegularizerType);
	}
}

bool ndBrainAgentDeterministicPolicyGradient_Trainer::IsSampling() const
{
	return !m_startOptimization;
}

ndUnsigned32 ndBrainAgentDeterministicPolicyGradient_Trainer::GetFramesCount() const
{
	return m_frameCount;
}

ndUnsigned32 ndBrainAgentDeterministicPolicyGradient_Trainer::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentDeterministicPolicyGradient_Trainer::GetAverageScore() const
{
	return m_averageExpectedRewards.GetAverage();
}

ndFloat32 ndBrainAgentDeterministicPolicyGradient_Trainer::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::SaveTrajectory()
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

		ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(dstIndex), m_policy.GetInputSize());
		nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(srcIndex + 1), m_policy.GetInputSize()));
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
			
			ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(m_replayBufferIndex), m_policy.GetInputSize());
			nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(i + 1), m_policy.GetInputSize()));

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

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnQvalueFunction(ndInt32 criticIndex)
{
	ndInt32 base = 0;
	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 n = m_parameters.m_criticUpdatesCount - 1; n >= 0; --n)
	{
		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base, criticIndex](ndInt32, ndInt32)
		{
			ndBrainFixSizeVector<256> criticObservationAction;
			criticObservationAction.SetCount(m_policy.GetOutputSize() + m_policy.GetInputSize());

			ndBrainLossLeastSquaredError loss(1);
			ndBrainFixSizeVector<1> groundTruth;
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				const ndInt32 index = m_miniBatchIndexBuffer[i + base];
				ndBrainTrainer& trainer = *m_criticTrainers[criticIndex][i];

				ndMemCpy(&criticObservationAction[0], m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
				ndMemCpy(&criticObservationAction[m_policy.GetOutputSize()], m_replayBuffer.GetObservations(index), m_policy.GetInputSize());

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

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::CalculateExpectedRewards()
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
		ndBrainFixSizeVector<256> criticObservationAction;

		ndAssert(count % batchSize == 0);
		criticObservationAction.SetCount(m_policy.GetOutputSize() + m_policy.GetInputSize());
		for (ndInt32 base = iterator.fetch_add(batchSize); base < count; base = iterator.fetch_add(batchSize))
		{
			for (ndInt32 i = 0; i < batchSize; ++i)
			{
				ndBrainFixSizeVector<ND_NUMBER_OF_CRITICS> rewards;
				rewards.SetCount(0);
				const ndInt32 index = m_miniBatchIndexBuffer[i + base];
				ndBrainFloat r = m_replayBuffer.GetReward(index);
				for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
				{
					rewards.PushBack(r);
				}
				if (!m_replayBuffer.GetTerminalState(index))
				{
					ndBrainMemVector nextAction(&criticObservationAction[0], m_policy.GetOutputSize());
					const ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(index), m_policy.GetInputSize());
					m_referencePolicy.MakePrediction(nextObservation, nextAction);
					ndMemCpy(&criticObservationAction[m_policy.GetOutputSize()], &nextObservation[0], nextObservation.GetCount());

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
				m_expectedRewards[i + base] = minQ;
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

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnPolicyFunction()
{
	ndAtomic<ndInt32> iterator(0);
	ndInt32 base = 0;
	for (ndInt32 n = m_parameters.m_policyUpdatesCount - 1; n >= 0; --n)
	{
		//ndFixSizeArray<ndInt32, 1024> indirectBuffer;
		//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
		//{
		//	indirectBuffer.PushBack(m_shuffleBuffer[m_shuffleBatchIndex]);
		//	m_shuffleBatchIndex = (m_shuffleBatchIndex + 1) % ndInt32(m_shuffleBuffer.GetCount());
		//}
		
		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32 threadIndex, ndInt32)
		{
			class ndLoss : public ndBrainLossLeastSquaredError
			{
				public:
				ndLoss(ndBrainAgentDeterministicPolicyGradient_Trainer* const owner, const ndInt32 index, ndInt32 m_threadIndex)
					:ndBrainLossLeastSquaredError(1)
					,m_criticLoss()
					,m_combinedInputGradients()
					,m_combinedActionObservation()
					,m_owner(owner)
					,m_index(index)
					,m_threadIndex(m_threadIndex)
				{
					m_combinedInputGradients.SetCount(m_owner->m_policy.GetInputSize() + m_owner->m_policy.GetOutputSize());
					m_combinedActionObservation.SetCount(m_owner->m_policy.GetInputSize() + m_owner->m_policy.GetOutputSize());
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
						loss[0] = ndBrainFloat(-1.0f);
					}
				};

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					ndMemCpy(&m_combinedActionObservation[0], &output[0], m_owner->m_policy.GetOutputSize());
					ndMemCpy(&m_combinedActionObservation[m_owner->m_policy.GetOutputSize()], m_owner->m_replayBuffer.GetNextObservations(m_index), m_owner->m_policy.GetInputSize());
					m_owner->m_critic[0].CalculateInputGradient(m_combinedActionObservation, m_combinedInputGradients, m_criticLoss);
					ndMemCpy(&loss[0], &m_combinedInputGradients[0], loss.GetCount());
				}

				ndCriticLoss m_criticLoss;
				ndBrainFixSizeVector<256> m_combinedInputGradients;
				ndBrainFixSizeVector<256> m_combinedActionObservation;
				ndBrainAgentDeterministicPolicyGradient_Trainer* m_owner;
				ndInt32 m_index;
				ndInt32 m_threadIndex;
			};

			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				const ndInt32 index = m_miniBatchIndexBuffer[i + base];
				ndBrainTrainer& trainer = *m_policyTrainers[i];
				const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), m_policy.GetInputSize());

				ndLoss loss(this, index, threadIndex);
				trainer.BackPropagate(observation, loss);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
		m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);

		base += m_parameters.m_miniBatchSize;
	}
	m_referencePolicy.SoftCopy(m_policy, m_parameters.m_policyMovingAverageFactor);
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::Optimize()
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

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::OptimizeStep()
{
	SaveTrajectory();
	if (m_startOptimization)
	{
		Optimize();
	}
	m_frameCount++;
	m_framesAlive++;
}