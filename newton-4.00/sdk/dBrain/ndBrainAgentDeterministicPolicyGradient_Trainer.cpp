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
#include "ndBrainTrainerCpu.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainOptimizerAdamCpu.h"
#include "ndBrainOptimizerAdamLegacy.h"
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainAgentDeterministicPolicyGradient_Trainer.h"

#define ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationRelu
//#define ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationTanh
//#define ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationLeakyRelu

#define ND_TD3_POLICY_FIX_SIGMA				ndBrainFloat(0.2f)
#define ND_TD3_POLICY_MIN_PER_ACTION_SIGMA	ndBrainFloat(0.01f)
#define ND_TD3_POLICY_MAX_PER_ACTION_SIGMA	ndBrainFloat(1.0f)
#define ND_MAX_SAC_ENTROPY_COEFFICIENT		ndBrainFloat (2.0e-5f)

ndBrainAgentDeterministicPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = ndBrainFloat(1.0e-4f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_discountRewardFactor = ndBrainFloat(0.99f);

	m_entropyRegularizerCoef = ndBrainFloat(0.0f);
	//m_entropyRegularizerCoef = ndBrainFloat(0.25f);
	m_policyMovingAverageFactor = ndBrainFloat(0.005f);
	m_criticMovingAverageFactor = ndBrainFloat(0.005f);

	m_usePerActionSigmas = false;
	m_actionFixSigma = ND_TD3_POLICY_FIX_SIGMA;

	m_policyRegularizerType = ndBrainOptimizer::m_ridge;
	m_criticRegularizerType = ndBrainOptimizer::m_ridge;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_randomSeed = 47;
	m_numberOfHiddenLayers = 3;
	m_policyUpdatesCount = 16;
	m_criticUpdatesCount = 16;
	m_maxTrajectorySteps = 1024 * 4;
	m_hiddenLayersNumberOfNeurons = 128;
	m_replayBufferStartOptimizeSize = m_maxTrajectorySteps * 4;

	m_replayBufferSize = 1024 * 1024;
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);

//m_threadsCount = 1;
//m_policyUpdatesCount = 2;
//m_criticUpdatesCount = 2;
//m_maxTrajectorySteps = 100;
}

ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::ndTrajectoryTransition()
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize;
	m_obsevationsSize = obsevationsSize;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	m_reward[entry] = ndBrainFloat(0.0f);
	m_terminal[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry)
{
	m_reward[entry] = src.m_reward[srcEntry];
	m_terminal[entry] = src.m_terminal[srcEntry];
	ndMemCpy(&m_actions[entry * m_actionsSize], &src.m_actions[srcEntry * m_actionsSize], m_actionsSize);
	ndMemCpy(&m_observations[entry * m_obsevationsSize], &src.m_observations[srcEntry * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&m_nextObservations[entry * m_obsevationsSize], &src.m_nextObservations[srcEntry * m_obsevationsSize], m_obsevationsSize);
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetCount() const
{
	return ndInt32(m_reward.GetCount());
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	m_reward.SetCount(count);
	m_terminal.SetCount(count);
	m_actions.SetCount(count * m_actionsSize);
	m_observations.SetCount(count * m_obsevationsSize);
	m_nextObservations.SetCount(count * m_obsevationsSize);
}

ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	return m_reward[entry];
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	m_reward[entry] = reward;
}

bool ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	return (m_terminal[entry] == 999.0f) ? true : false;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	m_terminal[entry] = isTernimal ? ndBrainFloat(999.0f) : ndBrainFloat(-999.0f);
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry)
{
	return &m_actions[entry * m_actionsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry) const
{
	return &m_actions[entry * m_actionsSize];
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	return &m_observations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	return &m_observations[entry * m_obsevationsSize];
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry)
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry) const
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

ndBrainAgentDeterministicPolicyGradient_Agent::ndBrainAgentDeterministicPolicyGradient_Agent(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master)
	:ndBrainAgent()
	,m_owner(master)
	,m_trajectory()
	,m_trajectoryBaseCount(0)
	,m_randomeGenerator()
{
	m_owner->m_agent = this;

	const ndBrain* const brain = *m_owner->m_policyTrainer->GetBrain();
	m_trajectory.Init(brain->GetOutputSize(), m_owner->m_parameters.m_numberOfObservations);
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
	if (m_owner->m_parameters.m_usePerActionSigmas)
	{
		const ndInt32 size = ndInt32(actions.GetCount()) / 2;
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			ndBrainFloat sigma = actions[size + i];
			ndBrainFloat unitVarianceSample = m_randomeGenerator.m_d(m_randomeGenerator.m_gen);
			ndBrainFloat sample = ndBrainFloat(actions[i]) + unitVarianceSample * sigma;
			ndBrainFloat clippedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
			actions[i] = clippedAction;
		}
	}
	else
	{
		ndFloat32 sigma = m_owner->m_parameters.m_actionFixSigma;
		const ndInt32 size = ndInt32(actions.GetCount());
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			ndBrainFloat unitVarianceSample = m_randomeGenerator.m_d(m_randomeGenerator.m_gen);
			ndBrainFloat sample = ndBrainFloat(actions[i]) + unitVarianceSample * sigma;
			ndBrainFloat clippedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
			actions[i] = clippedAction;
		}
	}
}

void ndBrainAgentDeterministicPolicyGradient_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);
	
	const ndBrain* const brain = *m_owner->m_policyTrainer->GetBrain();
	const ndBrainAgentDeterministicPolicyGradient_Trainer* const owner = *m_owner;
	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), brain->GetOutputSize());
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), owner->m_parameters.m_numberOfObservations);
	
	GetObservation(&observation[0]);
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());
	
	owner->m_policyTrainer->MakeSinglePrediction(observation, actions);
	
	SampleActions(actions);
	ApplyActions(&actions[0]);
}

ndBrainAgentDeterministicPolicyGradient_Trainer::ndBrainAgentDeterministicPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndBrainThreadPool()
	,m_name()
	,m_parameters(parameters)
	//,m_policy()
	//,m_referencePolicy()
	//,m_policyTrainers()
	//,m_policyOptimizer()
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
	m_replayBuffer.Init(m_policyTrainer->GetBrain()->GetOutputSize(), m_policyTrainer->GetBrain()->GetInputSize());

	ndBrainFloat unitEntropy = ndClamp(m_parameters.m_entropyRegularizerCoef, ndBrainFloat(0.0f), ndBrainFloat(1.0f));
	m_parameters.m_entropyRegularizerCoef = ND_MAX_SAC_ENTROPY_COEFFICIENT * unitEntropy;
}

ndBrainAgentDeterministicPolicyGradient_Trainer::~ndBrainAgentDeterministicPolicyGradient_Trainer()
{
}

ndBrain* ndBrainAgentDeterministicPolicyGradient_Trainer::GetPolicyNetwork()
{
	return *m_policyTrainer->GetBrain();
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
	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	ndInt32 numberOfOutput = m_parameters.m_usePerActionSigmas ? 2 * m_parameters.m_numberOfActions : m_parameters.m_numberOfActions;
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), numberOfOutput));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	if (m_parameters.m_usePerActionSigmas)
	{
		ndBrainFixSizeVector<256> bias;
		ndBrainFixSizeVector<256> slope;
		bias.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
		slope.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());

		ndInt32 sigmaSize = numberOfOutput / 2;
		ndBrainFloat b = ndBrainFloat(0.5f) * (ND_TD3_POLICY_MAX_PER_ACTION_SIGMA + ND_TD3_POLICY_MIN_PER_ACTION_SIGMA);
		ndBrainFloat a = ndBrainFloat(0.5f) * (ND_TD3_POLICY_MAX_PER_ACTION_SIGMA - ND_TD3_POLICY_MIN_PER_ACTION_SIGMA);

		bias.Set(ndBrainFloat(0.0f));
		slope.Set(ndBrainFloat(1.0f));
		ndMemSet(&bias[sigmaSize], b, sigmaSize);
		ndMemSet(&slope[sigmaSize], a, sigmaSize);
		layers.PushBack(new ndBrainLayerActivationLinear(slope, bias));
	}

	ndSharedPtr<ndBrain> policy (new ndBrain);
	//for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	//{
	//	m_policy.AddLayer(layers[i]);
	//}
	//m_policy.InitWeights();
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		policy->AddLayer(layers[i]);
	}
	policy->InitWeights();

	ndSharedPtr<ndBrainOptimizerAdamCpu> policyOptimizer(new ndBrainOptimizerAdamCpu());
	policyOptimizer->SetRegularizer(m_parameters.m_policyRegularizer);
	policyOptimizer->SetRegularizerType(m_parameters.m_policyRegularizerType);
	m_policyTrainer = ndSharedPtr<ndBrainTrainerCpu>(new ndBrainTrainerCpu(policy, policyOptimizer, this, m_parameters.m_miniBatchSize));
	
	ndSharedPtr<ndBrain> referencePolicy(new ndBrain(**policy));
	m_referencePolicy___ = ndSharedPtr<ndBrainTrainerCpuInference>(new ndBrainTrainerCpuInference(referencePolicy, this, m_parameters.m_miniBatchSize));
}

void ndBrainAgentDeterministicPolicyGradient_Trainer::BuildCriticClass()
{
	const ndBrain& policy = **m_policyTrainer->GetBrain();
	for (ndInt32 k = 0; k < sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0]); ++k)
	{
		ndFixSizeArray<ndBrainLayer*, 32> layers;

		layers.SetCount(0);
		layers.PushBack(new ndBrainLayerLinear(policy.GetOutputSize() + policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

		for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
		{
			ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ND_DETERMINISTIC_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
		}
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));

		ndSharedPtr<ndBrain> critic(new ndBrain);
		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			critic->AddLayer(layers[i]);
		}
		critic->InitWeights();
		ndSharedPtr<ndBrainOptimizerAdamCpu> criticOptimizer(new ndBrainOptimizerAdamCpu());
		criticOptimizer->SetRegularizer(m_parameters.m_policyRegularizer);
		criticOptimizer->SetRegularizerType(m_parameters.m_policyRegularizerType);
		m_criticTrainer[k] = ndSharedPtr<ndBrainTrainerCpu>(new ndBrainTrainerCpu(critic, criticOptimizer, this, m_parameters.m_miniBatchSize));

		ndSharedPtr<ndBrain> referenceCritic(new ndBrain(**critic));
		ndSharedPtr<ndBrainOptimizerAdamCpu> criticOptimizer1(new ndBrainOptimizerAdamCpu());
		m_referenceCriticTrainer[k] = ndSharedPtr<ndBrainTrainerCpu>(new ndBrainTrainerCpu(referenceCritic, criticOptimizer1, this, m_parameters.m_miniBatchSize));
	}
}

ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution) const
{
	ndAssert(0);
	return 0;
	//ndBrainFloat z2 = ndBrainFloat(0.0f);
	//ndBrainFloat invSigma2Det = ndBrainFloat(1.0f);
	//ndBrainFloat invSqrtPi = ndBrainFloat(1.0f) / ndSqrt(2.0f * ndPi);
	//
	//ndBrainFloat prob = 1.0f;
	//if (m_parameters.m_usePerActionSigmas)
	//{
	//	const ndInt32 size = ndInt32(distribution.GetCount()) / 2;
	//
	//	const ndBrainMemVector sampledProbabilities(m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
	//	for (ndInt32 i = size - 1; i >= 0; --i)
	//	{
	//		ndBrainFloat sigma = distribution[i + size];
	//		ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
	//		ndBrainFloat z = (sampledProbabilities[i] - distribution[i]) * invSigma;
	//
	//		z2 += (z * z);
	//		invSigma2Det *= (invSqrtPi * invSigma);
	//	}
	//	ndBrainFloat exponent = ndBrainFloat(0.5f) * z2;
	//	prob = invSigma2Det * ndBrainFloat(ndExp(-exponent));
	//}
	//else
	//{
	//	const ndInt32 count = ndInt32(distribution.GetCount());
	//
	//	ndBrainFloat sigma = m_parameters.m_actionFixSigma;
	//	ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
	//	const ndBrainMemVector sampledProbabilities(m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
	//	for (ndInt32 i = count - 1; i >= 0; --i)
	//	{
	//		ndBrainFloat z = (sampledProbabilities[i] - distribution[i]) * invSigma;
	//		z2 += z * z;
	//		invSigma2Det *= (invSqrtPi * invSigma);
	//	}
	//	ndBrainFloat exponent = ndBrainFloat(0.5f) * z2;
	//	prob = invSigma2Det * ndBrainFloat(ndExp(-exponent));
	//}
	//return ndMax(prob, ndBrainFloat(1.0e-4f));
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32 index) const
{
	ndAssert(0);
	return 0;
	//const ndBrainMemVector sampledProbabilities(m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
	//return CalculatePolicyProbability(index, sampledProbabilities);
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
	
		const ndBrain& brain = **m_policyTrainer->GetBrain();
		ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(dstIndex), brain.GetInputSize());
		nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(srcIndex + 1), brain.GetInputSize()));
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
		const ndBrain& brain = **m_policyTrainer->GetBrain();
		const ndInt32 numOfTransitions = m_agent->m_trajectory.GetCount() - 1;
		for (ndInt32 i = m_agent->m_trajectoryBaseCount; i < numOfTransitions; ++i)
		{
			m_replayBuffer.CopyFrom(m_replayBufferIndex, m_agent->m_trajectory, i);
			
			ndBrainFloat reward = m_agent->m_trajectory.GetReward(i + 1);
			m_replayBuffer.SetReward(m_replayBufferIndex, reward);
			
			bool terminalState = m_agent->m_trajectory.GetTerminalState(i + 1);
			m_replayBuffer.SetTerminalState(m_replayBufferIndex, terminalState);
			
			ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(m_replayBufferIndex), brain.GetInputSize());
			nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(i + 1), brain.GetInputSize()));
	
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
			m_averageFramesPerEpisodes.Update(ndReal (m_agent->m_trajectoryBaseCount));
	
			// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
			ndBrainFloat gamma = m_parameters.m_discountRewardFactor;
			ndBrainFloat stateReward = m_agent->m_trajectory.GetReward(m_agent->m_trajectory.GetCount() - 1);
			ndBrainFloat averageReward = stateReward;
			for (ndInt32 i = m_agent->m_trajectory.GetCount() - 2; i >= 0; --i)
			{
				ndBrainFloat r = m_agent->m_trajectory.GetReward(i);
				stateReward = r + gamma * stateReward;
				averageReward += stateReward;
			}
			averageReward /= ndBrainFloat(m_agent->m_trajectory.GetCount());
			m_averageExpectedRewards.Update(averageReward);
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
	ndAssert(0);
#if 0
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
#endif
}

//#pragma optimize( "", off )
//void ndBrainAgentDeterministicPolicyGradient_Trainer::CalculateExpectedRewards()
//{
//	ndInt32 count = m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize;
//	m_expectedRewards.SetCount(count);
//
//	m_miniBatchIndexBuffer.SetCount(0);
//	for (ndInt32 i = 0; i < count; ++i)
//	{
//		m_miniBatchIndexBuffer.PushBack(m_shuffleBuffer[m_shuffleBatchIndex]);
//		m_shuffleBatchIndex = (m_shuffleBatchIndex + 1) % ndInt32(m_shuffleBuffer.GetCount());
//	}
//
//	ndAtomic<ndInt32> iterator(0);
//	auto ExpectedRewards = ndMakeObject::ndFunction([this, count, &iterator](ndInt32, ndInt32)
//	{
//		const ndInt32 batchSize = 128;
//		ndBrainFixSizeVector<256> criticNextObservationAction;
//
//		ndAssert(count % batchSize == 0);
//		criticNextObservationAction.SetCount(m_policy.GetOutputSize() + m_policy.GetInputSize());
//		for (ndInt32 base = iterator.fetch_add(batchSize); base < count; base = iterator.fetch_add(batchSize))
//		{
//			for (ndInt32 j = 0; j < batchSize; ++j)
//			{
//				ndBrainFixSizeVector<ND_NUMBER_OF_CRITICS> rewards;
//				rewards.SetCount(0);
//				const ndInt32 index = m_miniBatchIndexBuffer[j + base];
//				ndBrainFloat r = m_replayBuffer.GetReward(index);
//				for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
//				{
//					rewards.PushBack(r);
//				}
//				if (!m_replayBuffer.GetTerminalState(index))
//				{
//					ndBrainMemVector nextAction(&criticNextObservationAction[0], m_policy.GetOutputSize());
//					const ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(index), m_policy.GetInputSize());
//					m_referencePolicy.MakePrediction(nextObservation, nextAction);
//					ndMemCpy(&criticNextObservationAction[m_policy.GetOutputSize()], &nextObservation[0], nextObservation.GetCount());
//
//					ndBrainFixSizeVector<1> criticQvalue;
//					for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
//					{
//						m_referenceCritic[k].MakePrediction(criticNextObservationAction, criticQvalue);
//						rewards[k] += m_parameters.m_discountRewardFactor * criticQvalue[0];
//					}
//				}
//
//				ndBrainFloat minQ = ndBrainFloat(1.0e10f);
//				for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
//				{
//					minQ = ndMin(minQ, rewards[i]);
//				}
//				m_expectedRewards[j + base] = minQ;
//			}
//		}
//	});
//	ndBrainThreadPool::ParallelExecute(ExpectedRewards);
//}

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
	
	//ndAtomic<ndInt32> iterator(0);
	//auto ExpectedRewards = ndMakeObject::ndFunction([this, count, &iterator](ndInt32, ndInt32)
	//{
	//	const ndInt32 batchSize = 128;
	//	ndBrainFixSizeVector<256> policyEntropyAction;
	//	ndBrainFixSizeVector<256> criticNextObservationAction;
	//
	//	ndAssert(count % batchSize == 0);
	//	policyEntropyAction.SetCount(m_policy.GetOutputSize());
	//	criticNextObservationAction.SetCount(m_policy.GetOutputSize() + m_policy.GetInputSize());
	//	for (ndInt32 base = iterator.fetch_add(batchSize); base < count; base = iterator.fetch_add(batchSize))
	//	{
	//		for (ndInt32 j = 0; j < batchSize; ++j)
	//		{
	//			ndBrainFixSizeVector<ND_NUMBER_OF_CRITICS> rewards;
	//			rewards.SetCount(0);
	//			const ndInt32 index = m_miniBatchIndexBuffer[j + base];
	//			ndBrainFloat r = m_replayBuffer.GetReward(index);
	//			for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
	//			{
	//				rewards.PushBack(r);
	//			}
	//			if (!m_replayBuffer.GetTerminalState(index))
	//			{
	//				ndBrainMemVector nextAction(&criticNextObservationAction[0], m_policy.GetOutputSize());
	//				const ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(index), m_policy.GetInputSize());
	//				m_policy.MakePrediction(nextObservation, nextAction);
	//				ndMemCpy(&criticNextObservationAction[m_policy.GetOutputSize()], &nextObservation[0], nextObservation.GetCount());
	//
	//				ndBrainFixSizeVector<1> criticQvalue;
	//				for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
	//				{
	//					m_referenceCritic[i].MakePrediction(criticNextObservationAction, criticQvalue);
	//					rewards[i] += m_parameters.m_discountRewardFactor * criticQvalue[0];
	//				}
	//			}
	//
	//			ndBrainFloat minQ = ndBrainFloat(1.0e10f);
	//			for (ndInt32 i = 0; i < sizeof(m_critic) / sizeof(m_critic[0]); ++i)
	//			{
	//				minQ = ndMin(minQ, rewards[i]);
	//			}
	//
	//			// calculate entropy
	//			if (m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
	//			{
	//				ndBrainFloat prob = CalculatePolicyProbability(index);
	//				ndBrainFloat logProb = ndBrainFloat(ndLog(prob));
	//				minQ -= m_parameters.m_entropyRegularizerCoef * logProb;
	//			}
	//			m_expectedRewards[j + base] = minQ;
	//		}
	//	}
	//});
	//ndBrainThreadPool::ParallelExecute(ExpectedRewards);

	const ndBrain& brain = **m_policyTrainer->GetBrain();
	m_nextActionBatch.SetCount(m_parameters.m_miniBatchSize * brain.GetOutputSize());
	m_nextObsevationsBatch.SetCount(m_parameters.m_miniBatchSize * brain.GetInputSize());
	for (ndInt32 i = 0; i < m_parameters.m_criticUpdatesCount; ++i)
	{
		for (ndInt32 j = 0; j < m_parameters.m_miniBatchSize; ++j)
		{
			const ndInt32 index = m_miniBatchIndexBuffer[i * m_parameters.m_miniBatchSize + j];
			ndBrainMemVector nextObsevations(&m_nextObsevationsBatch[j * brain.GetInputSize()], brain.GetInputSize());
			if (!m_replayBuffer.GetTerminalState(index))
			{
				nextObsevations.Set(ndBrainFloat(0.0f));
			} 
			else
			{
				const ndBrainMemVector nextReplyObservation(m_replayBuffer.GetNextObservations(index), brain.GetInputSize());
				nextObsevations.Set(nextReplyObservation);
			}
		}
		m_policyTrainer->MakePrediction(m_nextObsevationsBatch);
		m_policyTrainer->GetOutput(m_nextActionBatch);

		ndInt32 criticInputSize = brain.GetInputSize() + brain.GetOutputSize();
		m_criticNextObservationAction.SetCount(m_parameters.m_miniBatchSize * criticInputSize);
		for (ndInt32 j = 0; j < m_parameters.m_miniBatchSize; ++j)
		{
			const ndInt32 index = m_miniBatchIndexBuffer[i * m_parameters.m_miniBatchSize + j];
			const ndBrainMemVector nextAction(&m_nextActionBatch[j * brain.GetOutputSize()], brain.GetOutputSize());
			const ndBrainMemVector nextObsevations(&m_nextObsevationsBatch[j * brain.GetInputSize()], brain.GetInputSize());

			ndBrainMemVector criticNextObservationAction(&m_criticNextObservationAction[j * criticInputSize], criticInputSize);
			ndMemCpy(&criticNextObservationAction[0], &nextAction[0], brain.GetOutputSize());
			ndMemCpy(&criticNextObservationAction[brain.GetOutputSize()], &nextObsevations[0], brain.GetInputSize());
		}
		for (ndInt32 j = 0; j < sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0]); ++j)
		{
			m_referenceCriticTrainer[j]->MakePrediction(m_criticNextObservationAction);
		}
	}
}

////#pragma optimize( "", off )
//void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnPolicyFunction()
//{
//	ndAtomic<ndInt32> iterator(0);
//	ndInt32 base = 0;
//	for (ndInt32 n = m_parameters.m_policyUpdatesCount - 1; n >= 0; --n)
//	{
//		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
//		{
//			class ndLoss : public ndBrainLossLeastSquaredError
//			{
//				public:
//				ndLoss(ndBrainAgentDeterministicPolicyGradient_Trainer* const owner, const ndInt32 index)
//					:ndBrainLossLeastSquaredError(1)
//					,m_criticLoss()
//					,m_combinedInputGradients()
//					,m_combinedActionObservation()
//					,m_owner(owner)
//					,m_index(index)
//				{
//					m_combinedInputGradients.SetCount(m_owner->m_policy.GetInputSize() + m_owner->m_policy.GetOutputSize());
//					m_combinedActionObservation.SetCount(m_owner->m_policy.GetInputSize() + m_owner->m_policy.GetOutputSize());
//				}
//
//				class ndCriticLoss: public ndBrainLossLeastSquaredError
//				{
//					public:
//					ndCriticLoss()
//						:ndBrainLossLeastSquaredError(1)
//					{
//					}
//
//					void GetLoss(const ndBrainVector&, ndBrainVector& loss)
//					{
//						loss[0] = ndBrainFloat(-1.0f);
//					}
//				};
//
//				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
//				{
//					ndMemCpy(&m_combinedActionObservation[0], &output[0], m_owner->m_policy.GetOutputSize());
//					ndMemCpy(&m_combinedActionObservation[m_owner->m_policy.GetOutputSize()], m_owner->m_replayBuffer.GetObservations(m_index), m_owner->m_policy.GetInputSize());
//
//					m_owner->m_critic[0].CalculateInputGradient(m_combinedActionObservation, m_combinedInputGradients, m_criticLoss);
//					ndMemCpy(&loss[0], &m_combinedInputGradients[0], loss.GetCount());
//				}
//
//				ndCriticLoss m_criticLoss;
//				ndBrainFixSizeVector<256> m_combinedInputGradients;
//				ndBrainFixSizeVector<256> m_combinedActionObservation;
//				ndBrainAgentDeterministicPolicyGradient_Trainer* m_owner;
//				ndInt32 m_index;
//			};
//
//			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
//			{
//				const ndInt32 index = m_miniBatchIndexBuffer[i + base];
//				ndBrainTrainer& trainer = *m_policyTrainers[i];
//				const ndBrainMemVector observation(m_replayBuffer.GetObservations(index), m_policy.GetInputSize());
//
//				ndLoss loss(this, index);
//				trainer.BackPropagate(observation, loss);
//			}
//		});
//
//		iterator = 0;
//		ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
//		m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
//
//		base += m_parameters.m_miniBatchSize;
//	}
//}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnPolicyFunction()
{
	ndAssert(0);
#if 0
	ndAtomic<ndInt32> iterator(0);
	ndInt32 base = 0;
	for (ndInt32 n = m_parameters.m_policyUpdatesCount - 1; n >= 0; --n)
	{
		auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class ndLoss : public ndBrainLossLeastSquaredError
			{
				public:
				ndLoss(ndBrainAgentDeterministicPolicyGradient_Trainer* const owner, const ndInt32 index)
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
					ndCriticLoss(ndBrainAgentDeterministicPolicyGradient_Trainer* const owner, ndInt32 index)
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

					ndBrainAgentDeterministicPolicyGradient_Trainer* m_owner;
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
						if (m_owner->m_parameters.m_usePerActionSigmas)
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
					loss.Scale(ndBrainFloat(-1.0f));
				}

				ndCriticLoss m_criticLoss;
				ndBrainAgentDeterministicPolicyGradient_Trainer* m_owner;
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
#endif
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::Optimize()
{
	CalculateExpectedRewards();
	//ndAssert(0);
	//for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
	//{
	//	LearnQvalueFunction(k);
	//}
	//
	//if (!ndPolycyDelayMod)
	//{
	//	LearnPolicyFunction();
	//	m_referencePolicy.SoftCopy(m_policy, m_parameters.m_policyMovingAverageFactor);
	//}
	//for (ndInt32 k = 0; k < sizeof(m_critic) / sizeof(m_critic[0]); ++k)
	//{
	//	m_referenceCritic[k].SoftCopy(m_critic[k], m_parameters.m_criticMovingAverageFactor);
	//}
	//ndPolycyDelayMod = (ndPolycyDelayMod + 1) % ND_TD3_POLICY_DELAY_MOD;
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