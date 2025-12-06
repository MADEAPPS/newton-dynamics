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
#include "ndBrainSaveLoad.h"
#include "ndBrainCpuContext.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainAgentPolicyGradientActivation.h"
#include "ndBrainAgentOnPolicyGradient_Trainer.h"

#define ND_POLICY_MIN_SIGMA_SQUARE					ndBrainFloat(0.01f)
#define ND_POLICY_MAX_SIGMA_SQUARE					ndBrainFloat(1.0f)
#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON		ndBrainFloat(0.2f)
#define ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE	ndBrainFloat(1.0e-3f)
#define ND_CONTINUE_PROXIMA_POLICY_ITERATIONS		10

ndBrainAgentOnPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_randomSeed = 47;
	m_numberOfHiddenLayers = 3;
	m_maxTrajectorySteps = 4096;
	m_batchTrajectoryCount = 100;
	m_hiddenLayersNumberOfNeurons = 128;

	m_useGpuBackend = true;
	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_policyIterations = ND_CONTINUE_PROXIMA_POLICY_ITERATIONS;
	m_criticValueIterations = 8;

	m_learnRate = ndBrainFloat(1.0e-4f);
	//m_learnRate = ndBrainFloat(1.0e-5f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_discountRewardFactor = ndBrainFloat(0.99f);
	m_entropyRegularizerCoef = ndBrainFloat(0.0f);
	m_minSigmaSquared = ND_POLICY_MIN_SIGMA_SQUARE;
	m_maxSigmaSquared = ND_POLICY_MAX_SIGMA_SQUARE;

	m_policyRegularizerType = m_ridge;
	m_criticRegularizerType = m_ridge;
}

ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::ndTrajectory()
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::ndTrajectory(ndInt32 actionsSize, ndInt32 obsevationsSize)
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
	Init(actionsSize, obsevationsSize);
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize * 2;
	m_obsevationsSize = obsevationsSize;
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::Clear(ndInt32 entry)
{
	m_reward[entry] = ndBrainFloat(0.0f);
	m_terminal[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::CopyFrom(ndInt32 entry, ndTrajectory& src, ndInt32 srcEntry)
{
	m_reward[entry] = src.m_reward[srcEntry];
	m_terminal[entry] = src.m_terminal[srcEntry];
	ndMemCpy(&m_actions[entry * m_actionsSize], &src.m_actions[srcEntry * m_actionsSize], m_actionsSize);
	ndMemCpy(&m_observations[entry * m_obsevationsSize], &src.m_observations[srcEntry * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&m_nextObservations[entry * m_obsevationsSize], &src.m_nextObservations[srcEntry * m_obsevationsSize], m_obsevationsSize);
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetCount() const
{
	return ndInt32(m_reward.GetCount());
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetCount(ndInt32 count)
{
	m_reward.SetCount(count);
	m_terminal.SetCount(count);
	m_actions.SetCount(count * m_actionsSize);
	m_observations.SetCount(count * m_obsevationsSize);
	m_nextObservations.SetCount(count * m_obsevationsSize);
}

ndBrainFloat ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetReward(ndInt32 entry) const
{
	return m_reward[entry];
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	m_reward[entry] = reward;
}

bool ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetTerminalState(ndInt32 entry) const
{
	return (m_terminal[entry] == 0.0f) ? true : false;
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	m_terminal[entry] = isTernimal ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
}

ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry)
{
	return &m_actions[entry * m_actionsSize];
}

const ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry) const
{
	return &m_actions[entry * m_actionsSize];
}

ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry)
{
	return &m_observations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry) const
{
	return &m_observations[entry * m_obsevationsSize];
}

ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry)
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry) const
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetRewardOffset() const
{
	return 0;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetTerminalOffset() const
{
	return GetRewardOffset() + 1;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetActionOffset() const
{
	return GetTerminalOffset() + 1;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetObsevationOffset() const
{
	return m_actionsSize + GetActionOffset();
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetNextObsevationOffset() const
{
	return m_obsevationsSize + GetObsevationOffset();
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetStride() const
{
	return m_obsevationsSize + GetNextObsevationOffset();
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetFlatArray(ndInt32 index, ndBrainVector& output) const
{
	output.SetCount(GetStride());
	output[GetRewardOffset()] = m_reward[index];
	output[GetTerminalOffset()] = m_terminal[index];
	ndMemCpy(&output[GetActionOffset()], &m_actions[index * m_actionsSize], m_actionsSize);
	ndMemCpy(&output[GetObsevationOffset()], &m_observations[index * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&output[GetNextObsevationOffset()], &m_nextObservations[index * m_obsevationsSize], m_obsevationsSize);
}

ndBrainAgentOnPolicyGradient_Agent::ndBrainAgentOnPolicyGradient_Agent(ndBrainAgentOnPolicyGradient_Trainer* const master)
	:ndBrainAgent()
	,m_trajectory()
	,m_normalDistribution()
	,m_owner(master)
	,m_isDead(false)
{
	m_trajectory.Init(master->m_parameters.m_numberOfActions, master->m_parameters.m_numberOfObservations);

	ndUnsigned32 seed = master->m_uniformDistribution.Generate();
	m_normalDistribution.Init(seed);
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

void ndBrainAgentOnPolicyGradient_Agent::SampleActions(ndBrainVector& actions)
{
	const ndInt32 size = ndInt32(actions.GetCount()) / 2;
	for (ndInt32 i = size - 1; i >= 0; --i)
	{
		ndBrainFloat sigma = actions[size + i];
		ndBrainFloat normalSample = m_normalDistribution();
		ndBrainFloat sample = ndBrainFloat(actions[i]) + normalSample * sigma;
		ndBrainFloat clippedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = clippedAction;
	}
}

void ndBrainAgentOnPolicyGradient_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainAgentOnPolicyGradient_Trainer* const owner = m_owner;

	const ndBrain* const policy = owner->GetPolicyNetwork();
	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), policy->GetOutputSize());
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), owner->m_parameters.m_numberOfObservations);
	
	GetObservation(&observation[0]);

	bool isdead = IsTerminal();
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, isdead);

	policy->MakePrediction(observation, actions);
	SampleActions(actions);
	ApplyActions(&actions[0]);
	m_isDead = m_isDead || isdead;
}

ndBrainAgentOnPolicyGradient_Trainer::ndBrainAgentOnPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndClassAlloc()
	,m_name()
	,m_parameters(parameters)
	,m_context()
	,m_policyTrainer(nullptr)
	,m_criticTrainer(nullptr)
	,m_uniformDistribution()
	,m_agents()
	,m_trainingBuffer(nullptr)
	,m_advantageBuffer(nullptr)
	,m_policyActionBuffer(nullptr)
	,m_invLikelihoodBuffer(nullptr)
	,m_randomShuffleBuffer(nullptr)
	,m_policyGradientAccumulator(nullptr)
	,m_advantageMinibatchBuffer(nullptr)
	,m_invMinibatchLikelihoodBuffer(nullptr)
	,m_minibatchLikelihoodRatioBuffer(nullptr)
	,m_minuminMiniBatchClipRatioBuffer(nullptr)
	,m_maximunMiniBatchClipRatioBuffer(nullptr)
	,m_lastPolicy()
	,m_scratchBuffer()
	,m_shuffleBuffer()
	,m_trajectoryAccumulator()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_learnRate(m_parameters.m_learnRate)
	,m_frameCount(0)
	,m_horizonSteps(0)
	,m_eposideCount(0)
	,m_trajectiesCount(0)
	,m_numberOfGpuTransitions(0)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);

	ndSetRandSeed(m_parameters.m_randomSeed);
	m_uniformDistribution.Init(ndRandInt());

	m_trajectoryAccumulator.Init(m_parameters.m_numberOfActions, m_parameters.m_numberOfObservations);
	m_parameters.m_discountRewardFactor = ndClamp(m_parameters.m_discountRewardFactor, ndBrainFloat(0.1f), ndBrainFloat(0.999f));

	if (m_parameters.m_useGpuBackend)
	{
		m_context = ndSharedPtr<ndBrainContext>(new ndBrainGpuContext);
	}
	else
	{
		m_context = ndSharedPtr<ndBrainContext>(new ndBrainCpuContext);
	}

	m_parameters.m_policyIterations = ndClamp (m_parameters.m_policyIterations, 0, ND_CONTINUE_PROXIMA_POLICY_ITERATIONS);
	
	ndFloat32 gain = ndFloat32(1.0f);
	ndFloat32 maxGain = ndFloat32(0.99f) / (ndFloat32(1.0f) - m_parameters.m_discountRewardFactor);
	for (ndInt32 i = 0; (i < m_parameters.m_maxTrajectorySteps / 4) && (gain < maxGain); ++i)
	{
		gain = ndFloat32(1.0f) + m_parameters.m_discountRewardFactor * gain;
		m_horizonSteps++;
	}

	// create actor class
	BuildPolicyClass();
	BuildCriticClass();

	m_criticStateValue = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_advantageMinibatchBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_invMinibatchLikelihoodBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchLikelihoodRatioBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_randomShuffleMinibatchBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minuminMiniBatchClipRatioBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_maximunMiniBatchClipRatioBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));

	m_meanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_zMeanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigma2Buffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_meanGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));

	m_policyGradientAccumulator = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_policyTrainer->GetWeightAndBiasGradientBuffer()));
	m_advantageBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_invLikelihoodBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));

	m_trainingBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_trajectoryAccumulator.GetStride() * (m_parameters.m_batchTrajectoryCount + 1) * m_parameters.m_maxTrajectorySteps));
	m_policyActionBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, 2 * m_parameters.m_numberOfActions * m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_randomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_criticValueIterations * m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
}

ndBrain* ndBrainAgentOnPolicyGradient_Trainer::GetPolicyNetwork()
{
	return *m_policyTrainer->GetBrain();
}

void ndBrainAgentOnPolicyGradient_Trainer::AddAgent(ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>& agent)
{
	m_agents.Append(agent);
	agent->m_owner = this;
}

const ndString& ndBrainAgentOnPolicyGradient_Trainer::GetName() const
{
	return m_name;
}

void ndBrainAgentOnPolicyGradient_Trainer::SetName(const ndString& name)
{
	m_name = name;
}

void ndBrainAgentOnPolicyGradient_Trainer::BuildPolicyClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions * 2));
	layers.PushBack(new ndBrainAgentPolicyGradientActivation(layers[layers.GetCount() - 1]->GetOutputSize(), ndSqrt(m_parameters.m_minSigmaSquared), ndSqrt(m_parameters.m_maxSigmaSquared)));

	ndSharedPtr<ndBrain> policy (new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		policy->AddLayer(layers[i]);
	}
	policy->InitWeights();

	ndSharedPtr<ndBrainOptimizer> optimizer(new ndBrainOptimizerAdam(m_context));
	optimizer->SetRegularizer(m_parameters.m_policyRegularizer);
	optimizer->SetRegularizerType(m_parameters.m_policyRegularizerType);

	ndTrainerDescriptor descriptor(policy, m_context, m_parameters.m_miniBatchSize);
	m_policyTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor, optimizer));
}

void ndBrainAgentOnPolicyGradient_Trainer::BuildCriticClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	const ndBrain& policy = **m_policyTrainer->GetBrain();

	layers.SetCount(0);

	layers.PushBack(new ndBrainLayerLinear(policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers - 1; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	layers.PushBack(new ndBrainLayerActivationLeakyRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	
	ndSharedPtr<ndBrain> critic(new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		critic->AddLayer(layers[i]);
	}
	critic->InitWeights();
	
	ndSharedPtr<ndBrainOptimizer> optimizer(new ndBrainOptimizerAdam(m_context));
	optimizer->SetRegularizer(m_parameters.m_criticRegularizer);
	optimizer->SetRegularizerType(m_parameters.m_criticRegularizerType);

	ndTrainerDescriptor descriptor(critic, m_context, m_parameters.m_miniBatchSize);
	m_criticTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor, optimizer));
}

bool ndBrainAgentOnPolicyGradient_Trainer::IsValid() const
{
	return m_context->IsValid();
}

bool ndBrainAgentOnPolicyGradient_Trainer::IsSampling() const
{
	return false;
}

ndUnsigned32 ndBrainAgentOnPolicyGradient_Trainer::GetFramesCount() const
{
	return m_frameCount;
}

ndUnsigned32 ndBrainAgentOnPolicyGradient_Trainer::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentOnPolicyGradient_Trainer::GetAverageScore() const
{
	ndBrainFloat maxScore = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_parameters.m_discountRewardFactor);
	ndBrainFloat score = ndBrainFloat(1.0f) * m_averageExpectedRewards.GetAverage() / maxScore;
	return score;
}

ndFloat32 ndBrainAgentOnPolicyGradient_Trainer::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

void ndBrainAgentOnPolicyGradient_Trainer::SaveTrajectory(ndBrainAgentOnPolicyGradient_Agent* const agent)
{
	ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = agent->m_trajectory;

	// if the agent is dead, then remove all transtions past the last dead step.
	if (agent->m_isDead)
	{
		ndInt32 start = ndMax(0, ndInt32(trajectory.GetCount() - 64));
		for (ndInt32 i = start; i < trajectory.GetCount(); ++i)
		{
			if (trajectory.GetTerminalState(i))
			{
				trajectory.SetCount(i + 1);
				break;
			}
		}
	}

	for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
	{
		ndMemCpy(trajectory.GetNextObservations(i), trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations);
	}

	if (trajectory.GetCount() >= m_parameters.m_maxTrajectorySteps)
	{
		trajectory.SetCount(m_parameters.m_maxTrajectorySteps);
		trajectory.SetTerminalState(m_parameters.m_maxTrajectorySteps - 1, true);
	}

	ndInt32 base = m_trajectoryAccumulator.GetCount();
	m_trajectoryAccumulator.SetCount(m_trajectoryAccumulator.GetCount() + trajectory.GetCount());
	ndAssert(m_trajectoryAccumulator.GetCount() <= (m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));

	for (ndInt32 i = 0; i < trajectory.GetCount(); ++i)
	{
		m_trajectoryAccumulator.CopyFrom(base + i, trajectory, i);
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::UpdateScore()
{
	// using the Bellman equation to calculate trajectory expected rewards score.
	// (Monte Carlo method)
	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	ndBrainFloat expectedReward = m_trajectoryAccumulator.GetReward(m_trajectoryAccumulator.GetCount() - 1);
	for (ndInt32 i = m_trajectoryAccumulator.GetCount() - 2; i >= 0; --i)
	{
		ndBrainFloat teminate = m_trajectoryAccumulator.GetTerminalState(i) ? ndBrainFloat(0.0) : ndBrainFloat(1.0f);
		expectedReward = m_trajectoryAccumulator.GetReward(i) + expectedReward * teminate * m_parameters.m_discountRewardFactor;
		averageSum += expectedReward;
	}
	m_averageExpectedRewards.Update(averageSum / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(m_trajectoryAccumulator.GetCount()) / ndBrainFloat(m_parameters.m_batchTrajectoryCount));
}

void ndBrainAgentOnPolicyGradient_Trainer::TrajectoryToGpuBuffers()
{
	const ndInt32 stride = m_trajectoryAccumulator.GetStride();
	if (m_trajectoryAccumulator.GetCount() < m_parameters.m_miniBatchSize)
	{
		const ndInt32 transtionsCount = m_trajectoryAccumulator.GetCount();
		m_trajectoryAccumulator.SetCount(m_parameters.m_miniBatchSize);

		ndInt32 modIndex = 0;
		for (ndInt32 i = transtionsCount; i < m_parameters.m_miniBatchSize; ++i)
		{
			m_trajectoryAccumulator.CopyFrom(i, m_trajectoryAccumulator, modIndex);
			modIndex = (modIndex + 1) % transtionsCount;
		}
	}

	const ndInt32 count = m_trajectoryAccumulator.GetCount();
	m_scratchBuffer.SetCount(count * stride);
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndBrainMemVector dst(&m_scratchBuffer[i * stride], stride);
		m_trajectoryAccumulator.GetFlatArray(i, dst);
	}

	m_numberOfGpuTransitions = ndUnsigned32 (m_trajectoryAccumulator.GetCount() - m_trajectoryAccumulator.GetCount() % m_parameters.m_miniBatchSize);

	m_tmpShuffleBuffer.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32(m_trajectoryAccumulator.GetCount()); ++i)
	{
		m_tmpShuffleBuffer.PushBack(i);
	}
	if (m_tmpShuffleBuffer.GetCount() < ndInt32(m_parameters.m_criticValueIterations * m_parameters.m_miniBatchSize))
	{
		ndInt32 duplicate = 0;
		for (ndInt32 i = ndInt32(m_tmpShuffleBuffer.GetCount()); i < ndInt32(m_parameters.m_criticValueIterations * m_parameters.m_miniBatchSize); ++i)
		{
			m_tmpShuffleBuffer.PushBack(m_tmpShuffleBuffer[duplicate]);
			duplicate++;
		}
	}
	m_tmpShuffleBuffer.RandomShuffle(m_tmpShuffleBuffer.GetCount());
	m_tmpShuffleBuffer.SetCount(m_parameters.m_criticValueIterations * m_parameters.m_miniBatchSize);

	m_shuffleBuffer.SetCount(0);
	for (ndInt32 i = 0; i < m_parameters.m_criticValueIterations; ++i)
	{
		for (ndInt32 j = 0; j < ndInt32(m_parameters.m_criticValueIterations * m_parameters.m_miniBatchSize); ++j)
		{
			m_shuffleBuffer.PushBack(m_tmpShuffleBuffer[j]);
		}
		m_tmpShuffleBuffer.RandomShuffle(m_tmpShuffleBuffer.GetCount());
	}
	
	m_trainingBuffer->VectorToDevice(m_scratchBuffer);
	m_randomShuffleBuffer->MemoryToDevice(0, m_shuffleBuffer.GetCount() * sizeof(ndInt32), &m_shuffleBuffer[0]);
}

void ndBrainAgentOnPolicyGradient_Trainer::CalculateAdvantage()
{
	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();

	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	advantageInfo.m_srcOffsetInByte = 0;
	advantageInfo.m_dstOffsetInByte = 0;
	advantageInfo.m_dstStrideInByte = advantageInfo.m_srcStrideInByte;
	advantageInfo.m_strideInByte = advantageInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32 (m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_strideInByte = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo nextObservationInfo(observationInfo);
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));

	ndCopyBufferCommandInfo stateRewardInfo(observationInfo);
	stateRewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	stateRewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	stateRewardInfo.m_strideInByte = stateRewardInfo.m_dstStrideInByte;
	
	ndCopyBufferCommandInfo stateTerminalInfo(stateRewardInfo);
	stateTerminalInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetTerminalOffset() * sizeof(ndReal));

	const ndInt32 advantageStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	const ndInt32 numberOfGpuTransitions = ndInt32(m_numberOfGpuTransitions / m_parameters.m_miniBatchSize);
	const ndInt32 transitionStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));

	// calculate GAE(l, 1) // very noisy, the policy colapse most of the time.
	// calculate GAE(l, 0) // too smooth, and doesn't seem to work either
	// just using bellman equation to calculate state expected reward.
	// advantage(i) = reward(i) + alive(i) * (gamma * Value(i + 1) - value(i))
	for (ndInt32 i = numberOfGpuTransitions - 1; i >= 0; --i)
	{
		// get next state value
		inputBuffer->CopyBuffer(nextObservationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		
		outputBuffer->Scale(m_parameters.m_discountRewardFactor);
		m_advantageMinibatchBuffer->Set(*outputBuffer);
		
		inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		m_advantageMinibatchBuffer->Sub(*outputBuffer);

		outputBuffer->CopyBuffer(stateTerminalInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_advantageMinibatchBuffer->Mul(*outputBuffer);
		
		outputBuffer->CopyBuffer(stateRewardInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_advantageMinibatchBuffer->Add(*outputBuffer);
		
		// save advantage
		m_advantageBuffer->CopyBuffer(advantageInfo, 1, **m_advantageMinibatchBuffer);

		advantageInfo.m_dstOffsetInByte += advantageStrideInBytes;
		stateRewardInfo.m_srcOffsetInByte += transitionStrideInBytes;
		observationInfo.m_srcOffsetInByte += transitionStrideInBytes;
		stateTerminalInfo.m_srcOffsetInByte += transitionStrideInBytes;
		nextObservationInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizeCritic()
{
	// calculate value function by bootstrapping the trajectory transtions.
	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_strideInByte = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo rewardInfo;
	rewardInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	rewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	rewardInfo.m_dstOffsetInByte = 0;
	rewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	rewardInfo.m_strideInByte = rewardInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo terminalInfo(rewardInfo);
	terminalInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetTerminalOffset() * sizeof(ndReal));

	ndCopyBufferCommandInfo observationInfo(rewardInfo);
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_strideInByte = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo nextObservationInfo(observationInfo);
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));

	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const outputGradientBuffer = m_criticTrainer->GetOuputGradientBuffer();

	const ndInt32 numberOfUpdates = ndInt32(m_shuffleBuffer.GetCount() / m_parameters.m_miniBatchSize);

	// calculate GAE(l, 1) // very noisy, the policy colapse most of the time.
	// calculate GAE(l, 0) // too smooth, and doesn't seem to work either
	// just using bellman equation to calculate state state value.
	// gradValue(i) = 0.5 * (value(i) - reward(i) + alive(i) * gamma * Value(i + 1))^2
	for (ndInt32 i = numberOfUpdates - 1; i >= 0; --i)
	{
		m_randomShuffleMinibatchBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);

		inputBuffer->CopyBufferIndirect(nextObservationInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		outputBuffer->Scale(m_parameters.m_discountRewardFactor);

		outputGradientBuffer->CopyBufferIndirect(terminalInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		outputBuffer->Mul(*outputGradientBuffer);
		
		m_criticStateValue->CopyBufferIndirect(rewardInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		m_criticStateValue->Add(*outputBuffer);
		
		inputBuffer->CopyBufferIndirect(observationInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();

		outputGradientBuffer->Set(*outputBuffer);
		outputGradientBuffer->Sub(**m_criticStateValue);
		
		m_criticTrainer->BackPropagate();
		m_criticTrainer->ApplyLearnRate(m_learnRate);
		
		shuffleBufferInfo.m_srcOffsetInByte += ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	}
}

//#pragma optimize( "", off )
void ndBrainAgentOnPolicyGradient_Trainer::OptimizePolicy()
{
	m_policyGradientAccumulator->Set(ndBrainFloat(0.0f));
	ndBrainFloatBuffer* const weightAndBiasGradientBuffer = m_policyTrainer->GetWeightAndBiasGradientBuffer();

	ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();

	ndBrainFloatBuffer* const policyMinibatchLikelihood = *m_advantageMinibatchBuffer;

	ndCopyBufferCommandInfo policyObservationInfo;
	policyObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	policyObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	policyObservationInfo.m_dstOffsetInByte = 0;
	policyObservationInfo.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	policyObservationInfo.m_strideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));

	ndCopyBufferCommandInfo sampledActions(policyObservationInfo);
	sampledActions.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetActionOffset() * sizeof(ndReal));
	sampledActions.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));
	sampledActions.m_strideInByte = sampledActions.m_dstStrideInByte;

	ndCopyBufferCommandInfo copyMeanActions;
	copyMeanActions.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	copyMeanActions.m_srcOffsetInByte = 0;
	copyMeanActions.m_dstOffsetInByte = 0;
	copyMeanActions.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	copyMeanActions.m_strideInByte = sizeof(ndReal);

	ndCopyBufferCommandInfo copySigmaActions(copyMeanActions);
	copySigmaActions.m_srcOffsetInByte = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcStrideInByte = ndInt32(sizeof(ndReal));
	advantageInfo.m_srcOffsetInByte = 0;
	advantageInfo.m_dstOffsetInByte = 0;
	advantageInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	advantageInfo.m_strideInByte = advantageInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo likelihoodInfo(advantageInfo);

	ndCopyBufferCommandInfo policyActionInfo;
	policyActionInfo.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_srcOffsetInByte = 0;
	policyActionInfo.m_dstOffsetInByte = 0;
	policyActionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_strideInByte = policyActionInfo.m_dstStrideInByte;

	const ndInt32 advantageStrideInByte = m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));
	const ndInt32 transitionStrideInBytes = m_trajectoryAccumulator.GetStride() * m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));
	const ndInt32 policyActionStrideInByte = 2 * m_parameters.m_numberOfActions * m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));

	const ndInt32 numberOfBatches = ndInt32(m_numberOfGpuTransitions / m_parameters.m_miniBatchSize);
	for (ndInt32 i = numberOfBatches - 1; i >= 0; --i)
	{
		policyMinibatchInputBuffer->CopyBuffer(policyObservationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();

		m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
		m_sigmaBuffer->CopyBuffer(copySigmaActions, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

		m_zMeanBuffer->CopyBuffer(sampledActions, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_zMeanBuffer->Sub(**m_meanBuffer);

		if (m_parameters.m_policyIterations)
		{
			// calculate base probability and precompute some KL divergence values.
			m_policyActionBuffer->CopyBuffer(policyActionInfo, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
			 
			policyMinibatchLikelihood->CalculateLikelihood(**m_zMeanBuffer, **m_sigmaBuffer);
			policyMinibatchLikelihood->Reciprocal(*policyMinibatchLikelihood);
			m_invLikelihoodBuffer->CopyBuffer(likelihoodInfo, m_parameters.m_miniBatchSize, *policyMinibatchLikelihood);
		}

		// calculate gradient
		policyMinibatchOutputGradientBuffer->CalculateEntropyRegularizationGradient(**m_zMeanBuffer, **m_sigmaBuffer, ndBrainFloat(1.0f), m_parameters.m_numberOfActions);
		m_advantageMinibatchBuffer->CopyBuffer(advantageInfo, m_parameters.m_miniBatchSize, **m_advantageBuffer);

		// get a advantage minibatch and make gradient accend 
		policyMinibatchOutputBuffer->BroadcastScaler(**m_advantageMinibatchBuffer);
		policyMinibatchOutputGradientBuffer->Mul(*policyMinibatchOutputBuffer);
		policyMinibatchOutputGradientBuffer->Scale(ndReal(-1.0f));
		
		m_policyTrainer->BackPropagate();
		m_policyGradientAccumulator->Add(*weightAndBiasGradientBuffer);

		advantageInfo.m_srcOffsetInByte += advantageStrideInByte;
		likelihoodInfo.m_dstOffsetInByte += advantageStrideInByte;
		sampledActions.m_srcOffsetInByte += transitionStrideInBytes;
		policyActionInfo.m_dstOffsetInByte += policyActionStrideInByte;
		policyObservationInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}
	m_policyGradientAccumulator->Scale(ndBrainFloat(1.0f) / ndBrainFloat(numberOfBatches));
	weightAndBiasGradientBuffer->Set(**m_policyGradientAccumulator);
	m_policyTrainer->ApplyLearnRate(m_learnRate);
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizedSurrogatePolicy()
{
	m_policyGradientAccumulator->Set(ndBrainFloat(0.0f));
	ndBrainFloatBuffer* const weightAndBiasGradientBuffer = m_policyTrainer->GetWeightAndBiasGradientBuffer();

	ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();

	ndCopyBufferCommandInfo policyObservationInfo;
	policyObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	policyObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	policyObservationInfo.m_dstOffsetInByte = 0;
	policyObservationInfo.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	policyObservationInfo.m_strideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));

	ndCopyBufferCommandInfo sampledActions(policyObservationInfo);
	sampledActions.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetActionOffset() * sizeof(ndReal));
	sampledActions.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));
	sampledActions.m_strideInByte = sampledActions.m_dstStrideInByte;

	ndCopyBufferCommandInfo copyMeanActions;
	copyMeanActions.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	copyMeanActions.m_srcOffsetInByte = 0;
	copyMeanActions.m_dstOffsetInByte = 0;
	copyMeanActions.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	copyMeanActions.m_strideInByte = sizeof(ndReal);

	ndCopyBufferCommandInfo copySigmaActions(copyMeanActions);
	copySigmaActions.m_srcOffsetInByte = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcStrideInByte = ndInt32(sizeof(ndReal));
	advantageInfo.m_srcOffsetInByte = 0;
	advantageInfo.m_dstOffsetInByte = 0;
	advantageInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	advantageInfo.m_strideInByte = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo likelihoodInfo;
	likelihoodInfo.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	likelihoodInfo.m_srcOffsetInByte = 0;
	likelihoodInfo.m_dstOffsetInByte = 0;
	likelihoodInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	likelihoodInfo.m_strideInByte = likelihoodInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo policyActionInfo;
	policyActionInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	policyActionInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetActionOffset() * sizeof(ndReal));
	policyActionInfo.m_dstOffsetInByte = 0;
	policyActionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_strideInByte = policyActionInfo.m_dstStrideInByte;

	const ndInt32 advantageStrideInByte = m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));
	const ndInt32 transitionStrideInBytes = m_trajectoryAccumulator.GetStride() * m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));
	const ndInt32 policyActionStrideInByte = 2 * m_parameters.m_numberOfActions * m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));
	const ndInt32 likelihoodStrideInByte = 2 * m_parameters.m_numberOfActions * m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));

	const ndInt32 numberOfBatches = ndInt32(m_numberOfGpuTransitions / m_parameters.m_miniBatchSize);
	for (ndInt32 i = numberOfBatches - 1; i >= 0; --i)
	{
		policyMinibatchInputBuffer->CopyBuffer(policyObservationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();

		m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
		m_sigmaBuffer->CopyBuffer(copySigmaActions, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

		m_zMeanBuffer->CopyBuffer(sampledActions, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_zMeanBuffer->Sub(**m_meanBuffer);

		// get this minibatch advantage
		m_advantageMinibatchBuffer->CopyBuffer(advantageInfo, m_parameters.m_miniBatchSize, **m_advantageBuffer);

		//calculate the clip surrugate factor
		m_minibatchLikelihoodRatioBuffer->CalculateLikelihood(**m_zMeanBuffer, **m_sigmaBuffer);
		m_invMinibatchLikelihoodBuffer->CopyBuffer(advantageInfo, m_parameters.m_miniBatchSize, **m_invLikelihoodBuffer);
		m_minibatchLikelihoodRatioBuffer->Mul(**m_invMinibatchLikelihoodBuffer);

		// using this vaible as blend factor
		m_invMinibatchLikelihoodBuffer->Set(**m_advantageMinibatchBuffer);
		m_invMinibatchLikelihoodBuffer->Max(ndBrainFloat(0.0f));

		m_minuminMiniBatchClipRatioBuffer->Set(**m_minibatchLikelihoodRatioBuffer);
		m_maximunMiniBatchClipRatioBuffer->Set(**m_minibatchLikelihoodRatioBuffer);

		m_minuminMiniBatchClipRatioBuffer->Min(ndBrainFloat(1.0f) + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
		m_maximunMiniBatchClipRatioBuffer->Max(ndBrainFloat(1.0f) - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);

		m_maximunMiniBatchClipRatioBuffer->Blend(**m_minuminMiniBatchClipRatioBuffer, **m_invMinibatchLikelihoodBuffer);

		// calculate gradient
		policyMinibatchOutputGradientBuffer->CalculateEntropyRegularizationGradient(**m_zMeanBuffer, **m_sigmaBuffer, ndBrainFloat(1.0f), m_parameters.m_numberOfActions);

		// get a advantage minibatch and make gradient accend 
		m_advantageMinibatchBuffer->Mul(**m_maximunMiniBatchClipRatioBuffer);
		policyMinibatchOutputBuffer->BroadcastScaler(**m_advantageMinibatchBuffer);
		policyMinibatchOutputGradientBuffer->Mul(*policyMinibatchOutputBuffer);
		policyMinibatchOutputGradientBuffer->Scale(ndReal(-1.0f));

		m_policyTrainer->BackPropagate();
		m_policyGradientAccumulator->Add(*weightAndBiasGradientBuffer);

		likelihoodInfo.m_dstOffsetInByte += likelihoodStrideInByte;
		advantageInfo.m_srcOffsetInByte += advantageStrideInByte;
		sampledActions.m_srcOffsetInByte += transitionStrideInBytes;
		policyActionInfo.m_srcOffsetInByte += transitionStrideInBytes;
		policyActionInfo.m_dstOffsetInByte += policyActionStrideInByte;
		policyObservationInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}
	m_policyGradientAccumulator->Scale(ndBrainFloat(1.0f) / ndBrainFloat(numberOfBatches));
	weightAndBiasGradientBuffer->Set(**m_policyGradientAccumulator);
	m_policyTrainer->ApplyLearnRate(m_learnRate);
}

#pragma optimize( "", off )
ndBrainFloat ndBrainAgentOnPolicyGradient_Trainer::CalculateKLdivergence()
{
	// https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	// since I am using a diagonal sigma, I do not have to use Cholesky 

	ndBrainFloatBuffer* const policyMinibatchDivergence = m_policyTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchBaseDivergence = m_policyTrainer->GetOuputGradientBuffer();

	ndSharedPtr<ndBrainFloatBuffer> minbatchDivergence(m_criticStateValue);
	ndSharedPtr<ndBrainFloatBuffer> minbatchDivergenceAcc(m_advantageMinibatchBuffer);

	ndCopyBufferCommandInfo policyObservationInfo;
	policyObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	policyObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	policyObservationInfo.m_dstOffsetInByte = 0;
	policyObservationInfo.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	policyObservationInfo.m_strideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));

	ndCopyBufferCommandInfo policyActionInfo;
	policyActionInfo.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_srcOffsetInByte = 0;
	policyActionInfo.m_dstOffsetInByte = 0;
	policyActionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_strideInByte = policyActionInfo.m_dstStrideInByte;

	const ndInt32 policyActionStrideInByte = 2 * m_parameters.m_numberOfActions * m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));
	const ndInt32 transitionStrideInBytes = m_trajectoryAccumulator.GetStride() * m_parameters.m_miniBatchSize * ndInt32(sizeof(ndReal));

//m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_lastPolicy);
//m_context->SyncBufferCommandQueue();
//m_policyTrainer->UpdateParameters(m_lastPolicy);
//const ndBrain* const policy = GetPolicyNetwork();

	minbatchDivergenceAcc->Set(ndBrainFloat(0.0f));
	const ndInt32 numberOfBatches = ndInt32(m_numberOfGpuTransitions / m_parameters.m_miniBatchSize);
	ndAssert((numberOfBatches * m_parameters.m_miniBatchSize) == ndInt32(m_numberOfGpuTransitions));
	for (ndInt32 i = numberOfBatches - 1; i >= 0; --i)
	{
		policyMinibatchInputBuffer->CopyBuffer(policyObservationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();
		policyMinibatchBaseDivergence->CopyBuffer(policyActionInfo, m_parameters.m_miniBatchSize, **m_policyActionBuffer);
		
		minbatchDivergence->CalculatePartialKlDivergence(*policyMinibatchBaseDivergence, *policyMinibatchDivergence);
		minbatchDivergenceAcc->Add(**minbatchDivergence);
		
		policyActionInfo.m_srcOffsetInByte += policyActionStrideInByte;
		policyObservationInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}

	minbatchDivergenceAcc->ReductionSum();
	ndBrainFloat divergence = minbatchDivergenceAcc->GetElement(0);
	return divergence / ndFloat32(m_numberOfGpuTransitions);
}

#pragma optimize( "", off )
void ndBrainAgentOnPolicyGradient_Trainer::Optimize()
{
	UpdateScore();
	TrajectoryToGpuBuffers();

	CalculateAdvantage();

	OptimizePolicy();
	if (m_parameters.m_policyIterations)
	{
		ndBrainFloat divergence = CalculateKLdivergence();
		ndInt32 xxx = 0;
		for (ndInt32 i = m_parameters.m_policyIterations - 1; (i >= 0) && (divergence < ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE); --i)
		{
			OptimizedSurrogatePolicy();
			divergence = CalculateKLdivergence();
			xxx++;
		}
		xxx *= 1;
	}

	OptimizeCritic();
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizeStep()
{
	for (ndList<ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentOnPolicyGradient_Agent* const agent = *node->GetInfo();
		ndAssert(agent->m_trajectory.GetCount());

		bool isTeminal = agent->m_isDead || (agent->m_trajectory.GetCount() >= ndInt32(m_parameters.m_maxTrajectorySteps + m_horizonSteps));
		if (isTeminal)
		{
			SaveTrajectory(agent);
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
			agent->m_isDead = false;
			m_trajectiesCount++;
		}
		m_frameCount++;
	}

	if (ndInt32 (m_trajectiesCount) >= m_parameters.m_batchTrajectoryCount)
	{
		Optimize();
		for (ndList<ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentOnPolicyGradient_Agent* const agent = *node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
			agent->m_isDead = false;
		}
		m_eposideCount++;
		m_trajectiesCount = 0;
		m_trajectoryAccumulator.SetCount(0);

		m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_lastPolicy);
		m_context->SyncBufferCommandQueue();
		m_policyTrainer->UpdateParameters(m_lastPolicy);
	}
}
