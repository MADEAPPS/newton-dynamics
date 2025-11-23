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

#define ND_POLICY_TRAINING_EXPLORATION_NOISE	ndBrainFloat(0.2f)
#define ND_POLICY_DEFAULT_POLYAK_BLEND			ndBrainFloat(0.005f)
#define ND_POLICY_CONSTANT_SIGMA				ndBrainFloat(0.5f)
#define ND_POLICY_MIN_SIGMA						(ndBrainFloat(0.5f) * ND_POLICY_CONSTANT_SIGMA)
#define ND_POLICY_MAX_SIGMA						(ndBrainFloat(2.0f) * ND_POLICY_CONSTANT_SIGMA)

#define ND_CONTINUE_PROXIMA_POLICY_ITERATIONS	0

ndBrainAgentOnPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_learnRate = ndBrainFloat(1.0e-5f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_discountRewardFactor = ndBrainFloat(0.995f);

	m_polyakBlendFactor = ND_POLICY_DEFAULT_POLYAK_BLEND;
	m_useGpuBackend = true;

	m_policyRegularizerType = m_ridge;
	m_criticRegularizerType = m_ridge;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;
	m_batchTrajectoryCount = 100;

	m_randomSeed = 42;
	m_numberOfHiddenLayers = 2;
	m_maxTrajectorySteps = 1024 * 4;
	m_replayBufferSize = 1024 * 1024;
	m_hiddenLayersNumberOfNeurons = 256;
	m_replayBufferStartOptimizeSize = 1024 * 64;
	m_maxNumberOfTrainingSteps = 1000000;

m_useGpuBackend = false;
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
	,m_expectedReward()
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
	m_expectedReward[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::CopyFrom(ndInt32 entry, ndTrajectory& src, ndInt32 srcEntry)
{
	m_reward[entry] = src.m_reward[srcEntry];
	m_terminal[entry] = src.m_terminal[srcEntry];
	m_expectedReward[entry] = src.m_expectedReward[srcEntry];
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
	m_expectedReward.SetCount(count);
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

ndBrainFloat ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetExpectedReward(ndInt32 entry) const
{
	return m_expectedReward[entry];
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetExpectedReward(ndInt32 entry, ndBrainFloat reward)
{
	m_expectedReward[entry] = reward;
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

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetExpectedRewardOffset() const
{
	return GetRewardOffset() + 1;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetTerminalOffset() const
{
	return GetExpectedRewardOffset() + 1;
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
	output[GetExpectedRewardOffset()] = m_expectedReward[index];
	ndMemCpy(&output[GetActionOffset()], &m_actions[index * m_actionsSize], m_actionsSize);
	ndMemCpy(&output[GetObsevationOffset()], &m_observations[index * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&output[GetNextObsevationOffset()], &m_nextObservations[index * m_obsevationsSize], m_obsevationsSize);
}

ndBrainAgentOnPolicyGradient_Agent::ndBrainAgentOnPolicyGradient_Agent(ndBrainAgentOnPolicyGradient_Trainer* const master)
	:ndBrainAgent()
	,m_trajectory()
	,m_randomeGenerator()
	,m_owner(master)
	,m_isDead(false)
{
	m_trajectory.Init(master->m_parameters.m_numberOfActions, master->m_parameters.m_numberOfObservations);

	ndUnsigned32 agentSeed = m_owner->m_randomGenerator();
	m_randomeGenerator.m_gen.seed(agentSeed);
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
		ndBrainFloat unitVarianceSample = m_randomeGenerator.m_d(m_randomeGenerator.m_gen);
		ndBrainFloat sample = ndBrainFloat(actions[i]) + unitVarianceSample * sigma;
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
	,m_randomGenerator(std::random_device{}())
	,m_uniformDistribution(ndFloat32(0.0f), ndFloat32(1.0f))
	,m_agents()
	,m_trainingBuffer(nullptr)
	,m_advantageBuffer(nullptr)
	,m_randomShuffleBuffer(nullptr)
	,m_policyGradientAccumulator(nullptr)
	,m_advantageMinibatchBuffer(nullptr)
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
	,m_numberOfMinibatches(0)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);

	m_randomGenerator.seed(m_parameters.m_randomSeed);
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
	
	ndFloat32 gain = ndFloat32(1.0f);
	ndFloat32 maxGain = ndFloat32(0.98f) / (ndFloat32(1.0f) - m_parameters.m_discountRewardFactor);
	for (ndInt32 i = 0; (i < m_parameters.m_maxTrajectorySteps / 4) && (gain < maxGain); ++i)
	{
		gain = ndFloat32(1.0f) + m_parameters.m_discountRewardFactor * gain;
		m_horizonSteps++;
	}

	// create actor class
	BuildPolicyClass();
	BuildCriticClass();

	m_advantageMinibatchBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_randomShuffleMinibatchBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));

	m_meanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigma2Buffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_meanDeviationBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_meanGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));

	m_criticStateValue = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_advantageBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_randomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_trainingBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_trajectoryAccumulator.GetStride() * m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_policyGradientAccumulator = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_policyTrainer->GetWeightAndBiasGradientBuffer()));
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

//void ndBrainAgentOnPolicyGradient_Trainer::SaveState(const char* const baseName)
void ndBrainAgentOnPolicyGradient_Trainer::SaveState(const char* const)
{
	ndAssert(0);
	//m_context->SyncBufferCommandQueue();
	//
	//char fileName[256];
	//snprintf(fileName, sizeof (fileName), "%s_policy.dnn", baseName);
	//m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
	//m_policyTrainer->UpdateParameters(m_scratchBuffer);
	//m_policyTrainer->GetBrain()->SaveToFile(fileName);
	//
	//for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	//{
	//	snprintf(fileName, sizeof (fileName), "%s_critic_%d.dnn", baseName, j);
	//	m_criticTrainer[j]->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
	//	m_criticTrainer[j]->UpdateParameters(m_scratchBuffer);
	//	m_criticTrainer[j]->GetBrain()->SaveToFile(fileName);
	//
	//	snprintf(fileName, sizeof (fileName), "%s_referenceCritic_%d.dnn", baseName, j);
	//	m_referenceCriticTrainer[j]->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
	//	m_referenceCriticTrainer[j]->UpdateParameters(m_scratchBuffer);
	//	m_referenceCriticTrainer[j]->GetBrain()->SaveToFile(fileName);
	//}
}

//void ndBrainAgentOnPolicyGradient_Trainer::RecoverState(const char* baseName)
void ndBrainAgentOnPolicyGradient_Trainer::RecoverState(const char*)
{
	ndAssert(0);
	//m_context->SyncBufferCommandQueue();
	//
	//char fileName[256];
	//snprintf(fileName, sizeof (fileName), "%s_policy.dnn", baseName);
	//ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));
	//ndTrainerDescriptor descriptor(policy, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
	//descriptor.m_regularizer = m_parameters.m_policyRegularizer;
	//descriptor.m_regularizerType = m_parameters.m_policyRegularizerType;
	//m_policyTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
	//
	//for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	//{
	//	snprintf(fileName, sizeof (fileName), "%s_critic_%d.dnn", baseName, j);
	//	ndSharedPtr<ndBrain> critic(ndBrainLoad::Load(fileName));
	//	ndTrainerDescriptor descriptorDescriptor(critic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
	//	descriptorDescriptor.m_regularizer = m_parameters.m_criticRegularizer;
	//	descriptorDescriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
	//	m_criticTrainer[j] = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptorDescriptor));
	//
	//	snprintf(fileName, sizeof (fileName), "%s_referenceCritic_%d.dnn", baseName, j);
	//	ndSharedPtr<ndBrain> referenceCritic(ndBrainLoad::Load(fileName));
	//	ndTrainerDescriptor referenceCriticDescriptor(referenceCritic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
	//	referenceCriticDescriptor.m_regularizer = m_parameters.m_criticRegularizer;
	//	referenceCriticDescriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
	//	m_referenceCriticTrainer[j] = ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainerInference(referenceCriticDescriptor));
	//}
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
	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers - 1; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	// prevent exploding gradients. 
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions * 2));
	layers.PushBack(new ndBrainAgentPolicyGradientActivation(layers[layers.GetCount() - 1]->GetOutputSize(), ndBrainFloat(ndLog(ND_POLICY_MIN_SIGMA)), ndBrainFloat(ndLog(ND_POLICY_MAX_SIGMA))));

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
	
	// prevent exploding gradients.
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

//void ndBrainAgentOnPolicyGradient_Trainer::CalculateScore()
//{
//	ndAssert(0);
//	ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
//	
//	// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
//	ndBrainFloat gamma = m_parameters.m_discountRewardFactor;
//	ndBrainFloat stateReward = trajectory.GetReward(trajectory.GetCount() - 1);
//	ndBrainFloat averageReward = stateReward;
//	for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
//	{
//		ndBrainFloat r = trajectory.GetReward(i);
//		stateReward = r + gamma * stateReward;
//		averageReward += stateReward;
//	}
//	ndInt32 numberOfSteps = trajectory.GetCount();
//	if (!trajectory.GetTerminalState(trajectory.GetCount() - 1))
//	{
//		numberOfSteps -= m_horizonSteps;
//		if (numberOfSteps < 100)
//		{
//			numberOfSteps = 100;
//		}
//		stateReward = trajectory.GetReward(trajectory.GetCount() - 1);
//		ndBrainFloat substractAverageReward = trajectory.GetReward(trajectory.GetCount() - 1);
//		for (ndInt32 i = trajectory.GetCount() - 2; i >= numberOfSteps; --i)
//		{
//			ndBrainFloat r = trajectory.GetReward(i);
//			stateReward = r + gamma * stateReward;
//			substractAverageReward += stateReward;
//		}
//		averageReward -= substractAverageReward;
//	}
//	averageReward /= ndBrainFloat(numberOfSteps);
//	m_averageExpectedRewards.Update(averageReward);
//	m_averageFramesPerEpisodes.Update(ndReal(trajectory.GetCount()));
//	
//	if (m_startOptimization)
//	{
//		m_eposideCount++;
//	}
//}

//void ndBrainAgentOnPolicyGradient_Trainer::SaveTrajectoryNoTerminal()
//{
//	ndAssert(0);
//	ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
//	
//	ndInt32 stride = trajectory.GetStride();
//	ndInt32 actionsSize = m_policyTrainer->GetBrain()->GetOutputSize();
//	ndInt32 observationSize = m_policyTrainer->GetBrain()->GetInputSize();
//	
//	ndInt32 numbeOfEntries = ndInt32(trajectory.GetCount() - m_agent->m_trajectoryBaseIndex - 1);
//	m_scratchBuffer.SetCount(numbeOfEntries * stride);
//	for (ndInt32 i = 0; i < numbeOfEntries; ++i)
//	{
//		ndInt32 index = ndInt32(m_agent->m_trajectoryBaseIndex + i);
//		ndBrainMemVector entry(&m_scratchBuffer[i * stride], stride);
//		entry[trajectory.GetRewardOffset()] = trajectory.GetReward(index + 1);
//		entry[trajectory.GetTerminalOffset()] = ndBrainFloat (1.0f) - trajectory.GetTerminalState(index);
//	
//		ndBrainMemVector action(&entry[trajectory.GetActionOffset()], actionsSize);
//		const ndBrainMemVector srcAction(trajectory.GetActions(index), actionsSize);
//		action.Set(srcAction);
//	
//		ndBrainMemVector observation(&entry[trajectory.GetObsevationOffset()], observationSize);
//		const ndBrainMemVector srcObservation(trajectory.GetObservations(index), observationSize);
//		observation.Set(srcObservation);
//	
//		ndBrainMemVector nextObservation(&entry[trajectory.GetNextObsevationOffset()], observationSize);
//		const ndBrainMemVector nextSrcObservation(trajectory.GetObservations(index + 1), observationSize);
//		nextObservation.Set(nextSrcObservation);
//	}
//	m_agent->m_trajectoryBaseIndex = ndUnsigned32 (trajectory.GetCount() - 1);
//}

//void ndBrainAgentOnPolicyGradient_Trainer::SaveTrajectoryTerminal()
//{
//	ndAssert(0);
//	ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
//	SaveTrajectoryNoTerminal();
//	
//	ndInt32 stride = trajectory.GetStride();
//	ndInt32 actionsSize = m_policyTrainer->GetBrain()->GetOutputSize();
//	ndInt32 observationSize = m_policyTrainer->GetBrain()->GetInputSize();
//	
//	ndInt32 base = ndInt32 (m_scratchBuffer.GetCount() / stride);
//	m_scratchBuffer.SetCount(m_scratchBuffer.GetCount() + stride);
//	
//	ndInt32 index = ndInt32(m_agent->m_trajectoryBaseIndex);
//	ndBrainMemVector entry(&m_scratchBuffer[base * stride], stride);
//	entry[trajectory.GetRewardOffset()] = trajectory.GetReward(index);
//	entry[trajectory.GetTerminalOffset()] = ndBrainFloat(1.0f) - trajectory.GetTerminalState(index);
//	
//	ndBrainMemVector action(&entry[trajectory.GetActionOffset()], actionsSize);
//	const ndBrainMemVector srcAction(trajectory.GetActions(index), actionsSize);
//	action.Set(srcAction);
//	
//	ndBrainMemVector observation(&entry[trajectory.GetObsevationOffset()], observationSize);
//	ndBrainMemVector nextObservation(&entry[trajectory.GetNextObsevationOffset()], observationSize);
//	const ndBrainMemVector srcObservation(trajectory.GetObservations(index), observationSize);
//	
//	observation.Set(srcObservation);
//	nextObservation.Set(srcObservation);
//}

//void ndBrainAgentOnPolicyGradient_Trainer::CacheTrajectoryTransitions()
//{
//	ndAssert(0);
//	ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
//	for (ndInt32 i = ndInt32(m_agent->m_trajectoryBaseIndex); i < trajectory.GetCount(); ++i)
//	{
//		if (trajectory.GetTerminalState(i))
//		{
//			trajectory.SetCount(i + 1);
//			CalculateScore();
//			SaveTrajectoryTerminal();
//	
//			m_agent->ResetModel();
//			trajectory.SetCount(0);
//			m_agent->m_trajectoryBaseIndex = 0;
//			m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
//			return;
//		}
//	}
//	SaveTrajectoryNoTerminal();
//	if (trajectory.GetCount() >= m_parameters.m_maxTrajectorySteps)
//	{
//		CalculateScore();
//		m_agent->ResetModel();
//		trajectory.SetCount(0);
//		m_agent->m_trajectoryBaseIndex = 0;
//		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
//	}
//}

void ndBrainAgentOnPolicyGradient_Trainer::SaveTrajectory(ndBrainAgentOnPolicyGradient_Agent* const agent)
{
	ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = agent->m_trajectory;

	// if the agen is dead, then remove all stape pass the las dead step.
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

	// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
	ndBrainFloat gamma = m_parameters.m_discountRewardFactor;
	ndBrainFloat expectedRewrad = trajectory.GetReward(trajectory.GetCount() - 1);
	trajectory.SetExpectedReward(trajectory.GetCount() - 1, expectedRewrad);
	for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
	{
		ndBrainFloat r0 = trajectory.GetReward(i);
		expectedRewrad = r0 + gamma * expectedRewrad;
		trajectory.SetExpectedReward(i, expectedRewrad);
	}

	for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
	{
		ndMemCpy(trajectory.GetNextObservations(i), trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations);
	}

	if (trajectory.GetTerminalState(trajectory.GetCount() - 1))
	{
		ndMemCpy(trajectory.GetNextObservations(trajectory.GetCount() - 1), trajectory.GetObservations(trajectory.GetCount() - 1), m_parameters.m_numberOfObservations);
	}

	if (trajectory.GetCount() >= m_parameters.m_maxTrajectorySteps)
	{
		ndAssert(0);
	}

	ndInt32 base = m_trajectoryAccumulator.GetCount();
	m_trajectoryAccumulator.SetCount(m_trajectoryAccumulator.GetCount() + trajectory.GetCount());
	for (ndInt32 i = 0; i < trajectory.GetCount(); ++i)
	{
		m_trajectoryAccumulator.CopyFrom(base + i, trajectory, i);
	}
}

//void ndBrainAgentOnPolicyGradient_Trainer::TrainCritics(ndInt32 criticIndex)
//{
//	ndAssert(0);
//	ndInt32 criticInputSize = m_policyTrainer->GetBrain()->GetInputSize() + m_policyTrainer->GetBrain()->GetOutputSize();
//	
//	ndBrainTrainer& critic = **m_criticTrainer[criticIndex];
//	const ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
//	
//	ndCopyBufferCommandInfo criticInputAction;
//	criticInputAction.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
//	criticInputAction.m_srcOffsetInByte = ndInt32(trajectory.GetActionOffset() * sizeof(ndReal));
//	criticInputAction.m_dstOffsetInByte = 0;
//	criticInputAction.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
//	criticInputAction.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
//	ndBrainFloatBuffer* const criticMinibatchInputBuffer = critic.GetInputBuffer();
//	criticMinibatchInputBuffer->CopyBuffer(criticInputAction, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
//	
//	ndCopyBufferCommandInfo criticInputObservation;
//	criticInputObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
//	criticInputObservation.m_srcOffsetInByte = ndInt32(trajectory.GetObsevationOffset() * sizeof(ndReal));
//	criticInputObservation.m_dstOffsetInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
//	criticInputObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
//	criticInputObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
//	criticMinibatchInputBuffer->CopyBuffer(criticInputObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
//	
//	critic.MakePrediction();
//	
//	// calculate loss
//	const ndBrainFloatBuffer* const criticMinibatchOutputBuffer = critic.GetOuputBuffer();
//
//
//	ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
//	criticMinibatchOutputGradientBuffer->Set(*criticMinibatchOutputBuffer);
//	criticMinibatchOutputGradientBuffer->Sub(**m_minibatchExpectedRewards);
//
////static ndBrainVector xxxx0;
////static ndBrainVector xxxx1;
////criticMinibatchOutputBuffer->VectorFromDevice(xxxx0);
////criticMinibatchOutputGradientBuffer->VectorFromDevice(xxxx1);
////ndBrainFloat maxxxx = 2.0f / (1.0f - m_parameters.m_discountRewardFactor);
////for (ndInt32 i = 0; i < xxxx0.GetCount(); ++i)
////{
////	ndBrainFloat q = xxxx0[i];
////	if (q > maxxxx)
////	{
////		xxxx0[i] *= 1.0f;
////	}
////}
//
//	//// using a Huber loss to prevent exploding gradient
//	//const ndBrainFloat huberSlope = ndBrainFloat(8.0f);
//	//criticMinibatchOutputGradientBuffer->Min(huberSlope);
//	//criticMinibatchOutputGradientBuffer->Max(-huberSlope);
//
////static ndBrainVector xxxx2;
////criticMinibatchOutputGradientBuffer->VectorFromDevice(xxxx2);
//
//	// back propagate loss
//	critic.BackPropagate();
//
//	// update parameters
//	critic.ApplyLearnRate();
//}


//void ndBrainAgentOnPolicyGradient_Trainer::CalculateExpectedRewards()
//{
//	// Get the rewards for this mini batch
//	ndBrainTrainerInference* const policy = *m_policyTrainer;
//
//	ndBrainFloatBuffer* const policyMinibatchInputBuffer = policy->GetInputBuffer();
//	
//
//	const ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
//	ndInt32 transitionStrideInBytes = ndInt32(trajectory.GetStride() * sizeof(ndReal));
//
//	const ndInt32 policyInputSize = ndInt32(policy->GetBrain()->GetInputSize());
//
//	ndCopyBufferCommandInfo policyNextObservation;
//	policyNextObservation.m_srcStrideInByte = transitionStrideInBytes;
//	policyNextObservation.m_srcOffsetInByte = ndInt32(trajectory.GetNextObsevationOffset() * sizeof(ndReal));
//	policyNextObservation.m_dstOffsetInByte = 0;
//	policyNextObservation.m_dstStrideInByte = policyInputSize * ndInt32(sizeof(ndReal));
//	policyNextObservation.m_strideInByte = policyInputSize * ndInt32(sizeof(ndReal));
//	policyMinibatchInputBuffer->CopyBuffer(policyNextObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
//	policy->MakePrediction();
//
//	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = policy->GetOuputBuffer();
//	const ndInt32 policyOutputSize = ndInt32(policy->GetBrain()->GetOutputSize());
//	const ndInt32 policyActionSize = policyOutputSize / 2;
//
//	ndCopyBufferCommandInfo minibatchMean;
//	minibatchMean.m_srcOffsetInByte = 0;
//	minibatchMean.m_srcStrideInByte = policyOutputSize * ndInt32(sizeof(ndReal));
//	minibatchMean.m_dstOffsetInByte = 0;
//	minibatchMean.m_dstStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//	minibatchMean.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//	m_minibatchMean->CopyBuffer(minibatchMean, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
//
//	ndCopyBufferCommandInfo minibatchSigma;
//	minibatchSigma.m_srcOffsetInByte = policyActionSize * ndInt32(sizeof(ndReal));
//	minibatchSigma.m_srcStrideInByte = policyOutputSize * ndInt32(sizeof(ndReal));
//	minibatchSigma.m_dstOffsetInByte = 0;
//	minibatchSigma.m_dstStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//	minibatchSigma.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//	m_minibatchSigma->CopyBuffer(minibatchSigma, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
//
//	// draw a sample action from the normal distribution and clip again the bounds
//	m_minibatchUniformRandomDistribution->StandardNormalDistribution();
//	m_minibatchUniformRandomDistribution->Mul(**m_minibatchSigma);
//	m_minibatchMean->Add(**m_minibatchUniformRandomDistribution);
//	m_minibatchMean->Min(ndBrainFloat(1.0f));
//	m_minibatchMean->Max(ndBrainFloat(-1.0f));
//
//	const ndInt32 criticInputSize = policy->GetBrain()->GetInputSize() + policy->GetBrain()->GetOutputSize();
//	for (ndInt32 i = 0; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
//	{
//		ndBrainTrainerInference& referenceCritic = **m_referenceCriticTrainer[i];
//		ndBrainFloatBuffer& criticInputBuffer = *referenceCritic.GetInputBuffer();
//
//		ndCopyBufferCommandInfo criticInputNextActionMean;
//		criticInputNextActionMean.m_srcOffsetInByte = 0;
//		criticInputNextActionMean.m_srcStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//		criticInputNextActionMean.m_dstOffsetInByte = 0;
//		criticInputNextActionMean.m_dstStrideInByte = criticInputSize * ndInt32(sizeof(ndReal));
//		criticInputNextActionMean.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//		criticInputBuffer.CopyBuffer(criticInputNextActionMean, m_parameters.m_miniBatchSize, **m_minibatchMean);
//
//		ndCopyBufferCommandInfo criticInputNextActionSigma;
//		criticInputNextActionSigma.m_srcOffsetInByte = 0;
//		criticInputNextActionSigma.m_srcStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//		criticInputNextActionSigma.m_dstOffsetInByte = policyActionSize * ndInt32(sizeof(ndReal));
//		criticInputNextActionSigma.m_dstStrideInByte = criticInputSize * ndInt32(sizeof(ndReal));
//		criticInputNextActionSigma.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
//		criticInputBuffer.CopyBuffer(criticInputNextActionSigma, m_parameters.m_miniBatchSize, **m_minibatchSigma);
//
//		ndCopyBufferCommandInfo criticInputNextObservation;
//		criticInputNextObservation.m_srcOffsetInByte = 0;
//		criticInputNextObservation.m_srcStrideInByte = policyInputSize * ndInt32(sizeof(ndReal));
//		criticInputNextObservation.m_dstStrideInByte = criticInputSize * ndInt32(sizeof(ndReal));
//		criticInputNextObservation.m_dstOffsetInByte = policyOutputSize * ndInt32(sizeof(ndReal));
//		criticInputNextObservation.m_strideInByte = policyInputSize * ndInt32(sizeof(ndReal));
//		criticInputBuffer.CopyBuffer(criticInputNextObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);
//		m_referenceCriticTrainer[i]->MakePrediction();
//	}
//
//	// get the smaller of the q values
//	ndBrainFloatBuffer& qValue = *m_referenceCriticTrainer[0]->GetOuputBuffer();
//	for (ndInt32 i = 1; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
//	{
//		const ndBrainFloatBuffer& qValue1 = *m_referenceCriticTrainer[i]->GetOuputBuffer();
//		qValue.Min(qValue1);
//	}
//
////static ndBrainVector xxxx;
////qValue.VectorFromDevice(xxxx);
////ndBrainFloat maxxxx = 1.0f / (1.0f - m_parameters.m_discountRewardFactor);
////for (ndInt32 i = 0; i < xxxx.GetCount(); ++i)
////{
////	ndBrainFloat q = xxxx[i];
////	if (q > maxxxx)
////	{
////		xxxx[i] *= 1.0f;
////	}
////}
//
//	ndCopyBufferCommandInfo criticOutputTerminal;
//	criticOutputTerminal.m_srcOffsetInByte = ndInt32(trajectory.GetTerminalOffset() * sizeof(ndReal));
//	criticOutputTerminal.m_srcStrideInByte = transitionStrideInBytes;
//	criticOutputTerminal.m_dstOffsetInByte = 0;
//	criticOutputTerminal.m_dstStrideInByte = ndInt32(sizeof(ndReal));
//	criticOutputTerminal.m_strideInByte = ndInt32(sizeof(ndReal));
//	m_minibatchNoTerminal->CopyBuffer(criticOutputTerminal, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
//
//	// calculate and add entropy regularization to the q value
//	m_minibatchEntropy->CalculateEntropyRegularization(**m_minibatchUniformRandomDistribution, **m_minibatchSigma, m_parameters.m_entropyTemperature);
//
//	qValue.Sub(**m_minibatchEntropy);
//	qValue.Mul(**m_minibatchNoTerminal);
//	qValue.Scale(m_parameters.m_discountRewardFactor);
//
//	ndCopyBufferCommandInfo criticOutputReward;
//	criticOutputReward.m_srcOffsetInByte = ndInt32(trajectory.GetRewardOffset() * sizeof(ndReal));
//	criticOutputReward.m_srcStrideInByte = transitionStrideInBytes;
//	criticOutputReward.m_dstOffsetInByte = 0;
//	criticOutputReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
//	criticOutputReward.m_strideInByte = ndInt32(sizeof(ndReal));
//	m_minibatchExpectedRewards->CopyBuffer(criticOutputReward, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
//	m_minibatchExpectedRewards->Add(qValue);
//}

//void ndBrainAgentOnPolicyGradient_Trainer::TrainPolicy()
//{
//	const ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
//	
//	ndBrainTrainerInference* const policy = *m_policyTrainer;
//	ndInt32 criticInputSize = policy->GetBrain()->GetInputSize() + policy->GetBrain()->GetOutputSize();
//	
//	ndBrainFloatBuffer* const policyMinibatchInputBuffer = policy->GetInputBuffer();
//	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = policy->GetOuputBuffer();
//	
//	ndCopyBufferCommandInfo policyObservation;
//	policyObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
//	policyObservation.m_srcOffsetInByte = ndInt32(trajectory.GetObsevationOffset() * sizeof(ndReal));
//	policyObservation.m_dstOffsetInByte = 0;
//	policyObservation.m_dstStrideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
//	policyObservation.m_strideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
//	policyMinibatchInputBuffer->CopyBuffer(policyObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
//	
//	policy->MakePrediction();
//	
//	ndInt32 meanOutputSizeInBytes = ndInt32 (policy->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
//	
//	ndCopyBufferCommandInfo minibatchMean;
//	minibatchMean.m_srcOffsetInByte = 0;
//	minibatchMean.m_srcStrideInByte = 2 * meanOutputSizeInBytes;
//	minibatchMean.m_dstOffsetInByte = 0;
//	minibatchMean.m_dstStrideInByte = meanOutputSizeInBytes;
//	minibatchMean.m_strideInByte = meanOutputSizeInBytes;
//	m_minibatchMean->CopyBuffer(minibatchMean, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
//	
//	ndCopyBufferCommandInfo minibatchSigma;
//	minibatchSigma.m_srcOffsetInByte = meanOutputSizeInBytes;
//	minibatchSigma.m_srcStrideInByte = 2 * meanOutputSizeInBytes;
//	minibatchSigma.m_dstOffsetInByte = 0;
//	minibatchSigma.m_dstStrideInByte = meanOutputSizeInBytes;
//	minibatchSigma.m_strideInByte = meanOutputSizeInBytes;
//	m_minibatchSigma->CopyBuffer(minibatchSigma, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
//	
//	// exploration is generated by sampling a normal distributed with zero mean and action varince
//	m_minibatchUniformRandomDistribution->StandardNormalDistribution();
//	m_minibatchUniformRandomDistribution->Mul(**m_minibatchSigma);
//	
//	// add noise to the actions mean and clip the result to the actions limits
//	m_minibatchMean->Add(**m_minibatchUniformRandomDistribution);
//	m_minibatchMean->Min(ndBrainFloat(1.0f));
//	m_minibatchMean->Max(ndBrainFloat(-1.0f));
//	
//	// using the critic with the lower gradient
//	for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
//	{
//		ndBrainTrainer& critic = **m_criticTrainer[i];
//		ndBrainFloatBuffer* const criticMinibatchInputBuffer = critic.GetInputBuffer();
//	
//		ndCopyBufferCommandInfo criticInputNextActionMean;
//		criticInputNextActionMean.m_srcOffsetInByte = 0;
//		criticInputNextActionMean.m_srcStrideInByte = meanOutputSizeInBytes;
//		criticInputNextActionMean.m_dstOffsetInByte = 0;
//		criticInputNextActionMean.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
//		criticInputNextActionMean.m_strideInByte = meanOutputSizeInBytes;
//		criticMinibatchInputBuffer->CopyBuffer(criticInputNextActionMean, m_parameters.m_miniBatchSize, **m_minibatchMean);
//	
//		ndCopyBufferCommandInfo criticInputNextActionSigma;
//		criticInputNextActionSigma.m_srcOffsetInByte = 0;
//		criticInputNextActionSigma.m_srcStrideInByte = meanOutputSizeInBytes;
//		criticInputNextActionSigma.m_dstOffsetInByte = meanOutputSizeInBytes;
//		criticInputNextActionSigma.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
//		criticInputNextActionSigma.m_strideInByte = meanOutputSizeInBytes;
//		criticMinibatchInputBuffer->CopyBuffer(criticInputNextActionSigma, m_parameters.m_miniBatchSize, **m_minibatchSigma);
//	
//		ndCopyBufferCommandInfo criticInputObservation;
//		criticInputObservation.m_srcOffsetInByte = 0;
//		criticInputObservation.m_srcStrideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
//		criticInputObservation.m_dstOffsetInByte = 2 * meanOutputSizeInBytes;
//		criticInputObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
//		criticInputObservation.m_strideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
//		criticMinibatchInputBuffer->CopyBuffer(criticInputObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);
//	
//		critic.MakePrediction();
//	
//		//the gradient is just 1.0
//		ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
//		criticMinibatchOutputGradientBuffer->Set(ndBrainFloat(1.0f));
//		critic.BackPropagate();
//	}
//	
//	ndBrainTrainer& critic = **m_criticTrainer[0];
//	ndBrainFloatBuffer* const criticMinibatchOutputBuffer = critic.GetOuputBuffer();
//	ndBrainFloatBuffer* const criticMinibatchInputGradientBuffer = critic.GetInputGradientBuffer();
//	for (ndInt32 i = 1; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
//	{
//		ndBrainTrainer& critic1 = **m_criticTrainer[i];
//		ndBrainFloatBuffer* const criticMinibatchOutputBuffer1 = critic1.GetOuputBuffer();
//		ndBrainFloatBuffer* const criticMinibatchInputGradientBuffer1 = critic1.GetInputGradientBuffer();
//		
//		// re using m_minibatchExpectedRewards because is not use anymore 
//		m_minibatchExpectedRewards->Set(*criticMinibatchOutputBuffer1);
//		m_minibatchExpectedRewards->LessEqual(*criticMinibatchOutputBuffer);
//		m_minibatchCriticInputTest->BroadcastScaler(**m_minibatchExpectedRewards);
//		
//		criticMinibatchOutputBuffer->Blend(*criticMinibatchOutputBuffer1, **m_minibatchExpectedRewards);
//		criticMinibatchInputGradientBuffer->Blend(*criticMinibatchInputGradientBuffer1, **m_minibatchCriticInputTest);
//	}
//	
//	// calculate entropy Regularization
//	ndCopyBufferCommandInfo policyCopyGradients;
//	policyCopyGradients.m_srcOffsetInByte = 0;
//	policyCopyGradients.m_srcStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
//	policyCopyGradients.m_dstOffsetInByte = 0;
//	policyCopyGradients.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));;
//	policyCopyGradients.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
//	policyMinibatchOutputBuffer->CopyBuffer(policyCopyGradients, m_parameters.m_miniBatchSize, *criticMinibatchInputGradientBuffer);
//	ndBrainFloatBuffer* const policyMinibatchOutputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();
//	policyMinibatchOutputGradientBuffer->CalculateEntropyRegularizationGradient(**m_minibatchUniformRandomDistribution, **m_minibatchSigma, m_parameters.m_entropyTemperature, ndInt32(meanOutputSizeInBytes / sizeof(ndReal)));
//	
//	// subtract the qValue gradient from the entropy gradient.
//	// The subtraction order is in reverse order, to get the gradient ascend.
//	policyMinibatchOutputGradientBuffer->Sub(*policyMinibatchOutputBuffer);
//	m_policyTrainer->BackPropagate();
//	
//	m_policyTrainer->ApplyLearnRate();
//}


void ndBrainAgentOnPolicyGradient_Trainer::UpdateScore()
{
	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
	for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator.GetExpectedReward(i);
	}
	m_averageExpectedRewards.Update(averageSum / ndBrainFloat(stepNumber));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(stepNumber) / ndBrainFloat(m_parameters.m_batchTrajectoryCount));
}

void ndBrainAgentOnPolicyGradient_Trainer::TrajectoryToGpuBuffers()
{
	ndInt32 stride = m_trajectoryAccumulator.GetStride();
	const ndInt32 count = m_trajectoryAccumulator.GetCount();

	m_shuffleBuffer.SetCount(0);
	m_scratchBuffer.SetCount(count * stride);
	for (ndInt32 i = 0; i < count; ++i)
	{
		m_shuffleBuffer.PushBack(i);
		ndBrainMemVector dst(&m_scratchBuffer[i * stride], stride);
		m_trajectoryAccumulator.GetFlatArray(i, dst);
	}

	m_shuffleBuffer.RandomShuffle(count);
	m_trainingBuffer->VectorToDevice(m_scratchBuffer);
	m_randomShuffleBuffer->MemoryToDevice(0, count * sizeof(ndInt32), &m_shuffleBuffer[0]);

	ndInt32 roundTransitions = m_trajectoryAccumulator.GetCount() - m_trajectoryAccumulator.GetCount() % m_parameters.m_miniBatchSize;
	m_numberOfMinibatches = ndUnsigned32(roundTransitions);
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

	ndCopyBufferCommandInfo expectedRewardInfo;
	expectedRewardInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	expectedRewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetExpectedRewardOffset() * sizeof(ndReal));
	expectedRewardInfo.m_dstOffsetInByte = 0;
	expectedRewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	expectedRewardInfo.m_strideInByte = expectedRewardInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32 (m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_strideInByte = observationInfo.m_dstStrideInByte;

	const ndInt32 dstStride = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	const ndInt32 srcStride = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	for (ndInt32 i = ndInt32 (m_numberOfMinibatches / m_parameters.m_miniBatchSize) - 1; i >= 0; --i)
	{
		m_advantageMinibatchBuffer->CopyBuffer(expectedRewardInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		m_advantageMinibatchBuffer->Sub(*outputBuffer);
		m_advantageBuffer->CopyBuffer(advantageInfo, 1, **m_advantageMinibatchBuffer);

		advantageInfo.m_dstOffsetInByte += dstStride;
		observationInfo.m_srcOffsetInByte += srcStride;
		expectedRewardInfo.m_srcOffsetInByte += srcStride;
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizeCritic()
{
	// claulte value function by booth strapping
	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_strideInByte = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo rewardInfo;
	rewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	rewardInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	rewardInfo.m_dstOffsetInByte = 0;
	rewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	rewardInfo.m_strideInByte = rewardInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo terminalInfo;
	terminalInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetTerminalOffset() * sizeof(ndReal));
	terminalInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	terminalInfo.m_dstOffsetInByte = 0;
	terminalInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	terminalInfo.m_strideInByte = terminalInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal)); 
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	observationInfo.m_strideInByte = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo nextObservationInfo(observationInfo);
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));

	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const outputGradientBuffer = m_criticTrainer->GetOuputGradientBuffer();
	for (ndInt32 i = ndInt32(m_numberOfMinibatches/ m_parameters.m_miniBatchSize) - 1; i >= 0; --i)
	{
		m_randomShuffleMinibatchBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);
		inputBuffer->CopyBufferIndirect(nextObservationInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();

		outputGradientBuffer->CopyBufferIndirect(terminalInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		outputBuffer->Scale(m_parameters.m_discountRewardFactor);
		outputBuffer->Mul(*outputGradientBuffer);

//ndBrainVector xxxx4;
//outputGradientBuffer->VectorFromDevice(xxxx4);

		m_criticStateValue->CopyBufferIndirect(rewardInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		m_criticStateValue->Add(*outputBuffer);

		//ndBrainVector xxxx4;
		//m_criticStateValue->VectorFromDevice(xxxx4);

		inputBuffer->CopyBufferIndirect(observationInfo, **m_randomShuffleMinibatchBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		
		outputGradientBuffer->Set(*outputBuffer);
		outputGradientBuffer->Sub(**m_criticStateValue);

		m_criticTrainer->BackPropagate();
		m_criticTrainer->ApplyLearnRate(m_learnRate);
		
		shuffleBufferInfo.m_srcOffsetInByte += ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizePolicy()
{
	m_policyGradientAccumulator->Set(ndBrainFloat(0.0f));
	ndBrainFloatBuffer* const weightAndBiasGradientBuffer = m_policyTrainer->GetWeightAndBiasGradientBuffer();
	ndBrainFloat scale = ndBrainFloat(0.0f);

	ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();

	ndCopyBufferCommandInfo policyObservation;
	policyObservation.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	//policyObservation.m_srcOffsetInByte = ndInt32(observationOffset * sizeof(ndReal));
	policyObservation.m_dstOffsetInByte = 0;
	policyObservation.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	policyObservation.m_strideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));

	ndCopyBufferCommandInfo copyMeanActions;
	copyMeanActions.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	copyMeanActions.m_srcOffsetInByte = 0;
	copyMeanActions.m_dstOffsetInByte = 0;
	copyMeanActions.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	copyMeanActions.m_strideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));

	ndCopyBufferCommandInfo sampledActions;
	sampledActions.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	//sampledActions.m_srcOffsetInByte = ndInt32(actionOffset * sizeof(ndReal));
	sampledActions.m_dstOffsetInByte = 0;
	sampledActions.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));
	sampledActions.m_strideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));

	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcStrideInByte = ndInt32(sizeof(ndReal));
	//advantageInfo.m_srcOffsetInByte = ndInt32(advantageOffset * sizeof(ndReal));
	advantageInfo.m_dstOffsetInByte = 0;
	advantageInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	advantageInfo.m_strideInByte = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo policyGradient;
	policyGradient.m_srcStrideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));
	policyGradient.m_srcOffsetInByte = 0;
	policyGradient.m_dstOffsetInByte = 0;
	policyGradient.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyGradient.m_strideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));

	ndInt32 advantageOffset = 0;
	ndInt32 actionOffset = m_trajectoryAccumulator.GetActionOffset();
	ndInt32 observationOffset = m_trajectoryAccumulator.GetObsevationOffset();
	for (ndInt32 j = ndInt32(m_numberOfMinibatches/ m_parameters.m_miniBatchSize) - 1; j >= 0; --j)
	{
		policyObservation.m_srcOffsetInByte = ndInt32(observationOffset * sizeof(ndReal));
		policyMinibatchInputBuffer->CopyBuffer(policyObservation, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();
		m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

		sampledActions.m_srcOffsetInByte = ndInt32(actionOffset * sizeof(ndReal));
		m_meanDeviationBuffer->CopyBuffer(sampledActions, m_parameters.m_miniBatchSize, **m_trainingBuffer);

		sampledActions.m_srcOffsetInByte = ndInt32((actionOffset + m_parameters.m_numberOfActions) * sizeof(ndReal));
		m_sigmaBuffer->CopyBuffer(sampledActions, m_parameters.m_miniBatchSize, **m_trainingBuffer);

		// calculate some intermediate values
		// ndBrainFloat sigma = probabilityDistribution[size + i];
		// ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
		m_invSigmaBuffer->Reciprocal(**m_sigmaBuffer);
		m_invSigma2Buffer->Set(**m_invSigmaBuffer);
		m_invSigma2Buffer->Mul(**m_invSigmaBuffer);

		// calculate deviation from the mean
		// ndBrainFloat meanDeviation = sampledProbability[i] - probabilityDistribution[i];
		m_meanDeviationBuffer->Sub(**m_meanBuffer);

		// calculate mean gradient
		// ndBrainFloat meanGrad = meanDeviation * invSigma * invSigma;
		m_meanGradiendBuffer->Set(**m_meanDeviationBuffer);
		m_meanGradiendBuffer->Mul(**m_invSigma2Buffer);

		// calculate sigma gradient gradient
		// ndBrainFloat sigmaGrad = meanDeviation * meanDeviation * invSigma * invSigma * invSigma - sigma;
		// ndBrainFloat sigmaGrad = meanDeviation * invSigma * meanGradient - sigma;
		m_sigmaGradiendBuffer->Set(**m_meanDeviationBuffer);
		m_sigmaGradiendBuffer->Mul(**m_invSigmaBuffer);
		m_sigmaGradiendBuffer->Mul(**m_meanGradiendBuffer);
		m_sigmaGradiendBuffer->Sub(**m_sigmaBuffer);
//m_sigmaGradiendBuffer->Scale(0.0);

		//ndBrainVector xxxx;
		//policyMinibatchOutputBuffer->VectorFromDevice(xxxx);
		//ndBrainVector xxxx1;
		//m_meanBuffer->VectorFromDevice(xxxx1);
		//ndBrainVector xxxx2;
		//m_sigmaBuffer->VectorFromDevice(xxxx2);
		//ndBrainVector xxxx4;
		//m_invSigmaBuffer->VectorFromDevice(xxxx4);

		// get a advantage minibatch and make gradient accend 
		advantageInfo.m_srcOffsetInByte = ndInt32(advantageOffset * sizeof(ndReal));
		m_advantageMinibatchBuffer->CopyBuffer(advantageInfo, m_parameters.m_miniBatchSize, **m_advantageBuffer);
		m_meanGradiendBuffer->BroadcastScaler(**m_advantageMinibatchBuffer);
		m_sigmaGradiendBuffer->BroadcastScaler(**m_advantageMinibatchBuffer);

		policyGradient.m_dstOffsetInByte = 0;
		policyMinibatchOutputGradientBuffer->CopyBuffer(policyGradient, m_parameters.m_miniBatchSize, **m_meanGradiendBuffer);

		policyGradient.m_dstOffsetInByte = sizeof(ndReal);
		policyMinibatchOutputGradientBuffer->CopyBuffer(policyGradient, m_parameters.m_miniBatchSize, **m_sigmaGradiendBuffer);

		policyMinibatchOutputGradientBuffer->Scale(ndReal(-1.0f));

		m_policyTrainer->BackPropagate();
		m_policyGradientAccumulator->Add(*weightAndBiasGradientBuffer);

		ndInt32 nextBatchSize = m_trajectoryAccumulator.GetStride() * m_parameters.m_miniBatchSize;

		scale += ndBrainFloat(1.0f);
		actionOffset += nextBatchSize;
		observationOffset += nextBatchSize;
		advantageOffset += m_parameters.m_miniBatchSize;
	}
	m_policyGradientAccumulator->Scale(1.0f / scale);
	weightAndBiasGradientBuffer->Set(m_policyGradientAccumulator);
	m_policyTrainer->ApplyLearnRate(m_learnRate);
}

void ndBrainAgentOnPolicyGradient_Trainer::Optimize()
{
	UpdateScore();
	TrajectoryToGpuBuffers();

	OptimizeCritic();

	CalculateAdvantage();
	OptimizePolicy();

	//ndBrainFloat divergence = CalculateKLdivergence();
	for (ndInt32 i = ND_CONTINUE_PROXIMA_POLICY_ITERATIONS - 1; i >= 0; --i)
	{
		ndAssert(0);
		//OptimizedSurrogate();
		//divergence = CalculateKLdivergence();
	}
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
		//// calculate aneling paremater
		//ndFloat64 num = ndFloat64(m_frameCount);
		//ndFloat64 den = ndFloat64(m_parameters.m_maxNumberOfTrainingSteps - m_parameters.m_replayBufferStartOptimizeSize);
		//ndBrainFloat param = ndBrainFloat((ndFloat64(1.0f) - ndClamp(num / den, ndFloat64(0.0f), ndFloat64(1.0f))));

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

