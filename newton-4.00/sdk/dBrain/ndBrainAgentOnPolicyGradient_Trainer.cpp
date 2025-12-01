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

#define ND_POLICY_MIN_SIGMA_SQUARE				ndBrainFloat(0.01f)
#define ND_POLICY_MAX_SIGMA_SQUARE				ndBrainFloat(1.0f)

#define ND_CONTINUE_PROXIMA_POLICY_ITERATIONS	0

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
	m_criticValueIterations = 5;

	m_learnRate = ndBrainFloat(1.0e-4f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_discountRewardFactor = ndBrainFloat(0.99f);
	m_minSigmaSquared = ND_POLICY_MIN_SIGMA_SQUARE;
	m_maxSigmaSquared = ND_POLICY_MAX_SIGMA_SQUARE;

	m_policyRegularizerType = m_ridge;
	m_criticRegularizerType = m_ridge;


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
	,m_randomGenerator()
	,m_owner(master)
	,m_isDead(false)
{
	m_trajectory.Init(master->m_parameters.m_numberOfActions, master->m_parameters.m_numberOfObservations);

	ndUnsigned32 agentSeed = m_owner->m_randomGenerator();
	m_randomGenerator.m_gen.seed(agentSeed);
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
		ndBrainFloat unitVarianceSample = m_randomGenerator.m_d(m_randomGenerator.m_gen);
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
	ndFloat32 maxGain = ndFloat32(0.99f) / (ndFloat32(1.0f) - m_parameters.m_discountRewardFactor);
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
	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
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

	for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
	{
		ndMemCpy(trajectory.GetNextObservations(i), trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations);
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

	if (trajectory.GetCount() >= m_parameters.m_maxTrajectorySteps)
	{
		ndAssert(0);
		trajectory.SetCount(m_parameters.m_maxTrajectorySteps);
		trajectory.SetTerminalState(m_parameters.m_maxTrajectorySteps - 1, true);
	}

	ndInt32 base = m_trajectoryAccumulator.GetCount();
	m_trajectoryAccumulator.SetCount(m_trajectoryAccumulator.GetCount() + trajectory.GetCount());
	for (ndInt32 i = 0; i < trajectory.GetCount(); ++i)
	{
		m_trajectoryAccumulator.CopyFrom(base + i, trajectory, i);
	}
}

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
		//inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		//m_criticTrainer->MakePrediction();
		//m_advantageMinibatchBuffer->Sub(*outputBuffer);
		m_advantageBuffer->CopyBuffer(advantageInfo, 1, **m_advantageMinibatchBuffer);

		advantageInfo.m_dstOffsetInByte += dstStride;
		observationInfo.m_srcOffsetInByte += srcStride;
		expectedRewardInfo.m_srcOffsetInByte += srcStride;
	}
	ndAssert(0);
	//m_advantageBuffer->GaussianNormalize(ndInt32 (m_numberOfMinibatches));
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
	sampledActions.m_dstOffsetInByte = 0;
	sampledActions.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));
	sampledActions.m_strideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));

	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcStrideInByte = ndInt32(sizeof(ndReal));
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

	//OptimizeCritic();
	//CalculateAdvantage();
	//OptimizePolicy();

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

