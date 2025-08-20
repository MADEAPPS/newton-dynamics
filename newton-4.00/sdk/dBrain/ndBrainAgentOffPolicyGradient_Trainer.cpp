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
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainAgentPolicyGradientActivation.h"
#include "ndBrainAgentOffPolicyGradient_Trainer.h"

// this activation suffers a lot form exploding gradients, when many neuron die, 
#define ND_HIDEN_LAYERS_ACTIVATION				ndBrainLayerActivationRelu
 
// trying leaky linear rectifier to see if is better at dealing with banishing and exploding gradients
//#define ND_HIDEN_LAYERS_ACTIVATION			ndBrainLayerActivationLeakyRelu

// not exploding gradients but too slow to convergence
//#define ND_HIDEN_LAYERS_ACTIVATION			ndBrainLayerActivationTanh
 
#define ND_POLICY_MAX_ENTROPY_TEMPERATURE		ndBrainFloat(0.1f)
#define ND_POLICY_MIN_ENTROPY_TEMPERATURE		ndBrainFloat(0.01f)

#define ND_POLICY_TRAINING_EXPLORATION_NOISE	ndBrainFloat(0.2f)
#define ND_POLICY_DEFAULT_POLYAK_BLEND			ndBrainFloat(0.005f)
#define ND_POLICY_MIN_SIGMA						(ndBrainFloat(0.01f))
#define ND_POLICY_MAX_SIGMA						(ndBrainFloat(0.10f))

//#define ND_USING_TANGH_AS_LAST_LINEAR_LAYER

ndBrainAgentOffPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_learnRate = ndBrainFloat(1.0e-4f);
	//m_policyRegularizer = ndBrainFloat(0.0f);
	//m_criticRegularizer = ndBrainFloat(0.0f);
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

	m_randomSeed = 42;
	m_numberOfUpdates = 8;
	m_numberOfHiddenLayers = 2;
	m_maxTrajectorySteps = 1024 * 4;
	m_replayBufferSize = 1024 * 1024;
	m_hiddenLayersNumberOfNeurons = 256;
	m_replayBufferStartOptimizeSize = 1024 * 64;

	m_entropyTemperature = ND_POLICY_MAX_ENTROPY_TEMPERATURE;
	m_entropyMinTemperature = ND_POLICY_MIN_ENTROPY_TEMPERATURE;
	m_entropyMaxTemperature = ND_POLICY_MAX_ENTROPY_TEMPERATURE;
	m_entropyFrameStar = 0;
	m_entropyFrameEnd = m_replayBufferStartOptimizeSize * 2;

//m_useGpuBackend = false;
//m_numberOfUpdates = 1;
//m_replayBufferStartOptimizeSize = 1024 * 2;
}

ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::ndTrajectory()
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::ndTrajectory(ndInt32 actionsSize, ndInt32 obsevationsSize)
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

void ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize;
	m_obsevationsSize = obsevationsSize;
}

void ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::Clear(ndInt32 entry)
{
	m_reward[entry] = ndBrainFloat(0.0f);
	m_terminal[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

void ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::CopyFrom(ndInt32 entry, ndTrajectory& src, ndInt32 srcEntry)
{
	m_reward[entry] = src.m_reward[srcEntry];
	m_terminal[entry] = src.m_terminal[srcEntry];
	ndMemCpy(&m_actions[entry * m_actionsSize], &src.m_actions[srcEntry * m_actionsSize], m_actionsSize);
	ndMemCpy(&m_observations[entry * m_obsevationsSize], &src.m_observations[srcEntry * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&m_nextObservations[entry * m_obsevationsSize], &src.m_nextObservations[srcEntry * m_obsevationsSize], m_obsevationsSize);
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetCount() const
{
	return ndInt32(m_reward.GetCount());
}

void ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::SetCount(ndInt32 count)
{
	m_reward.SetCount(count);
	m_terminal.SetCount(count);
	m_actions.SetCount(count * m_actionsSize);
	m_observations.SetCount(count * m_obsevationsSize);
	m_nextObservations.SetCount(count * m_obsevationsSize);
}

ndBrainFloat ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetReward(ndInt32 entry) const
{
	return m_reward[entry];
}

void ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	m_reward[entry] = reward;
}

bool ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetTerminalState(ndInt32 entry) const
{
	return (m_terminal[entry] == 0.0f) ? true : false;
}

void ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	m_terminal[entry] = isTernimal ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
}

ndBrainFloat* ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry)
{
	return &m_actions[entry * m_actionsSize];
}

const ndBrainFloat* ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry) const
{
	return &m_actions[entry * m_actionsSize];
}

ndBrainFloat* ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry)
{
	return &m_observations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry) const
{
	return &m_observations[entry * m_obsevationsSize];
}

ndBrainFloat* ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry)
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry) const
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetRewardOffset() const
{
	return 0;
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetTerminalOffset() const
{
	return GetRewardOffset() + 1;
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetActionOffset() const
{
	return 1 + GetTerminalOffset();
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetObsevationOffset() const
{
	return m_actionsSize + GetActionOffset();
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetNextObsevationOffset() const
{
	return m_obsevationsSize + GetObsevationOffset();
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetStride() const
{
	//ndBrainVector m_reward;
	//ndBrainVector m_terminal;
	//ndBrainVector m_actions;
	//ndBrainVector m_observations;
	//ndBrainVector m_nextObservations;
	//return 1 + 1 + m_actionsSize + m_obsevationsSize + m_obsevationsSize;
	return m_obsevationsSize + GetNextObsevationOffset();
}

void ndBrainAgentOffPolicyGradient_Agent::ndTrajectory::GetFlatArray(ndInt32 index, ndBrainVector& output) const
{
	output.SetCount(GetStride());
	output[0] = m_reward[index];
	output[1] = m_terminal[index];
	ndMemCpy(&output[GetActionOffset()], &m_actions[index * m_actionsSize], m_actionsSize);
	ndMemCpy(&output[GetObsevationOffset()], &m_observations[index * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&output[GetNextObsevationOffset()], &m_nextObservations[index * m_obsevationsSize], m_obsevationsSize);
}

ndBrainAgentOffPolicyGradient_Agent::ndBrainAgentOffPolicyGradient_Agent(ndBrainAgentOffPolicyGradient_Trainer* const master)
	:ndBrainAgent()
	,m_owner(nullptr)
	,m_trajectory()
	,m_randomeGenerator()
	,m_trajectoryBaseIndex(0)
{
	const ndBrain* const brain = *master->m_policyTrainer->GetBrain();
	m_trajectory.Init(brain->GetOutputSize(), master->m_parameters.m_numberOfObservations);
	m_randomeGenerator.m_gen.seed(master->m_parameters.m_randomSeed);
}

ndInt32 ndBrainAgentOffPolicyGradient_Agent::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

void ndBrainAgentOffPolicyGradient_Agent::SampleActions(ndBrainVector& actions)
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

void ndBrainAgentOffPolicyGradient_Agent::Step()
{
	ndAssert(m_owner);
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainAgentOffPolicyGradient_Trainer* const owner = m_owner;

	const ndBrain* const policy = owner->GetPolicyNetwork();
	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), policy->GetOutputSize());
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), owner->m_parameters.m_numberOfObservations);
	
	GetObservation(&observation[0]);

	bool isdead = IsTerminal();
	//ndBrainFloat rewardScale = ndFloat32(1.0f) - owner->m_parameters.m_discountRewardFactor;
	ndBrainFloat rewardScale = ndFloat32(1.0f);
	ndBrainFloat reward = CalculateReward() * rewardScale;

	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, isdead);
	policy->MakePrediction(observation, actions);
	
	SampleActions(actions);
	ApplyActions(&actions[0]);
}

ndBrainAgentOffPolicyGradient_Trainer::ndBrainAgentOffPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndClassAlloc()
	,m_name()
	,m_parameters(parameters)
	,m_context()
	,m_agent(nullptr)
	,m_randomGenerator(std::random_device{}())
	,m_uniformDistribution(ndFloat32(0.0f), ndFloat32(1.0f))
	,m_uniformRandom(nullptr)
	,m_minibatchMean(nullptr)
	,m_minibatchSigma(nullptr)
	,m_replayBufferFlat(nullptr)
	,m_minibatchNoTerminal(nullptr)
	,m_minibatchOfTransitions(nullptr)
	,m_minibatchExpectedRewards(nullptr)
	,m_minibatchCriticInputTest(nullptr)
	,m_minibatchUniformRandomDistribution(nullptr)
	,m_randomShuffleBuffer(nullptr)
	,m_minibatchIndexBuffer(nullptr)
	,m_lastPolicy()
	,m_scratchBuffer()
	,m_shuffleBuffer()
	,m_miniBatchIndices()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_frameCount(0)
	,m_horizonSteps(0)
	,m_eposideCount(0)
	,m_replayBufferIndex(0)
	,m_shuffleBatchIndex(0)
	,m_replayIsFilled(false)
	,m_startOptimization(false)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);

	m_randomGenerator.seed(m_parameters.m_randomSeed);
	
	m_parameters.m_numberOfUpdates = ndMax(m_parameters.m_numberOfUpdates, 2);
	m_parameters.m_numberOfUpdates = ndMax(m_parameters.m_numberOfUpdates, 2);
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

	ndInt32 inputSize = m_policyTrainer->GetBrain()->GetInputSize();
	ndInt32 outputSize = m_policyTrainer->GetBrain()->GetOutputSize();
	ndInt32 actionsSize = outputSize / 2;
	ndBrainAgentOffPolicyGradient_Agent::ndTrajectory trajectory(outputSize, inputSize);

	m_minibatchNoTerminal = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchIndexBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchExpectedRewards = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));

	m_replayBufferFlat = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_replayBufferSize * trajectory.GetStride()));
	m_minibatchOfTransitions = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * trajectory.GetStride()));
	m_minibatchCriticInputTest = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, (outputSize + inputSize) * m_parameters.m_miniBatchSize));
	m_randomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_numberOfUpdates * m_parameters.m_miniBatchSize));

	m_minibatchEntropy = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchMean = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, actionsSize * m_parameters.m_miniBatchSize));
	m_minibatchSigma = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, actionsSize * m_parameters.m_miniBatchSize));
	m_minibatchUniformRandomDistribution = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, actionsSize * m_parameters.m_miniBatchSize));
	m_uniformRandom = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, actionsSize * m_parameters.m_numberOfUpdates * m_parameters.m_miniBatchSize));
}

void ndBrainAgentOffPolicyGradient_Trainer::SetAgent(ndSharedPtr<ndBrainAgentOffPolicyGradient_Agent>& agent)
{
	m_agent = agent;
	m_agent->m_owner = this;
}

ndBrain* ndBrainAgentOffPolicyGradient_Trainer::GetPolicyNetwork()
{
	return *m_policyTrainer->GetBrain();
}

void ndBrainAgentOffPolicyGradient_Trainer::SaveState(const char* const baseName)
{
	m_context->SyncBufferCommandQueue();

	char fileName[256];
	sprintf(fileName, "%s_policy.dnn", baseName);
	m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
	m_policyTrainer->UpdateParameters(m_scratchBuffer);
	m_policyTrainer->GetBrain()->SaveToFile(fileName);

	for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	{
		sprintf(fileName, "%s_critic_%d.dnn", baseName, j);
		m_criticTrainer[j]->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
		m_criticTrainer[j]->UpdateParameters(m_scratchBuffer);
		m_criticTrainer[j]->GetBrain()->SaveToFile(fileName);

		sprintf(fileName, "%s_referenceCritic_%d.dnn", baseName, j);
		m_referenceCriticTrainer[j]->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
		m_referenceCriticTrainer[j]->UpdateParameters(m_scratchBuffer);
		m_referenceCriticTrainer[j]->GetBrain()->SaveToFile(fileName);
	}
}

void ndBrainAgentOffPolicyGradient_Trainer::RecoverState(const char* baseName)
{
	m_context->SyncBufferCommandQueue();

	char fileName[256];
	sprintf(fileName, "%s_policy.dnn", baseName);
	ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));
	ndTrainerDescriptor descriptor(policy, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
	descriptor.m_regularizer = m_parameters.m_policyRegularizer;
	descriptor.m_regularizerType = m_parameters.m_policyRegularizerType;
	m_policyTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));

	for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	{
		sprintf(fileName, "%s_critic_%d.dnn", baseName, j);
		ndSharedPtr<ndBrain> critic(ndBrainLoad::Load(fileName));
		ndTrainerDescriptor descriptorDescriptor(critic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
		descriptorDescriptor.m_regularizer = m_parameters.m_criticRegularizer;
		descriptorDescriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_criticTrainer[j] = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptorDescriptor));

		sprintf(fileName, "%s_referenceCritic_%d.dnn", baseName, j);
		ndSharedPtr<ndBrain> referenceCritic(ndBrainLoad::Load(fileName));
		ndTrainerDescriptor referenceCriticDescriptor(referenceCritic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
		referenceCriticDescriptor.m_regularizer = m_parameters.m_criticRegularizer;
		referenceCriticDescriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_referenceCriticTrainer[j] = ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainerInference(referenceCriticDescriptor));
	}
}

const ndString& ndBrainAgentOffPolicyGradient_Trainer::GetName() const
{
	return m_name;
}

void ndBrainAgentOffPolicyGradient_Trainer::SetName(const ndString& name)
{
	m_name = name;
}

void ndBrainAgentOffPolicyGradient_Trainer::BuildPolicyClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ND_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions * 2));

	ndBrainVector bias;
	ndBrainVector slope;
	bias.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
	slope.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
	bias.Set(ndBrainFloat(0.0f));
	slope.Set(ndBrainFloat(1.0f));
	ndInt32 elements = ndInt32(bias.GetCount() / 2);
	ndBrainMemVector biasVariance(&bias[elements], elements);
	ndBrainMemVector slopeVariance(&slope[elements], elements);
	ndBrainFloat minLogSigma = ndLog(ND_POLICY_MIN_SIGMA);
	ndBrainFloat maxLogSigma = ndLog(ND_POLICY_MAX_SIGMA);
	biasVariance.Set((maxLogSigma + minLogSigma) * ndBrainFloat(0.5f));
	slopeVariance.Set((maxLogSigma - minLogSigma) * ndBrainFloat(0.5f));

	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	layers.PushBack(new ndBrainLayerActivationLinear(slope, bias));
	layers.PushBack(new ndBrainAgentPolicyGradientActivation(layers[layers.GetCount() - 1]->GetOutputSize()));

	ndSharedPtr<ndBrain> policy (new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		policy->AddLayer(layers[i]);
	}
	policy->InitWeights();

	ndTrainerDescriptor descriptor(policy, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
	descriptor.m_regularizer = m_parameters.m_policyRegularizer;
	descriptor.m_regularizerType = m_parameters.m_policyRegularizerType;
	m_policyTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
}

void ndBrainAgentOffPolicyGradient_Trainer::BuildCriticClass()
{
	auto BuildNeuralNetwork = [this]()
	{
		const ndBrain& policy = **m_policyTrainer->GetBrain();
		ndFixSizeArray<ndBrainLayer*, 32> layers;
		layers.SetCount(0);
		layers.PushBack(new ndBrainLayerLinear(policy.GetOutputSize() + policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

		for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
		{
			ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ND_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
		}

		#ifdef ND_USING_TANGH_AS_LAST_LINEAR_LAYER
			// prevent exploding gradients. 
			// it does not seem to make a difference bu the weighs and bias are much smaller. 
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
		#endif

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
		layers.PushBack(new ndBrainLayerActivationLeakyRelu(layers[layers.GetCount() - 1]->GetOutputSize()));

		ndSharedPtr<ndBrain> critic(new ndBrain);
		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			critic->AddLayer(layers[i]);
		}
		critic->InitWeights();
		return critic;
	};

	for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	{
		ndSharedPtr<ndBrain> critic(BuildNeuralNetwork());

		ndSharedPtr<ndBrain> referenceCritic(new ndBrain(**critic));
		ndTrainerDescriptor referenceDescriptor(referenceCritic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
		referenceDescriptor.m_regularizer = m_parameters.m_criticRegularizer;
		referenceDescriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_referenceCriticTrainer[j] = ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainerInference(referenceDescriptor));
		
		ndTrainerDescriptor descriptor(critic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_learnRate);
		descriptor.m_regularizer = m_parameters.m_criticRegularizer;
		descriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_criticTrainer[j] = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
	}
}

bool ndBrainAgentOffPolicyGradient_Trainer::IsValid() const
{
	return m_context->IsValid();
}

bool ndBrainAgentOffPolicyGradient_Trainer::IsSampling() const
{
	return !m_startOptimization;
}

ndUnsigned32 ndBrainAgentOffPolicyGradient_Trainer::GetFramesCount() const
{
	return m_frameCount;
}

ndUnsigned32 ndBrainAgentOffPolicyGradient_Trainer::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentOffPolicyGradient_Trainer::GetAverageScore() const
{
	ndBrainFloat maxScopre = ndBrainFloat(1.0f) / (ndFloat32(1.0f) - m_parameters.m_discountRewardFactor);
	ndBrainFloat score = ndBrainFloat(1.0f) * m_averageExpectedRewards.GetAverage() / maxScopre;
	return score;
	//return m_averageExpectedRewards.GetAverage();
}

ndFloat32 ndBrainAgentOffPolicyGradient_Trainer::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

//#pragma optimize( "", off )
void ndBrainAgentOffPolicyGradient_Trainer::CalculateScore()
{
	ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;

	// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
	ndBrainFloat gamma = m_parameters.m_discountRewardFactor;
	ndBrainFloat stateReward = trajectory.GetReward(trajectory.GetCount() - 1);
	ndBrainFloat averageReward = stateReward;
	for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
	{
		ndBrainFloat r = trajectory.GetReward(i);
		stateReward = r + gamma * stateReward;
		averageReward += stateReward;
	}
	ndInt32 numberOfSteps = trajectory.GetCount();
	if (!trajectory.GetTerminalState(trajectory.GetCount() - 1))
	{
		numberOfSteps -= m_horizonSteps;
		if (numberOfSteps < 100)
		{
			numberOfSteps = 100;
		}
		stateReward = trajectory.GetReward(trajectory.GetCount() - 1);
		ndBrainFloat substractAverageReward = trajectory.GetReward(trajectory.GetCount() - 1);
		for (ndInt32 i = trajectory.GetCount() - 2; i >= numberOfSteps; --i)
		{
			ndBrainFloat r = trajectory.GetReward(i);
			stateReward = r + gamma * stateReward;
			substractAverageReward += stateReward;
		}
		averageReward -= substractAverageReward;
	}
	averageReward /= ndBrainFloat(numberOfSteps);
	m_averageExpectedRewards.Update(averageReward);
	m_averageFramesPerEpisodes.Update(ndReal(trajectory.GetCount()));

	if (m_startOptimization)
	{
		m_eposideCount++;
	}
}

void ndBrainAgentOffPolicyGradient_Trainer::SaveTrajectoryNoTerminal()
{
	ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;

	ndInt32 stride = trajectory.GetStride();
	ndInt32 actionsSize = m_policyTrainer->GetBrain()->GetOutputSize();
	ndInt32 observationSize = m_policyTrainer->GetBrain()->GetInputSize();

	ndInt32 numbeOfEntries = ndInt32(trajectory.GetCount() - m_agent->m_trajectoryBaseIndex - 1);
	m_scratchBuffer.SetCount(numbeOfEntries * stride);
	for (ndInt32 i = 0; i < numbeOfEntries; ++i)
	{
		ndInt32 index = ndInt32(m_agent->m_trajectoryBaseIndex + i);
		ndBrainMemVector entry(&m_scratchBuffer[i * stride], stride);
		entry[trajectory.GetRewardOffset()] = trajectory.GetReward(index + 1);
		entry[trajectory.GetTerminalOffset()] = ndBrainFloat (1.0f) - trajectory.GetTerminalState(index);

		ndBrainMemVector action(&entry[trajectory.GetActionOffset()], actionsSize);
		const ndBrainMemVector srcAction(trajectory.GetActions(index), actionsSize);
		action.Set(srcAction);

		ndBrainMemVector observation(&entry[trajectory.GetObsevationOffset()], observationSize);
		const ndBrainMemVector srcObservation(trajectory.GetObservations(index), observationSize);
		observation.Set(srcObservation);

		ndBrainMemVector nextObservation(&entry[trajectory.GetNextObsevationOffset()], observationSize);
		const ndBrainMemVector nextSrcObservation(trajectory.GetObservations(index + 1), observationSize);
		nextObservation.Set(nextSrcObservation);
	}
	m_agent->m_trajectoryBaseIndex = ndUnsigned32 (trajectory.GetCount() - 1);
}

void ndBrainAgentOffPolicyGradient_Trainer::SaveTrajectoryTerminal()
{
	ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	SaveTrajectoryNoTerminal();

	ndInt32 stride = trajectory.GetStride();
	ndInt32 actionsSize = m_policyTrainer->GetBrain()->GetOutputSize();
	ndInt32 observationSize = m_policyTrainer->GetBrain()->GetInputSize();

	ndInt32 base = ndInt32 (m_scratchBuffer.GetCount() / stride);
	m_scratchBuffer.SetCount(m_scratchBuffer.GetCount() + stride);

	ndInt32 index = ndInt32(m_agent->m_trajectoryBaseIndex);
	ndBrainMemVector entry(&m_scratchBuffer[base * stride], stride);
	entry[trajectory.GetRewardOffset()] = trajectory.GetReward(index);
	entry[trajectory.GetTerminalOffset()] = ndBrainFloat(1.0f) - trajectory.GetTerminalState(index);

	ndBrainMemVector action(&entry[trajectory.GetActionOffset()], actionsSize);
	const ndBrainMemVector srcAction(trajectory.GetActions(index), actionsSize);
	action.Set(srcAction);

	ndBrainMemVector observation(&entry[trajectory.GetObsevationOffset()], observationSize);
	ndBrainMemVector nextObservation(&entry[trajectory.GetNextObsevationOffset()], observationSize);
	const ndBrainMemVector srcObservation(trajectory.GetObservations(index), observationSize);

	observation.Set(srcObservation);
	nextObservation.Set(srcObservation);
}

void ndBrainAgentOffPolicyGradient_Trainer::CacheTrajectoryTransitions()
{
	ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	for (ndInt32 i = ndInt32(m_agent->m_trajectoryBaseIndex); i < trajectory.GetCount(); ++i)
	{
		if (trajectory.GetTerminalState(i))
		{
			trajectory.SetCount(i + 1);
			CalculateScore();
			SaveTrajectoryTerminal();

			m_agent->ResetModel();
			trajectory.SetCount(0);
			m_agent->m_trajectoryBaseIndex = 0;
			m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
			return;
		}
	}
	SaveTrajectoryNoTerminal();
	if (trajectory.GetCount() >= m_parameters.m_maxTrajectorySteps)
	{
		CalculateScore();
		m_agent->ResetModel();
		trajectory.SetCount(0);
		m_agent->m_trajectoryBaseIndex = 0;
		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	}
}

void ndBrainAgentOffPolicyGradient_Trainer::SaveTrajectory()
{
	CacheTrajectoryTransitions();

	ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;

	ndInt32 stride = trajectory.GetStride();
	ndInt32 entriesCount = ndInt32 (m_scratchBuffer.GetCount() / stride);
	size_t sizeInBytes = m_scratchBuffer.GetCount() * sizeof(ndReal);
	size_t startOffset = m_replayBufferIndex * stride * sizeof(ndReal);

	if (ndInt32(m_replayBufferIndex + entriesCount) <= m_parameters.m_replayBufferSize) 
	{
		if (!m_replayIsFilled)
		{
			ndInt32 base = ndInt32(m_shuffleBuffer.GetCount());
			for (ndInt32 i = 0; i < entriesCount; ++i)
			{
				m_shuffleBuffer.PushBack(base + i);
			}
		}
		ndAssert((startOffset + sizeInBytes) <= m_replayBufferFlat->SizeInBytes());
		m_replayBufferFlat->MemoryToDevice(startOffset, sizeInBytes, &m_scratchBuffer[0]);
		m_replayBufferIndex = m_replayBufferIndex + entriesCount;
	}
	else
	{
		m_replayIsFilled = true;
		m_replayBufferIndex = 0;
	}

	m_scratchBuffer.SetCount(0);
	m_startOptimization = m_startOptimization || (ndInt32 (m_replayBufferIndex) > m_parameters.m_replayBufferStartOptimizeSize);
}

//#pragma optimize( "", off )
void ndBrainAgentOffPolicyGradient_Trainer::CalculateExpectedRewards()
{
	// Get the rewards for this mini batch
	ndBrainTrainerInference* const policy = *m_policyTrainer;

	ndBrainFloatBuffer* const policyMinibatchInputBuffer = policy->GetInputBuffer();


	const ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	ndInt32 transitionStrideInBytes = ndInt32(trajectory.GetStride() * sizeof(ndReal));

	const ndInt32 policyInputSize = ndInt32(policy->GetBrain()->GetInputSize());

	ndCopyBufferCommandInfo policyNextObservation;
	policyNextObservation.m_srcStrideInByte = transitionStrideInBytes;
	policyNextObservation.m_srcOffsetInByte = ndInt32(trajectory.GetNextObsevationOffset() * sizeof(ndReal));
	policyNextObservation.m_dstOffsetInByte = 0;
	policyNextObservation.m_dstStrideInByte = policyInputSize * ndInt32(sizeof(ndReal));
	policyNextObservation.m_strideInByte = policyInputSize * ndInt32(sizeof(ndReal));
	policyMinibatchInputBuffer->CopyBuffer(policyNextObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
	policy->MakePrediction();

	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = policy->GetOuputBuffer();
	const ndInt32 policyOutputSize = ndInt32(policy->GetBrain()->GetOutputSize());
	const ndInt32 policyActionSize = policyOutputSize / 2;

	ndCopyBufferCommandInfo minibatchMean;
	minibatchMean.m_srcOffsetInByte = 0;
	minibatchMean.m_srcStrideInByte = policyOutputSize * ndInt32(sizeof(ndReal));
	minibatchMean.m_dstOffsetInByte = 0;
	minibatchMean.m_dstStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
	minibatchMean.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
	m_minibatchMean->CopyBuffer(minibatchMean, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

	ndCopyBufferCommandInfo minibatchSigma;
	minibatchSigma.m_srcOffsetInByte = policyActionSize * ndInt32(sizeof(ndReal));
	minibatchSigma.m_srcStrideInByte = policyOutputSize * ndInt32(sizeof(ndReal));
	minibatchSigma.m_dstOffsetInByte = 0;
	minibatchSigma.m_dstStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
	minibatchSigma.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
	m_minibatchSigma->CopyBuffer(minibatchSigma, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

	// draw a sample action from the normal distribution and clip again the bounds
	m_minibatchUniformRandomDistribution->StandardNormalDistribution();
	m_minibatchUniformRandomDistribution->Mul(**m_minibatchSigma);
	m_minibatchMean->Add(**m_minibatchUniformRandomDistribution);
	m_minibatchMean->Min(ndBrainFloat(1.0f));
	m_minibatchMean->Max(ndBrainFloat(-1.0f));

	const ndInt32 criticInputSize = policy->GetBrain()->GetInputSize() + policy->GetBrain()->GetOutputSize();
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
	{
		ndBrainTrainerInference& referenceCritic = **m_referenceCriticTrainer[i];
		ndBrainFloatBuffer& criticInputBuffer = *referenceCritic.GetInputBuffer();

		ndCopyBufferCommandInfo criticInputNextActionMean;
		criticInputNextActionMean.m_srcOffsetInByte = 0;
		criticInputNextActionMean.m_srcStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
		criticInputNextActionMean.m_dstOffsetInByte = 0;
		criticInputNextActionMean.m_dstStrideInByte = criticInputSize * ndInt32(sizeof(ndReal));
		criticInputNextActionMean.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
		criticInputBuffer.CopyBuffer(criticInputNextActionMean, m_parameters.m_miniBatchSize, **m_minibatchMean);

		ndCopyBufferCommandInfo criticInputNextActionSigma;
		criticInputNextActionSigma.m_srcOffsetInByte = 0;
		criticInputNextActionSigma.m_srcStrideInByte = policyActionSize * ndInt32(sizeof(ndReal));
		criticInputNextActionSigma.m_dstOffsetInByte = policyActionSize * ndInt32(sizeof(ndReal));
		criticInputNextActionSigma.m_dstStrideInByte = criticInputSize * ndInt32(sizeof(ndReal));
		criticInputNextActionSigma.m_strideInByte = policyActionSize * ndInt32(sizeof(ndReal));
		criticInputBuffer.CopyBuffer(criticInputNextActionSigma, m_parameters.m_miniBatchSize, **m_minibatchSigma);

		ndCopyBufferCommandInfo criticInputNextObservation;
		criticInputNextObservation.m_srcOffsetInByte = 0;
		criticInputNextObservation.m_srcStrideInByte = policyInputSize * ndInt32(sizeof(ndReal));
		criticInputNextObservation.m_dstStrideInByte = criticInputSize * ndInt32(sizeof(ndReal));
		criticInputNextObservation.m_dstOffsetInByte = policyOutputSize * ndInt32(sizeof(ndReal));
		criticInputNextObservation.m_strideInByte = policyInputSize * ndInt32(sizeof(ndReal));
		criticInputBuffer.CopyBuffer(criticInputNextObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);
		m_referenceCriticTrainer[i]->MakePrediction();
	}

	// get the smaller of the q values
	ndBrainFloatBuffer& qValue = *m_referenceCriticTrainer[0]->GetOuputBuffer();
	for (ndInt32 i = 1; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
	{
		const ndBrainFloatBuffer& qValue1 = *m_referenceCriticTrainer[i]->GetOuputBuffer();
		qValue.Min(qValue1);
	}

	ndCopyBufferCommandInfo criticOutputTerminal;
	criticOutputTerminal.m_srcOffsetInByte = ndInt32(trajectory.GetTerminalOffset() * sizeof(ndReal));
	criticOutputTerminal.m_srcStrideInByte = transitionStrideInBytes;
	criticOutputTerminal.m_dstOffsetInByte = 0;
	criticOutputTerminal.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	criticOutputTerminal.m_strideInByte = ndInt32(sizeof(ndReal));
	m_minibatchNoTerminal->CopyBuffer(criticOutputTerminal, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);

	// calculate and add entropy regularization to the q value
	m_minibatchEntropy->CalculateEntropyRegularization(**m_minibatchUniformRandomDistribution, **m_minibatchSigma, m_parameters.m_entropyTemperature);

	qValue.Sub(**m_minibatchEntropy);
	qValue.Mul(**m_minibatchNoTerminal);
	qValue.Scale(m_parameters.m_discountRewardFactor);

	ndCopyBufferCommandInfo criticOutputReward;
	criticOutputReward.m_srcOffsetInByte = ndInt32(trajectory.GetRewardOffset() * sizeof(ndReal));
	criticOutputReward.m_srcStrideInByte = transitionStrideInBytes;
	criticOutputReward.m_dstOffsetInByte = 0;
	criticOutputReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	criticOutputReward.m_strideInByte = ndInt32(sizeof(ndReal));
	m_minibatchExpectedRewards->CopyBuffer(criticOutputReward, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
	m_minibatchExpectedRewards->Add(qValue);
}

//#pragma optimize( "", off )
void ndBrainAgentOffPolicyGradient_Trainer::TrainCritics(ndInt32 criticIndex)
{
	ndInt32 criticInputSize = m_policyTrainer->GetBrain()->GetInputSize() + m_policyTrainer->GetBrain()->GetOutputSize();
	
	ndBrainTrainer& critic = **m_criticTrainer[criticIndex];
	const ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	
	ndCopyBufferCommandInfo criticInputAction;
	criticInputAction.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
	criticInputAction.m_srcOffsetInByte = ndInt32(trajectory.GetActionOffset() * sizeof(ndReal));
	criticInputAction.m_dstOffsetInByte = 0;
	criticInputAction.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
	criticInputAction.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
	ndBrainFloatBuffer* const criticMinibatchInputBuffer = critic.GetInputBuffer();
	criticMinibatchInputBuffer->CopyBuffer(criticInputAction, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
	
	ndCopyBufferCommandInfo criticInputObservation;
	criticInputObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
	criticInputObservation.m_srcOffsetInByte = ndInt32(trajectory.GetObsevationOffset() * sizeof(ndReal));
	criticInputObservation.m_dstOffsetInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
	criticInputObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
	criticInputObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	criticMinibatchInputBuffer->CopyBuffer(criticInputObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
	
	critic.MakePrediction();
	
	// calculate loss
	const ndBrainFloatBuffer* const criticMinibatchOutputBuffer = critic.GetOuputBuffer();
	ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
	criticMinibatchOutputGradientBuffer->Set(*criticMinibatchOutputBuffer);
	criticMinibatchOutputGradientBuffer->Sub(**m_minibatchExpectedRewards);

	//// using a Huber loss to prevent exploding gradient
	//const ndBrainFloat huberSlope = ndBrainFloat(8.0f);
	//criticMinibatchOutputGradientBuffer->Min(huberSlope);
	//criticMinibatchOutputGradientBuffer->Max(-huberSlope);

	// back propagate loss
	critic.BackPropagate();

	// update parameters
	critic.ApplyLearnRate();
}

void ndBrainAgentOffPolicyGradient_Trainer::TrainPolicy()
{
	const ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;

	ndBrainTrainerInference* const policy = *m_policyTrainer;
	ndInt32 criticInputSize = policy->GetBrain()->GetInputSize() + policy->GetBrain()->GetOutputSize();

	ndBrainFloatBuffer* const policyMinibatchInputBuffer = policy->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = policy->GetOuputBuffer();

	ndCopyBufferCommandInfo policyObservation;
	policyObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
	policyObservation.m_srcOffsetInByte = ndInt32(trajectory.GetObsevationOffset() * sizeof(ndReal));
	policyObservation.m_dstOffsetInByte = 0;
	policyObservation.m_dstStrideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyObservation.m_strideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyMinibatchInputBuffer->CopyBuffer(policyObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);

	policy->MakePrediction();

	ndInt32 meanOutputSizeInBytes = ndInt32 (policy->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);

	ndCopyBufferCommandInfo minibatchMean;
	minibatchMean.m_srcOffsetInByte = 0;
	minibatchMean.m_srcStrideInByte = 2 * meanOutputSizeInBytes;
	minibatchMean.m_dstOffsetInByte = 0;
	minibatchMean.m_dstStrideInByte = meanOutputSizeInBytes;
	minibatchMean.m_strideInByte = meanOutputSizeInBytes;
	m_minibatchMean->CopyBuffer(minibatchMean, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

	ndCopyBufferCommandInfo minibatchSigma;
	minibatchSigma.m_srcOffsetInByte = meanOutputSizeInBytes;
	minibatchSigma.m_srcStrideInByte = 2 * meanOutputSizeInBytes;
	minibatchSigma.m_dstOffsetInByte = 0;
	minibatchSigma.m_dstStrideInByte = meanOutputSizeInBytes;
	minibatchSigma.m_strideInByte = meanOutputSizeInBytes;
	m_minibatchSigma->CopyBuffer(minibatchSigma, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

	// exploration is generated by sampling a normal distributed with zero mean and action varince
	m_minibatchUniformRandomDistribution->StandardNormalDistribution();
	m_minibatchUniformRandomDistribution->Mul(**m_minibatchSigma);

	// add noise to the actions mean and clip the result to the actions limits
	m_minibatchMean->Add(**m_minibatchUniformRandomDistribution);
	m_minibatchMean->Min(ndBrainFloat(1.0f));
	m_minibatchMean->Max(ndBrainFloat(-1.0f));

	// using the critic with the lower gradient
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
	{
		ndBrainTrainer& critic = **m_criticTrainer[i];
		ndBrainFloatBuffer* const criticMinibatchInputBuffer = critic.GetInputBuffer();

		ndCopyBufferCommandInfo criticInputNextActionMean;
		criticInputNextActionMean.m_srcOffsetInByte = 0;
		criticInputNextActionMean.m_srcStrideInByte = meanOutputSizeInBytes;
		criticInputNextActionMean.m_dstOffsetInByte = 0;
		criticInputNextActionMean.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputNextActionMean.m_strideInByte = meanOutputSizeInBytes;
		criticMinibatchInputBuffer->CopyBuffer(criticInputNextActionMean, m_parameters.m_miniBatchSize, **m_minibatchMean);

		ndCopyBufferCommandInfo criticInputNextActionSigma;
		criticInputNextActionSigma.m_srcOffsetInByte = 0;
		criticInputNextActionSigma.m_srcStrideInByte = meanOutputSizeInBytes;
		criticInputNextActionSigma.m_dstOffsetInByte = meanOutputSizeInBytes;
		criticInputNextActionSigma.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputNextActionSigma.m_strideInByte = meanOutputSizeInBytes;
		criticMinibatchInputBuffer->CopyBuffer(criticInputNextActionSigma, m_parameters.m_miniBatchSize, **m_minibatchSigma);

		ndCopyBufferCommandInfo criticInputObservation;
		criticInputObservation.m_srcOffsetInByte = 0;
		criticInputObservation.m_srcStrideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
		criticInputObservation.m_dstOffsetInByte = 2 * meanOutputSizeInBytes;
		criticInputObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputObservation.m_strideInByte = ndInt32(policy->GetBrain()->GetInputSize() * sizeof(ndReal));
		criticMinibatchInputBuffer->CopyBuffer(criticInputObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);

		critic.MakePrediction();

		//the gradient is just 1.0
		ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
		criticMinibatchOutputGradientBuffer->Set(ndBrainFloat(1.0f));
		critic.BackPropagate();
	}

	ndBrainTrainer& critic = **m_criticTrainer[0];
	ndBrainFloatBuffer* const criticMinibatchOutputBuffer = critic.GetOuputBuffer();
	ndBrainFloatBuffer* const criticMinibatchInputGradientBuffer = critic.GetInputGradientBuffer();
	for (ndInt32 i = 1; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
	{
		ndBrainTrainer& critic1 = **m_criticTrainer[i];
		ndBrainFloatBuffer* const criticMinibatchOutputBuffer1 = critic1.GetOuputBuffer();
		ndBrainFloatBuffer* const criticMinibatchInputGradientBuffer1 = critic1.GetInputGradientBuffer();
		
		// re using m_minibatchExpectedRewards because is not use anymore 
		m_minibatchExpectedRewards->Set(*criticMinibatchOutputBuffer1);
		m_minibatchExpectedRewards->LessEqual(*criticMinibatchOutputBuffer);
		m_minibatchCriticInputTest->BroadcastScaler(**m_minibatchExpectedRewards);
		
		criticMinibatchOutputBuffer->Blend(*criticMinibatchOutputBuffer1, **m_minibatchExpectedRewards);
		criticMinibatchInputGradientBuffer->Blend(*criticMinibatchInputGradientBuffer1, **m_minibatchCriticInputTest);
	}

	// calculate entropy Regularization
	ndCopyBufferCommandInfo policyCopyGradients;
	policyCopyGradients.m_srcOffsetInByte = 0;
	policyCopyGradients.m_srcStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
	policyCopyGradients.m_dstOffsetInByte = 0;
	policyCopyGradients.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));;
	policyCopyGradients.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
	policyMinibatchOutputBuffer->CopyBuffer(policyCopyGradients, m_parameters.m_miniBatchSize, *criticMinibatchInputGradientBuffer);
	ndBrainFloatBuffer* const policyMinibatchOutputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();
	policyMinibatchOutputGradientBuffer->CalculateEntropyRegularizationGradient(**m_minibatchUniformRandomDistribution, **m_minibatchSigma, m_parameters.m_entropyTemperature, ndInt32(meanOutputSizeInBytes / sizeof(ndReal)));

	// subtract the qValue gradient from the entropy gradient.
	// The subtraction order is in reverse order, to get the gradient ascend.
	policyMinibatchOutputGradientBuffer->Sub(*policyMinibatchOutputBuffer);
	m_policyTrainer->BackPropagate();

	m_policyTrainer->ApplyLearnRate();
}

//#pragma optimize( "", off )
void ndBrainAgentOffPolicyGradient_Trainer::Optimize()
{
	// get the number of indirect transitions 
	m_miniBatchIndices.SetCount(0);
	const ndInt32 numberOfSamples = m_parameters.m_numberOfUpdates * m_parameters.m_miniBatchSize;
	if ((numberOfSamples + m_shuffleBatchIndex) >= (m_shuffleBuffer.GetCount() - numberOfSamples))
	{
		// re shuffle after every pass.
		m_shuffleBatchIndex = 0;
		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	}

	for (ndInt32 i = 0; i < numberOfSamples; ++i)
	{
		m_miniBatchIndices.PushBack(m_shuffleBuffer[m_shuffleBatchIndex]);
		m_shuffleBatchIndex++;
		if (m_shuffleBatchIndex >= m_shuffleBuffer.GetCount())
		{
			// re shuffle after every pass.
			m_shuffleBatchIndex = 0;
			m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
		}
	}

	// load all the shuffled indices, they use GPU command to get a mini batch
	ndAssert(m_randomShuffleBuffer->SizeInBytes() == m_miniBatchIndices.GetCount() * sizeof(ndInt32));
	m_randomShuffleBuffer->MemoryToDevice(0, m_randomShuffleBuffer->SizeInBytes(), &m_miniBatchIndices[0]);

	// get a vector of random numbers
	m_scratchBuffer.SetCount(0);
	const ndInt32 numberOfActions = m_policyTrainer->GetBrain()->GetOutputSize() / 2;
	for (ndInt32 i = 0; i < numberOfSamples; ++i)
	{
		for (ndInt32 j = 0; j < numberOfActions; ++j)
		{
			m_scratchBuffer.PushBack(m_uniformDistribution(m_randomGenerator));
		}
	}
	m_uniformRandom->VectorToDevice(m_scratchBuffer);

	const ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	ndInt32 transitionSizeInBytes = ndInt32(trajectory.GetStride() * sizeof(ndInt32));
	ndInt32 copyIndicesStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));

	for (ndInt32 i = 0; i < m_parameters.m_numberOfUpdates; ++i)
	{
		// get a mini batch of uniform distributed random number form 0.0 to 1.0
		ndCopyBufferCommandInfo minibatchReparametization;
		ndInt32 strideSizeInBytes = ndInt32(sizeof(ndInt32)) * numberOfActions * m_parameters.m_miniBatchSize;
		minibatchReparametization.m_dstOffsetInByte = 0;
		minibatchReparametization.m_dstStrideInByte = strideSizeInBytes;
		minibatchReparametization.m_srcOffsetInByte = i * strideSizeInBytes;
		minibatchReparametization.m_srcStrideInByte = strideSizeInBytes;
		minibatchReparametization.m_strideInByte = strideSizeInBytes;
		m_minibatchUniformRandomDistribution->CopyBuffer(minibatchReparametization, 1, **m_uniformRandom);

		// sample a random mini batch of shuffled transitions indices
		ndCopyBufferCommandInfo copyIndicesInfo;
		copyIndicesInfo.m_dstOffsetInByte = 0;
		copyIndicesInfo.m_dstStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_srcOffsetInByte = ndInt32(i * copyIndicesStrideInBytes);
		copyIndicesInfo.m_srcStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_strideInByte = copyIndicesStrideInBytes;
		m_minibatchIndexBuffer->CopyBuffer(copyIndicesInfo, 1, **m_randomShuffleBuffer);

		// get an indirect mini batch of transition from flat array
		ndCopyBufferCommandInfo minibatchOfTransitions;
		minibatchOfTransitions.m_dstOffsetInByte = 0;
		minibatchOfTransitions.m_dstStrideInByte = transitionSizeInBytes;
		minibatchOfTransitions.m_srcOffsetInByte = 0;
		minibatchOfTransitions.m_srcStrideInByte = transitionSizeInBytes;
		minibatchOfTransitions.m_strideInByte = transitionSizeInBytes;
		m_minibatchOfTransitions->CopyBufferIndirect(minibatchOfTransitions, **m_minibatchIndexBuffer, **m_replayBufferFlat);

		// calculate expected rewards for thsi mini batch
		CalculateExpectedRewards();

		for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
		{
			TrainCritics(j);
		}

		// load another uniform random array
		//m_minibatchUniformRandomDistribution->CopyBuffer(minibatchReparametization, 1, **m_uniformRandom1);
		m_minibatchUniformRandomDistribution->CopyBuffer(minibatchReparametization, 1, **m_uniformRandom);
		TrainPolicy();

		for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
		{
			ndBrainTrainer* const criticTrainer = *m_criticTrainer[j];
			ndBrainTrainerInference* const referenceCritic = *m_referenceCriticTrainer[j];

			const ndBrainFloatBuffer* const parameterBuffer = criticTrainer->GetWeightAndBiasBuffer();
			ndBrainFloatBuffer* const referenceParameterBuffer = referenceCritic->GetWeightAndBiasBuffer();
			referenceParameterBuffer->Blend(*parameterBuffer, m_parameters.m_polyakBlendFactor);
		}
	}
}

void ndBrainAgentOffPolicyGradient_Trainer::OptimizeStep()
{
	SaveTrajectory();
	if (m_startOptimization)
	{
		ndFloat64 t = ndClamp(ndFloat64(m_frameCount - m_parameters.m_entropyFrameStar) / ndFloat64(m_parameters.m_entropyFrameEnd - m_parameters.m_entropyFrameStar), ndFloat64(0.0f), ndFloat64(1.0f));
		m_parameters.m_entropyTemperature = ndBrainFloat(m_parameters.m_entropyMaxTemperature + t * (m_parameters.m_entropyMinTemperature - m_parameters.m_entropyMaxTemperature));

		Optimize();
		m_frameCount++;

		m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_lastPolicy);
		m_context->SyncBufferCommandQueue();
		m_policyTrainer->UpdateParameters(m_lastPolicy);
	}
}