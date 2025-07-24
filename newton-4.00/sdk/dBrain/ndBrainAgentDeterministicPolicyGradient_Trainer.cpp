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
#include "ndBrainAgentDeterministicPolicyGradient_Trainer.h"

#define ND_SAC_HIDEN_LAYERS_ACTIVATION		ndBrainLayerActivationRelu
//#define ND_SAC_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationTanh
//#define ND_SAC_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationLeakyRelu

#define ND_SAC_POLICY_FIX_SIGMA				ndBrainFloat(0.5f)

#define ND_SAC_POLICY_MIN_PER_ACTION_SIGMA	ndBrainFloat(0.01f)
#define ND_SAC_POLICY_MAX_PER_ACTION_SIGMA	ndBrainFloat(1.0f)
#define ND_SAC_MAX_ENTROPY_COEFFICIENT		ndBrainFloat(2.0e-5f)

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

	m_useGpuBackend = true;
	m_usePerActionSigmas = false;
	m_actionFixSigma = ND_SAC_POLICY_FIX_SIGMA;

	m_policyRegularizerType = m_ridge;
	m_criticRegularizerType = m_ridge;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_randomSeed = 47;
	m_policyUpdatesCount = 16;
	m_criticUpdatesCount = 16;
	m_numberOfHiddenLayers = 3;
	m_maxTrajectorySteps = 1024 * 4;
	m_replayBufferSize = 1024 * 1024;
	m_hiddenLayersNumberOfNeurons = 256;
	m_replayBufferStartOptimizeSize = 1024 * 64;

m_useGpuBackend = false;
//m_policyUpdatesCount = 1;
//m_criticUpdatesCount = 1;
//m_miniBatchSize = 16;
m_miniBatchSize = 128;
//m_replayBufferSize = 1024 * 2;
//m_hiddenLayersNumberOfNeurons = 64;
//m_replayBufferStartOptimizeSize = 1024;
}

ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::ndTrajectory()
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::ndTrajectory(ndInt32 actionsSize, ndInt32 obsevationsSize)
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

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize;
	m_obsevationsSize = obsevationsSize;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::Clear(ndInt32 entry)
{
	m_reward[entry] = ndBrainFloat(0.0f);
	m_terminal[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::CopyFrom(ndInt32 entry, ndTrajectory& src, ndInt32 srcEntry)
{
	m_reward[entry] = src.m_reward[srcEntry];
	m_terminal[entry] = src.m_terminal[srcEntry];
	ndMemCpy(&m_actions[entry * m_actionsSize], &src.m_actions[srcEntry * m_actionsSize], m_actionsSize);
	ndMemCpy(&m_observations[entry * m_obsevationsSize], &src.m_observations[srcEntry * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&m_nextObservations[entry * m_obsevationsSize], &src.m_nextObservations[srcEntry * m_obsevationsSize], m_obsevationsSize);
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetCount() const
{
	return ndInt32(m_reward.GetCount());
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::SetCount(ndInt32 count)
{
	m_reward.SetCount(count);
	m_terminal.SetCount(count);
	m_actions.SetCount(count * m_actionsSize);
	m_observations.SetCount(count * m_obsevationsSize);
	m_nextObservations.SetCount(count * m_obsevationsSize);
}

ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetReward(ndInt32 entry) const
{
	return m_reward[entry];
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	m_reward[entry] = reward;
}

bool ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetTerminalState(ndInt32 entry) const
{
	return (m_terminal[entry] == 0.0f) ? true : false;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	m_terminal[entry] = isTernimal ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry)
{
	return &m_actions[entry * m_actionsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry) const
{
	return &m_actions[entry * m_actionsSize];
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry)
{
	return &m_observations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry) const
{
	return &m_observations[entry * m_obsevationsSize];
}

ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry)
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry) const
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetRewardOffset() const
{
	return 0;
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetTerminalOffset() const
{
	return GetRewardOffset() + 1;
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetActionOffset() const
{
	return 1 + GetTerminalOffset();
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetObsevationOffset() const
{
	return m_actionsSize + GetActionOffset();
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetNextObsevationOffset() const
{
	return m_obsevationsSize + GetObsevationOffset();
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetStride() const
{
	//ndBrainVector m_reward;
	//ndBrainVector m_terminal;
	//ndBrainVector m_actions;
	//ndBrainVector m_observations;
	//ndBrainVector m_nextObservations;
	//return 1 + 1 + m_actionsSize + m_obsevationsSize + m_obsevationsSize;
	return m_obsevationsSize + GetNextObsevationOffset();
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetFlatArray(ndInt32 index, ndBrainVector& output) const
{
	output.SetCount(GetStride());
	output[0] = m_reward[index];
	output[1] = m_terminal[index];
	ndMemCpy(&output[GetActionOffset()], &m_actions[index * m_actionsSize], m_actionsSize);
	ndMemCpy(&output[GetObsevationOffset()], &m_observations[index * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&output[GetNextObsevationOffset()], &m_nextObservations[index * m_obsevationsSize], m_obsevationsSize);
}

ndBrainAgentDeterministicPolicyGradient_Agent::ndBrainAgentDeterministicPolicyGradient_Agent(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master)
	:ndBrainAgent()
	,m_owner(master)
	,m_trajectory()
	,m_randomeGenerator()
	,m_trajectoryBaseIndex(0)
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

	ndBrainAgentDeterministicPolicyGradient_Trainer* const owner = *m_owner;

	const ndBrain* const policy = owner->GetPolicyNetwork();
	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), policy->GetOutputSize());
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), owner->m_parameters.m_numberOfObservations);
	
	GetObservation(&observation[0]);
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());
	
	policy->MakePrediction(observation, actions);
	
	SampleActions(actions);
	ApplyActions(&actions[0]);
}

ndBrainAgentDeterministicPolicyGradient_Trainer::ndBrainAgentDeterministicPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndClassAlloc()
	,m_name()
	,m_parameters(parameters)
	,m_context()
	,m_miniBatchIndices()
	,m_replayBufferFlat()
	,m_agent(nullptr)
	,m_randomGenerator(std::random_device{}())
	,m_uniformDistribution(ndFloat32(0.0f), ndFloat32(1.0f))
	,m_shuffleBuffer()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_replayBufferIndex(0)
	,ndPolycyDelayMod(0)
	,m_shuffleBatchIndex(0)
	,m_replayIsFilled(false)
	,m_startOptimization(false)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);

	m_randomGenerator.seed(m_parameters.m_randomSeed);
	
	m_parameters.m_criticUpdatesCount = ndMax(m_parameters.m_criticUpdatesCount, 2);
	m_parameters.m_policyUpdatesCount = ndMax(m_parameters.m_policyUpdatesCount, 2);

	if (m_parameters.m_useGpuBackend)
	{
		m_context = ndSharedPtr<ndBrainContext>(new ndBrainGpuContext);
	}
	else
	{
		m_context = ndSharedPtr<ndBrainContext>(new ndBrainCpuContext);
	}

	// create actor class
	BuildPolicyClass();
	BuildCriticClass();

	ndInt32 inputSize = m_policyTrainer->GetBrain()->GetInputSize();
	ndInt32 outputSize = m_policyTrainer->GetBrain()->GetOutputSize();

	ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory trajectory(outputSize, inputSize);
	m_replayBufferFlat = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, trajectory.GetStride() * m_parameters.m_replayBufferSize));
	m_expectedRewards = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize));
	m_uniformRandom0 = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize));
	m_uniformRandom1 = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize));

	m_sigmaMinibatch = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize));
	m_rewardMinibatch = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize));
	m_noTerminalMinibatch = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize));
	m_uniformRandomMinibatch = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize));

	m_minibatchIndexBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_randomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize));

	m_critickOutputTest = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_critickInputTest = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, (outputSize + inputSize) *m_parameters.m_miniBatchSize));

	m_context->Set(**m_sigmaMinibatch, m_parameters.m_actionFixSigma);

	m_shuffleBuffer.SetCount(0);
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
		layers.PushBack(new ND_SAC_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
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
		ndBrainFloat b = ndBrainFloat(0.5f) * (ND_SAC_POLICY_MAX_PER_ACTION_SIGMA + ND_SAC_POLICY_MIN_PER_ACTION_SIGMA);
		ndBrainFloat a = ndBrainFloat(0.5f) * (ND_SAC_POLICY_MAX_PER_ACTION_SIGMA - ND_SAC_POLICY_MIN_PER_ACTION_SIGMA);
	
		bias.Set(ndBrainFloat(0.0f));
		slope.Set(ndBrainFloat(1.0f));
		ndMemSet(&bias[sigmaSize], b, sigmaSize);
		ndMemSet(&slope[sigmaSize], a, sigmaSize);
		layers.PushBack(new ndBrainLayerActivationLinear(slope, bias));
	}
	
	ndSharedPtr<ndBrain> policy (new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		policy->AddLayer(layers[i]);
	}
	policy->InitWeights();
	
	ndTrainerDescriptor descriptor(policy, m_context, m_parameters.m_miniBatchSize, m_parameters.m_policyLearnRate);
	descriptor.m_regularizer = m_parameters.m_policyRegularizer;
	descriptor.m_regularizerType = m_parameters.m_policyRegularizerType;
	m_policyTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
}

void ndBrainAgentDeterministicPolicyGradient_Trainer::BuildCriticClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	const ndBrain& policy = **m_policyTrainer->GetBrain();
	for (ndInt32 k = 0; k < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++k)
	{
		layers.SetCount(0);
		layers.PushBack(new ndBrainLayerLinear(policy.GetOutputSize() + policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	
		for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers-1; ++i)
		{
			ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ND_SAC_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
		}

		// prevent exploding gradiens. 
		// it does not seem to make a diference bu the weighs and bias are much smaller. 
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
		layers.PushBack(new ndBrainLayerActivationLeakyRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
		//layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	
		ndSharedPtr<ndBrain> critic(new ndBrain);
		for (ndInt32 i = 0; i < layers.GetCount(); ++i)
		{
			critic->AddLayer(layers[i]);
		}
		critic->InitWeights();
	
		// big mistake here ???
		ndSharedPtr<ndBrain> referenceCritic(new ndBrain(**critic));
	
		ndTrainerDescriptor descriptor(critic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_policyLearnRate);
		descriptor.m_regularizer = m_parameters.m_criticRegularizer;
		descriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_criticTrainer[k] = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
		m_referenceCriticTrainer[k] = ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainerInference(descriptor));
	}
}

//ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution) const
ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32, const ndBrainVector&) const
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
//ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32 index) const
ndBrainFloat ndBrainAgentDeterministicPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32) const
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

void ndBrainAgentDeterministicPolicyGradient_Trainer::CalculateScore()
{
	ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;

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
	averageReward /= ndBrainFloat(trajectory.GetCount());
	m_averageExpectedRewards.Update(averageReward);
	m_averageFramesPerEpisodes.Update(ndReal(trajectory.GetCount()));
}

void ndBrainAgentDeterministicPolicyGradient_Trainer::SaveTrajectoryNoTerminal()
{
	ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;

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

void ndBrainAgentDeterministicPolicyGradient_Trainer::SaveTrajectoryTerminal()
{
	ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
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

#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::SaveTrajectoryLoadBuffer()
{
	ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	for (ndInt32 i = ndInt32(m_agent->m_trajectoryBaseIndex); i < trajectory.GetCount(); ++i)
	{
		if (trajectory.GetTerminalState(i))
		{
			trajectory.SetCount(i + 1);
			CalculateScore();
			SaveTrajectoryTerminal();

			m_eposideCount++;
			m_framesAlive = 0;
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
		m_eposideCount++;
		m_framesAlive = 0;
		m_agent->ResetModel();
		trajectory.SetCount(0);
		m_agent->m_trajectoryBaseIndex = 0;
		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	}
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::SaveTrajectory()
{
	SaveTrajectoryLoadBuffer();

	ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;

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
void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnQvalueFunction(ndInt32 criticIndex)
{
	ndInt32 criticInputSize = m_policyTrainer->GetBrain()->GetInputSize() + m_policyTrainer->GetBrain()->GetOutputSize();

	ndBrainTrainer& critic = **m_criticTrainer[criticIndex];

	ndInt32 copyIndicesStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
	for (ndInt32 n = 0; n < m_parameters.m_criticUpdatesCount; ++n)
	{
		// calculate q value
		ndCopyBufferCommandInfo copyIndicesInfo;
		copyIndicesInfo.m_srcOffsetInByte = ndInt32(n * sizeof(ndUnsigned32) * m_parameters.m_miniBatchSize);
		copyIndicesInfo.m_srcStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_dstOffsetInByte = 0;
		copyIndicesInfo.m_dstStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_strideInByte = copyIndicesStrideInBytes;
		m_minibatchIndexBuffer->CopyBuffer(copyIndicesInfo, 1, **m_randomShuffleBuffer);

		const ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
		ndCopyBufferCommandInfo criticInputAction;
		criticInputAction.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
		criticInputAction.m_srcOffsetInByte = ndInt32(trajectory.GetActionOffset() * sizeof(ndReal));
		criticInputAction.m_dstOffsetInByte = 0;
		criticInputAction.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputAction.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		ndBrainFloatBuffer* const criticMinibatchInputBuffer = critic.GetInputBuffer();
		criticMinibatchInputBuffer->CopyBufferIndirect(criticInputAction, **m_minibatchIndexBuffer, **m_replayBufferFlat);

		ndCopyBufferCommandInfo criticInputObservation;
		criticInputObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
		criticInputObservation.m_srcOffsetInByte = ndInt32(trajectory.GetObsevationOffset() * sizeof(ndReal));
		criticInputObservation.m_dstOffsetInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		criticInputObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
		criticMinibatchInputBuffer->CopyBufferIndirect(criticInputObservation, **m_minibatchIndexBuffer, **m_replayBufferFlat);

		critic.MakePrediction();

		// claculate loss
		ndCopyBufferCommandInfo expectedReward;
		expectedReward.m_srcStrideInByte = ndInt32(sizeof(ndReal));
		expectedReward.m_srcOffsetInByte = ndInt32(n * sizeof(ndInt32) * m_parameters.m_miniBatchSize);
		expectedReward.m_dstOffsetInByte = 0;
		expectedReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
		expectedReward.m_strideInByte = ndInt32(sizeof(ndReal));
		m_critickOutputTest->CopyBuffer(expectedReward, m_parameters.m_miniBatchSize, **m_expectedRewards);

		const ndBrainFloatBuffer* const criticMinibatchOutputBuffer = critic.GetOuputBuffer();
		ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
		m_context->CopyBuffer(*criticMinibatchOutputGradientBuffer, *criticMinibatchOutputBuffer);
		m_context->Sub(*criticMinibatchOutputGradientBuffer, **m_critickOutputTest);

		// backpropagete thes loss
		critic.BackPropagate();
		critic.ApplyLearnRate();
	}

	ndBrainFloatBuffer* const parameterBuffer = critic.GetWeightAndBiasBuffer();
	ndBrainFloatBuffer* const referenceParameterBuffer = m_referenceCriticTrainer[criticIndex]->GetWeightAndBiasBuffer();
	m_context->Blend(*parameterBuffer, *referenceParameterBuffer, m_parameters.m_criticMovingAverageFactor);
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::CalculateExpectedRewards()
{
	m_miniBatchIndices.SetCount(0);
	ndInt32 samplesCount = m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize;

	// get the number of indirect transitions 
	for (ndInt32 i = 0; i < samplesCount; ++i)
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

	// get a vector of randon numbers
	//m_scratchBuffer.SetCount(0);
	//for (ndInt32 i = m_policyTrainer->GetBrain()->GetOutputSize() * samplesCount - 1; i >= 0; --i)
	//{
	//	m_scratchBuffer.PushBack(m_uniformDistribution(m_randomGenerator));
	//}
	//m_uniformRandom0->VectorToDevice(m_scratchBuffer);

	// get a secund vector of randon numbers
	//m_scratchBuffer.SetCount(0);
	//for (ndInt32 i = m_policyTrainer->GetBrain()->GetOutputSize() * samplesCount - 1; i >= 0; --i)
	//{
	//	m_scratchBuffer.PushBack(m_uniformDistribution(m_randomGenerator));
	//}
	//m_uniformRandom1->VectorToDevice(m_scratchBuffer);

	// load all the shuffled indices, they use GPU command to get a minibatch
	ndAssert(m_randomShuffleBuffer->SizeInBytes() == m_miniBatchIndices.GetCount() * sizeof(ndInt32));
	m_randomShuffleBuffer->MemoryToDevice(0, m_randomShuffleBuffer->SizeInBytes(), &m_miniBatchIndices[0]);

	const ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	for (ndInt32 n = 0; n < m_parameters.m_criticUpdatesCount; ++n)
	{
		// Get the rewards for this minibatch, try dispaching the entire epoch
		ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();

		ndCopyBufferCommandInfo copyIndicesInfo;
		ndInt32 copyIndicesStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
		copyIndicesInfo.m_dstOffsetInByte = 0;
		copyIndicesInfo.m_dstStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_srcOffsetInByte = ndInt32(n * sizeof(ndUnsigned32) * m_parameters.m_miniBatchSize);
		copyIndicesInfo.m_srcStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_strideInByte = copyIndicesStrideInBytes;
		m_minibatchIndexBuffer->CopyBuffer(copyIndicesInfo, 1, **m_randomShuffleBuffer);

		ndCopyBufferCommandInfo policyNextObservation;
		policyNextObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
		policyNextObservation.m_srcOffsetInByte = ndInt32(trajectory.GetNextObsevationOffset() * sizeof(ndReal));
		policyNextObservation.m_dstOffsetInByte = 0;
		policyNextObservation.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
		policyNextObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
		policyMinibatchInputBuffer->CopyBufferIndirect(policyNextObservation, **m_minibatchIndexBuffer, **m_replayBufferFlat);
		m_policyTrainer->MakePrediction();

		ndCopyBufferCommandInfo criticOutputReward;
		criticOutputReward.m_srcOffsetInByte = ndInt32(trajectory.GetRewardOffset() * sizeof(ndReal));
		criticOutputReward.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
		criticOutputReward.m_dstOffsetInByte = 0;
		criticOutputReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
		criticOutputReward.m_strideInByte = ndInt32(sizeof(ndReal));
		m_rewardMinibatch->CopyBufferIndirect(criticOutputReward, **m_minibatchIndexBuffer, **m_replayBufferFlat);

		ndCopyBufferCommandInfo criticOutputTerminal;
		criticOutputTerminal.m_srcOffsetInByte = ndInt32(trajectory.GetTerminalOffset() * sizeof(ndReal));
		criticOutputTerminal.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
		criticOutputTerminal.m_dstOffsetInByte = 0;
		criticOutputTerminal.m_dstStrideInByte = ndInt32(sizeof(ndReal));
		criticOutputTerminal.m_strideInByte = ndInt32(sizeof(ndReal));
		m_noTerminalMinibatch->CopyBufferIndirect(criticOutputTerminal, **m_minibatchIndexBuffer, **m_replayBufferFlat);

		//ndCopyBufferCommandInfo uniformRandom;
		//uniformRandom.m_srcOffsetInByte = 0;
		//uniformRandom.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		//uniformRandom.m_dstOffsetInByte = 0;
		//uniformRandom.m_dstStrideInByte = ndInt32(m_parameters.m_miniBatchSize * m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		//uniformRandom.m_strideInByte = ndInt32(m_parameters.m_miniBatchSize * m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		//ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();
		//uniformRandom.m_srcOffsetInByte = ndInt32(n * uniformRandom.m_srcOffsetInByte);
		//m_uniformRandomMinibatch->CopyBuffer(uniformRandom, 1, **m_uniformRandom0);
		//m_context->GaussianSample(*policyMinibatchOutputBuffer, **m_sigmaMinibatch, **m_uniformRandomMinibatch);

		ndInt32 criticInputSize = m_policyTrainer->GetBrain()->GetInputSize() + m_policyTrainer->GetBrain()->GetOutputSize();
		ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();
		for (ndInt32 i = 0; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
		{
			ndBrainTrainerInference& referenceCritic = **m_referenceCriticTrainer[i];
			ndBrainFloatBuffer& criticInputBuffer = *referenceCritic.GetInputBuffer();

			ndCopyBufferCommandInfo criticInputNextAction;
			criticInputNextAction.m_srcOffsetInByte = 0;
			criticInputNextAction.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
			criticInputNextAction.m_dstOffsetInByte = 0;
			criticInputNextAction.m_dstStrideInByte = ndInt32 (criticInputSize * sizeof (ndReal));
			criticInputNextAction.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
			criticInputBuffer.CopyBuffer(criticInputNextAction, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

			ndCopyBufferCommandInfo criticInputNextObservation;
			criticInputNextObservation.m_srcOffsetInByte = 0;
			criticInputNextObservation.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
			criticInputNextObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
			criticInputNextObservation.m_dstOffsetInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
			criticInputNextObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
			criticInputBuffer.CopyBuffer(criticInputNextObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);
			m_referenceCriticTrainer[i]->MakePrediction();

			ndBrainFloatBuffer& qValueBuffer = *referenceCritic.GetOuputBuffer();
			m_context->Mul(qValueBuffer, **m_noTerminalMinibatch);
			m_context->Scale(qValueBuffer, m_parameters.m_discountRewardFactor);
			m_context->Add(qValueBuffer, **m_rewardMinibatch);
		}
		
		ndBrainFloatBuffer& minExpectedReward = *m_referenceCriticTrainer[0]->GetOuputBuffer();
		for (ndInt32 i = 1; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
		{
			const ndBrainFloatBuffer& qValueBuffer1 = *m_referenceCriticTrainer[i]->GetOuputBuffer();
			m_context->Min(minExpectedReward, qValueBuffer1);
		}

		ndCopyBufferCommandInfo expectedReward;
		expectedReward.m_srcOffsetInByte = 0;
		expectedReward.m_srcStrideInByte = ndInt32(sizeof(ndReal));
		expectedReward.m_dstOffsetInByte = ndInt32(n * m_parameters.m_miniBatchSize * sizeof(ndReal));
		expectedReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
		expectedReward.m_strideInByte = ndInt32(sizeof(ndReal));
		m_expectedRewards->CopyBuffer(expectedReward, m_parameters.m_miniBatchSize, minExpectedReward);
	}
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnPolicyFunction()
{
	const ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
		
	ndInt32 copyIndicesStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
	ndInt32 criticInputSize = m_policyTrainer->GetBrain()->GetInputSize() + m_policyTrainer->GetBrain()->GetOutputSize();
	for (ndInt32 n = 0; n < m_parameters.m_policyUpdatesCount; ++n)
	{
		ndCopyBufferCommandInfo copyIndicesInfo;
		copyIndicesInfo.m_dstOffsetInByte = 0;
		copyIndicesInfo.m_dstStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_srcStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_srcOffsetInByte = ndInt32(n * sizeof(ndUnsigned32) * m_parameters.m_miniBatchSize);
		copyIndicesInfo.m_strideInByte = copyIndicesStrideInBytes;
		m_minibatchIndexBuffer->CopyBuffer(copyIndicesInfo, 1, **m_randomShuffleBuffer);

		ndCopyBufferCommandInfo policyObservation;
		policyObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
		policyObservation.m_srcOffsetInByte = ndInt32(trajectory.GetObsevationOffset() * sizeof(ndReal));
		policyObservation.m_dstOffsetInByte = 0;
		policyObservation.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
		policyObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
		ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();
		policyMinibatchInputBuffer->CopyBufferIndirect(policyObservation, **m_minibatchIndexBuffer, **m_replayBufferFlat);
		m_policyTrainer->MakePrediction();

		//ndCopyBufferCommandInfo uniformRandom;
		//uniformRandom.m_srcOffsetInByte = ndInt32(n * uniformRandom.m_srcOffsetInByte);
		//uniformRandom.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		//uniformRandom.m_dstOffsetInByte = 0;
		//uniformRandom.m_dstStrideInByte = ndInt32(m_parameters.m_miniBatchSize * m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		//uniformRandom.m_strideInByte = ndInt32(m_parameters.m_miniBatchSize * m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		//m_uniformRandomMinibatch->CopyBuffer(uniformRandom, 1, **m_uniformRandom1);
		//m_context->GaussianSample(*policyMinibatchOutputBuffer, **m_sigmaMinibatch, **m_uniformRandomMinibatch);

		ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();
		for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
		{
			ndBrainTrainer& critic = **m_criticTrainer[i];
			ndBrainFloatBuffer* const criticMinibatchInputBuffer = critic.GetInputBuffer();

			ndCopyBufferCommandInfo criticInputAction;
			criticInputAction.m_srcOffsetInByte = 0;
			criticInputAction.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
			criticInputAction.m_dstOffsetInByte = 0;
			criticInputAction.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
			criticInputAction.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
			criticMinibatchInputBuffer->CopyBuffer(criticInputAction, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

			ndCopyBufferCommandInfo criticInputObservation;
			criticInputObservation.m_srcOffsetInByte = 0;
			criticInputObservation.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
			criticInputObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
			criticInputObservation.m_dstOffsetInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
			criticInputObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
			criticMinibatchInputBuffer->CopyBuffer(criticInputObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);
			critic.MakePrediction();

			// big mistake here, 
			//ndBrainFloatBuffer* const criticMinibatchOutputBuffer = critic.GetOuputBuffer();
			//ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
			//m_context->Set(*criticMinibatchOutputGradientBuffer, *criticMinibatchOutputBuffer);
			//m_context->GreaterEqual(*criticMinibatchOutputGradientBuffer, ndBrainFloat (0.0f));
			//m_context->Select(*criticMinibatchOutputGradientBuffer, *criticMinibatchOutputGradientBuffer, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));

			//the gradient is just 1.0
			ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
			m_context->Set(*criticMinibatchOutputGradientBuffer, ndBrainFloat(1.0f));
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

			m_context->Set(**m_critickOutputTest, *criticMinibatchOutputBuffer1);
			m_context->LessEqual(**m_critickOutputTest, *criticMinibatchOutputBuffer);
			m_context->BroadcastScaler(**m_critickInputTest, criticInputSize, **m_critickOutputTest);

			m_context->Blend(*criticMinibatchOutputBuffer, *criticMinibatchOutputBuffer1, **m_critickOutputTest);
			m_context->Blend(*criticMinibatchInputGradientBuffer, *criticMinibatchInputGradientBuffer1, **m_critickInputTest);
		}
	 
		// gradient ascend
		m_context->Scale(*criticMinibatchInputGradientBuffer, ndFloat32(-1.0f));

		ndCopyBufferCommandInfo policyCopyGradients;
		policyCopyGradients.m_srcOffsetInByte = 0;
		policyCopyGradients.m_srcStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		policyCopyGradients.m_dstOffsetInByte = 0;
		policyCopyGradients.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));;
		policyCopyGradients.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));

		ndBrainFloatBuffer* const policyMinibatchOutputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();
		policyMinibatchOutputGradientBuffer->CopyBuffer(policyCopyGradients, m_parameters.m_miniBatchSize, *criticMinibatchInputGradientBuffer);
		m_policyTrainer->BackPropagate();

		m_policyTrainer->ApplyLearnRate();
	}

	m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
	m_policyTrainer->UpdateParameters(m_scratchBuffer);
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::Optimize()
{
	CalculateExpectedRewards();
	for (ndInt32 k = 0; k < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++k)
	{
		LearnQvalueFunction(k);
	}
	
	if (!ndPolycyDelayMod)
	{
		LearnPolicyFunction();
	}
	ndPolycyDelayMod = (ndPolycyDelayMod + 1) % ND_SAC_POLICY_DELAY_MOD;
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::OptimizeStep()
{
	//m_context->SyncBufferCommandQueue();
	SaveTrajectory();
	if (m_startOptimization)
	{
		//m_context->SyncBufferCommandQueue();
		Optimize();
	}

	m_frameCount++;
	m_framesAlive++;
}