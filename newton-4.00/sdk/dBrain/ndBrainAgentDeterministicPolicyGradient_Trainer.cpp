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

#define ND_SAC_POLICY_FIX_SIGMA				ndBrainFloat(0.2f)
#define ND_SAC_POLICY_MIN_PER_ACTION_SIGMA	ndBrainFloat(0.01f)
#define ND_SAC_POLICY_MAX_PER_ACTION_SIGMA	ndBrainFloat(1.0f)
#define ND_SAC_MAX_ENTROPY_COEFFICIENT		ndBrainFloat(2.0e-5f)

#define ND_MINI_FLAT_BUFFER_CACHE			128

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
	m_numberOfHiddenLayers = 3;
	m_policyUpdatesCount = 16;
	m_criticUpdatesCount = 16;
	m_maxTrajectorySteps = 1024 * 4;
	m_replayBufferSize = 1024 * 1024;
	m_hiddenLayersNumberOfNeurons = 256;
	m_replayBufferStartOptimizeSize = m_maxTrajectorySteps * 4;


//m_useGpuBackend = false;
//m_policyUpdatesCount = 1;
//m_criticUpdatesCount = 1;
//m_policyUpdatesCount = 2;
//m_criticUpdatesCount = 2;
//m_hiddenLayersNumberOfNeurons = 256;
m_replayBufferStartOptimizeSize = 100;
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
	return 1;
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetActionOffset() const
{
	return 2;
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetObsevationOffset() const
{
	return 2 + m_actionsSize;
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetNextObsevationOffset() const
{
	return 2 + m_actionsSize + m_obsevationsSize;
}

ndInt32 ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetStride() const
{
	//ndBrainVector m_reward;
	//ndBrainVector m_terminal;
	//ndBrainVector m_actions;
	//ndBrainVector m_observations;
	//ndBrainVector m_nextObservations;
	return 1 + 1 + m_actionsSize + m_obsevationsSize * 2;
}

void ndBrainAgentDeterministicPolicyGradient_Agent::ndTrajectory::GetFlatArray(ndInt32 index, ndBrainVector& output) const
{
	output.SetCount(GetStride());
	output[0] = m_reward[index];
	output[1] = m_terminal[index];
	ndMemCpy(&output[2], &m_actions[index * m_actionsSize], m_actionsSize);
	ndMemCpy(&output[2 + m_actionsSize], &m_observations[index * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&output[2 + m_actionsSize + m_obsevationsSize], &m_nextObservations[index * m_obsevationsSize], m_obsevationsSize);
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
	:ndClassAlloc()
	,m_name()
	,m_parameters(parameters)
	,m_context()
	,m_miniBatchIndices()
	,m_replayBufferFlat()
	,m_replayBuffer()
	,m_replayBufferCache()
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

	m_replayBufferCache.Init(m_policyTrainer->GetBrain()->GetOutputSize(), m_policyTrainer->GetBrain()->GetInputSize());
	m_replayBuffer.Init(m_policyTrainer->GetBrain()->GetOutputSize(), m_policyTrainer->GetBrain()->GetInputSize());
	m_replayBuffer.SetCount(parameters.m_replayBufferSize);
	m_replayBuffer.SetCount(0);

	m_replayBufferFlat = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_replayBuffer.GetStride() * m_parameters.m_replayBufferSize));
	m_replayFlatBufferCache = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_replayBuffer.GetStride() * ND_MINI_FLAT_BUFFER_CACHE));
	m_rewardBatch = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_policyTrainer->GetBrain()->GetOutputSize() * m_parameters.m_miniBatchSize));
	m_terminalBatch = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_policyTrainer->GetBrain()->GetOutputSize() * m_parameters.m_miniBatchSize));
	m_expectedRewards = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize));

	m_minibatchIndexBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_randomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize));

#if 0
	ndCopyBufferCommandInfo policyInput;
	policyInput.m_srcOffsetInByte = 0;
	policyInput.m_dstOffsetInByte = ndInt32(m_replayBuffer.GetNextObsevationOffset() * sizeof(ndReal));
	policyInput.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyInput.m_srcStrideInByte = ndInt32(m_replayBuffer.GetStride() * sizeof(ndReal));
	policyInput.m_dstStrideInByte = policyInput.m_strideInByte;
	m_policyNextObservationParameters = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(*m_context, sizeof(ndCopyBufferCommandInfo), &policyInput));

	ndCopyBufferCommandInfo replayCacheBufferInfo;
	replayCacheBufferInfo.m_srcOffsetInByte = 0;
	replayCacheBufferInfo.m_dstOffsetInByte = 0;
	replayCacheBufferInfo.m_strideInByte = ndInt32(ND_MINI_FLAT_BUFFER_CACHE * m_replayBuffer.GetStride() * sizeof (ndReal));
	replayCacheBufferInfo.m_srcStrideInByte = replayCacheBufferInfo.m_strideInByte;
	replayCacheBufferInfo.m_dstStrideInByte = replayCacheBufferInfo.m_strideInByte;
	m_replayFlatBufferCacheParameters = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(*m_context, sizeof(ndCopyBufferCommandInfo), &replayCacheBufferInfo, true));
	
	ndCopyBufferCommandInfo crictiActionInputBufferInfo;
	crictiActionInputBufferInfo.m_srcOffsetInByte = 0;
	crictiActionInputBufferInfo.m_dstOffsetInByte = 0;
	crictiActionInputBufferInfo.m_strideInByte = ndInt32(m_referenceCriticTrainer[0]->GetBrain()->GetInputSize() * sizeof(ndReal));
	crictiActionInputBufferInfo.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
	crictiActionInputBufferInfo.m_dstStrideInByte = crictiActionInputBufferInfo.m_strideInByte;
	m_criticNextActionParameters = ndSharedPtr<ndBrainBuffer>(new ndBrainUniformBuffer(*m_context, sizeof(ndCopyBufferCommandInfo), &crictiActionInputBufferInfo));
	
	ndCopyBufferCommandInfo crictiObservationInputBufferInfo;
	crictiObservationInputBufferInfo.m_srcOffsetInByte = 0;
	crictiObservationInputBufferInfo.m_dstOffsetInByte = ndInt32(m_replayBuffer.GetActionOffset() * sizeof(ndReal));
	crictiObservationInputBufferInfo.m_strideInByte = ndInt32(m_referenceCriticTrainer[0]->GetBrain()->GetInputSize() * sizeof(ndReal));
	crictiObservationInputBufferInfo.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	crictiObservationInputBufferInfo.m_dstStrideInByte = crictiObservationInputBufferInfo.m_strideInByte;
	m_criticNextObservationParameters = ndSharedPtr<ndBrainBuffer>(new ndBrainUniformBuffer(*m_context, sizeof(ndCopyBufferCommandInfo), &crictiObservationInputBufferInfo));

	ndBrainFloat unitEntropy = ndClamp(m_parameters.m_entropyRegularizerCoef, ndBrainFloat(0.0f), ndBrainFloat(1.0f));
	m_parameters.m_entropyRegularizerCoef = ND_SAC_MAX_ENTROPY_COEFFICIENT * unitEntropy;
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
	{
		m_rewardBatch[i].SetCount(m_parameters.m_miniBatchSize);
	}
#endif
}

ndBrainAgentDeterministicPolicyGradient_Trainer::~ndBrainAgentDeterministicPolicyGradient_Trainer()
{
}

ndBrain* ndBrainAgentDeterministicPolicyGradient_Trainer::GetPolicyNetwork()
{
	ndBrainFloatBuffer* const weightdAndBiasBuffer = m_policyTrainer->GetWeightAndBiasBuffer();
	weightdAndBiasBuffer->VectorFromDevice(m_parametersBuffer);
	m_policyTrainer->UpdateParameters(m_parametersBuffer);
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
	
		for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
		{
			ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ND_SAC_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
		}
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
		// this does not make it better, as usual my intuition fails again 
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
		//m_referenceCriticTrainer[k] = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
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

#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::SaveTrajectory()
{
	// remove all dead states except the last 
	bool isTerminal = false;
	while (m_agent->m_trajectory.GetTerminalState(m_agent->m_trajectory.GetCount() - 2))
	{
		m_agent->m_trajectory.SetCount(m_agent->m_trajectory.GetCount() - 1);
	}
	if (m_agent->m_trajectory.GetTerminalState(m_agent->m_trajectory.GetCount() - 1))
	{
		// add a sentinel state to get the terminal
		isTerminal = true;
		m_agent->m_trajectory.SetCount(m_agent->m_trajectory.GetCount() + 1);
	}
	
	auto AddTransition = [this](ndInt32 dstIndex, ndInt32 srcIndex)
	{
		ndInt32 dstCacheIndex = m_replayBufferCache.GetCount();
		m_replayBufferCache.SetCount(dstCacheIndex + 1);
		m_replayBufferCache.CopyFrom(dstCacheIndex, m_agent->m_trajectory, srcIndex);
		
		const ndBrain& brain = **m_policyTrainer->GetBrain();
		ndBrainMemVector nextObservation(m_replayBufferCache.GetNextObservations(dstCacheIndex), brain.GetInputSize());
		nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(srcIndex + 1), brain.GetInputSize()));
		
		ndInt32 count = 0;
		if (m_replayBufferCache.GetCount() >= ND_MINI_FLAT_BUFFER_CACHE)
		{
			ndInt32 stride = m_replayBufferCache.GetStride();
			ndAssert(m_replayBufferCache.GetCount() == ND_MINI_FLAT_BUFFER_CACHE);
			m_parametersBuffer.SetCount(stride * m_replayBufferCache.GetCount());
			
			m_replayBuffer.SetCount(dstIndex + m_replayBufferCache.GetCount());
			for (ndInt32 i = 0; i < m_replayBufferCache.GetCount(); ++i)
			{
				m_shuffleBuffer.PushBack(dstIndex + i);
				m_replayBuffer.CopyFrom(dstIndex + i, m_replayBufferCache, i);
			
				ndBrainMemVector mem(&m_parametersBuffer[i * stride], stride);
				m_replayBufferCache.GetFlatArray(i, mem);
			}
			
			size_t sizeInBytes = sizeof(ndReal) * stride * m_replayBufferCache.GetCount();
			m_replayFlatBufferCache->MemoryToDevice(0, sizeInBytes, &m_parametersBuffer[0]);
			
			ndCopyBufferCommandInfo replayCacheBufferInfo;
			replayCacheBufferInfo.m_strideInByte = ndInt32 (sizeof(ndReal) * stride);
			replayCacheBufferInfo.m_srcOffsetInByte = 0;
			replayCacheBufferInfo.m_srcStrideInByte = ndInt32(sizeof(ndReal) * stride);
			replayCacheBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal) * stride);
			replayCacheBufferInfo.m_dstOffsetInByte = ndInt32(sizeof(ndReal) * dstIndex * stride);
			m_replayBufferFlat->CopyBuffer(replayCacheBufferInfo, m_replayBufferCache.GetCount(), **m_replayFlatBufferCache);
			
			count = m_replayBufferCache.GetCount();
			m_replayBufferCache.SetCount(0);
		}
		
		return count;
	};
	
	ndInt32 replayBufferCount____ = m_replayBuffer.GetCount();
	ndInt32 numberOfNewTranstitions = m_agent->m_trajectory.GetCount() - m_agent->m_trajectoryBaseCount;
	if ((replayBufferCount____ + numberOfNewTranstitions) < m_parameters.m_replayBufferStartOptimizeSize)
	{
		const ndInt32 numOfTransitions = m_agent->m_trajectory.GetCount() - 1;
		for (ndInt32 i = m_agent->m_trajectoryBaseCount; i < numOfTransitions; ++i)
		{
			replayBufferCount____ += AddTransition(replayBufferCount____, i);
		}
		m_agent->m_trajectoryBaseCount = numOfTransitions;
	}
	else if (!m_startOptimization)
	{
		m_startOptimization = true;
		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	}
	else if ((replayBufferCount____ + numberOfNewTranstitions) < m_parameters.m_replayBufferSize)
	{
		const ndInt32 numOfTransitions = m_agent->m_trajectory.GetCount() - 1;
		for (ndInt32 i = m_agent->m_trajectoryBaseCount; i < numOfTransitions; ++i)
		{
			replayBufferCount____ += AddTransition(replayBufferCount____, i);
		}
		m_agent->m_trajectoryBaseCount = numOfTransitions;
	}
	else if (replayBufferCount____ < m_parameters.m_replayBufferSize)
	{
		ndAssert(0);
	//	const ndInt32 numOfTransitions = m_parameters.m_replayBufferSize - replayBufferCount;
	//	for (ndInt32 i = 0; i < numOfTransitions; ++i)
	//	{
	//		AddTransition(replayBufferCount, m_agent->m_trajectoryBaseCount + i);
	//	}
	//	m_agent->m_trajectoryBaseCount += numOfTransitions;
	}
	else
	{
	//	ndAssert(0);
	//	const ndBrain& brain = **m_policyTrainer->GetBrain();
	//	const ndInt32 numOfTransitions = m_agent->m_trajectory.GetCount() - 1;
	//	for (ndInt32 i = m_agent->m_trajectoryBaseCount; i < numOfTransitions; ++i)
	//	{
	//		m_replayBuffer.CopyFrom(m_replayBufferIndex, m_agent->m_trajectory, i);
	//		
	//		ndBrainFloat reward = m_agent->m_trajectory.GetReward(i + 1);
	//		m_replayBuffer.SetReward(m_replayBufferIndex, reward);
	//		
	//		bool terminalState = m_agent->m_trajectory.GetTerminalState(i + 1);
	//		m_replayBuffer.SetTerminalState(m_replayBufferIndex, terminalState);
	//		
	//		ndBrainMemVector nextObservation(m_replayBuffer.GetNextObservations(m_replayBufferIndex), brain.GetInputSize());
	//		nextObservation.Set(ndBrainMemVector(m_agent->m_trajectory.GetObservations(i + 1), brain.GetInputSize()));
	//
	//		m_replayBufferIndex = (m_replayBufferIndex + 1) % m_parameters.m_replayBufferSize;
	//	}
	//	m_agent->m_trajectoryBaseCount = numOfTransitions;
	}
	
	//bool terminalState = m_agent->m_trajectory.GetTerminalState(m_agent->m_trajectory.GetCount() - 1);
	if (isTerminal || (m_agent->m_trajectoryBaseCount >= m_parameters.m_maxTrajectorySteps))
	{
		if (m_startOptimization)
		{
			//ndAssert(0);
			//ndTrace(("todo: m_startOptimization\n"))
			m_eposideCount++;
			m_framesAlive = 0;
			m_averageFramesPerEpisodes.Update(ndReal (m_agent->m_trajectoryBaseCount));
	//
	//		// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
	//		ndBrainFloat gamma = m_parameters.m_discountRewardFactor;
	//		ndBrainFloat stateReward = m_agent->m_trajectory.GetReward(m_agent->m_trajectory.GetCount() - 1);
	//		ndBrainFloat averageReward = stateReward;
	//		for (ndInt32 i = m_agent->m_trajectory.GetCount() - 2; i >= 0; --i)
	//		{
	//			ndBrainFloat r = m_agent->m_trajectory.GetReward(i);
	//			stateReward = r + gamma * stateReward;
	//			averageReward += stateReward;
	//		}
	//		averageReward /= ndBrainFloat(m_agent->m_trajectory.GetCount());
	//		m_averageExpectedRewards.Update(averageReward);
		}
	
		m_agent->ResetModel();
		m_agent->m_trajectory.SetCount(0);
		m_agent->m_trajectoryBaseCount = 0;
	}
}

//#pragma optimize( "", off )
//void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnQvalueFunction(ndInt32 criticIndex)
void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnQvalueFunction(ndInt32)
{
	ndAssert(0);
	//const ndBrain& brain = **m_policyTrainer->GetBrain();
	//ndInt32 criticInputSize = brain.GetInputSize() + brain.GetOutputSize();
	//
	//m_criticValue[0].SetCount(m_parameters.m_miniBatchSize);
	//m_criticOutputGradients[0].SetCount(m_parameters.m_miniBatchSize);
	//m_criticObservationActionBatch.SetCount(m_parameters.m_miniBatchSize * criticInputSize);
	//
	//for (ndInt32 n = 0; n < m_parameters.m_criticUpdatesCount; ++n)
	//{
	//	ndAssert(0);
	//	//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	//	//{
	//	//	const ndInt32 index = m_miniBatchIndices[n * m_parameters.m_miniBatchSize + i];
	//	//
	//	//	ndBrainMemVector criticObservationAction(&m_criticObservationActionBatch[i * criticInputSize], criticInputSize);
	//	//	ndMemCpy(&criticObservationAction[0], m_replayBuffer.GetActions(index), brain.GetOutputSize());
	//	//	ndMemCpy(&criticObservationAction[brain.GetOutputSize()], m_replayBuffer.GetObservations(index), brain.GetInputSize());
	//	//}
	//	//m_criticTrainer[criticIndex]->MakePrediction(m_criticObservationActionBatch);
	//	//m_criticTrainer[criticIndex]->GetOutput(m_criticValue[0]);
	//	//
	//	//ndBrainLossLeastSquaredError loss(1);
	//	//ndBrainFixSizeVector<1> groundTruth;
	//	//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	//	//{
	//	//	groundTruth[0] = m_expectedRewards[n * m_parameters.m_miniBatchSize + i];
	//	//	loss.SetTruth(groundTruth);
	//	//	const ndBrainMemVector output(&m_criticValue[0][i], 1);
	//	//	ndBrainMemVector gradient(&m_criticOutputGradients[0][i], 1);
	//	//	loss.GetLoss(output, gradient);
	//	//}
	//	//m_criticTrainer[criticIndex]->BackPropagate(m_criticOutputGradients[0]);
	//	//ndAssert(0);
	//	////m_criticTrainer[criticIndex]->ApplyLearnRate(m_parameters.m_criticLearnRate);
	//}
}

#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::CalculateExpectedRewards()
{
	m_miniBatchIndices.SetCount(0);
	ndInt32 count = m_parameters.m_criticUpdatesCount * m_parameters.m_miniBatchSize;
	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		m_miniBatchIndices.PushBack(m_shuffleBuffer[m_shuffleBatchIndex]);
		m_shuffleBatchIndex++;
		if (m_shuffleBatchIndex >= m_shuffleBuffer.GetCount())
		{
			m_shuffleBatchIndex = 0;
			m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
		}
	}

	
	ndBrainFloatBuffer* criticInputBuffer[ND_SAC_NUMBER_OF_CRITICS];
	ndBrainFloatBuffer* criticOutputBuffer[ND_SAC_NUMBER_OF_CRITICS];
	//ndBrainFloatBuffer* const miniBatchOutputGradientBuffer = m_policyTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();
	
	for (ndInt32 i = 0; i < ND_SAC_NUMBER_OF_CRITICS; ++i)
	{
		criticInputBuffer[i] = m_referenceCriticTrainer[i]->GetInputBuffer();
		criticOutputBuffer[i] = m_referenceCriticTrainer[i]->GetOuputBuffer();
	}

	ndCopyBufferCommandInfo policyObservation;
	policyObservation.m_srcOffsetInByte = ndInt32(m_replayBuffer.GetNextObsevationOffset() * sizeof(ndReal)); 
	policyObservation.m_srcStrideInByte = ndInt32(m_replayBuffer.GetStride() * sizeof(ndReal));
	policyObservation.m_dstOffsetInByte = 0;
	policyObservation.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));

	ndCopyBufferCommandInfo criticInputAction;
	ndInt32 criticInputSize = m_policyTrainer->GetBrain()->GetInputSize() + m_policyTrainer->GetBrain()->GetOutputSize();
	criticInputAction.m_srcOffsetInByte = 0;
	criticInputAction.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
	criticInputAction.m_dstOffsetInByte = 0;
	criticInputAction.m_dstStrideInByte = ndInt32 (criticInputSize * sizeof (ndReal));
	criticInputAction.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));

	ndCopyBufferCommandInfo criticInputObservation;
	criticInputObservation.m_srcOffsetInByte = 0;
	criticInputObservation.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	criticInputObservation.m_dstOffsetInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));;
	criticInputObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
	criticInputObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));

	ndCopyBufferCommandInfo criticOutputReward;
	criticOutputReward.m_srcOffsetInByte = ndInt32(m_replayBuffer.GetRewardOffset() * sizeof(ndReal));
	criticOutputReward.m_srcStrideInByte = ndInt32(m_replayBuffer.GetStride() * sizeof(ndReal));
	criticOutputReward.m_dstOffsetInByte = 0;
	criticOutputReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	criticOutputReward.m_strideInByte = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo criticOutputTerminal;
	criticOutputTerminal.m_srcOffsetInByte = ndInt32(m_replayBuffer.GetTerminalOffset() * sizeof(ndReal));
	criticOutputTerminal.m_srcStrideInByte = ndInt32(m_replayBuffer.GetStride() * sizeof(ndReal));
	criticOutputTerminal.m_dstOffsetInByte = 0;
	criticOutputTerminal.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	criticOutputTerminal.m_strideInByte = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo expectedReward;
	expectedReward.m_srcOffsetInByte = 0;
	expectedReward.m_srcStrideInByte = ndInt32(sizeof(ndReal));
	expectedReward.m_dstOffsetInByte = 0;
	expectedReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	expectedReward.m_strideInByte = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo copyIndicesInfo;
	ndInt32 copyIndicesStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
	copyIndicesInfo.m_dstOffsetInByte = 0;
	copyIndicesInfo.m_srcOffsetInByte = 0;
	copyIndicesInfo.m_strideInByte = copyIndicesStrideInBytes;
	copyIndicesInfo.m_srcStrideInByte = copyIndicesStrideInBytes;
	copyIndicesInfo.m_dstStrideInByte = copyIndicesStrideInBytes;

	// load all the shuffled indices, they use GPU command to get a minibatch
	ndAssert(m_randomShuffleBuffer->SizeInBytes() == m_miniBatchIndices.GetCount() * sizeof(ndInt32));
	m_randomShuffleBuffer->MemoryToDevice(0, m_randomShuffleBuffer->SizeInBytes(), &m_miniBatchIndices[0]);
	for (ndInt32 n = 0; n < m_parameters.m_criticUpdatesCount; ++n)
	{
		// Get the rewards for this minibatch, try dispaching the entire epoch
		//m_context->SyncBufferCommandQueue();

		// for now load the indirect buffer, later this will use a copybuffer
		copyIndicesInfo.m_srcOffsetInByte = ndInt32(n * sizeof(ndUnsigned32) * m_parameters.m_miniBatchSize);
		m_minibatchIndexBuffer->CopyBuffer(copyIndicesInfo, 1, **m_randomShuffleBuffer);

		policyMinibatchInputBuffer->CopyBufferIndirect(policyObservation, **m_minibatchIndexBuffer, **m_replayBufferFlat);
		m_rewardBatch->CopyBufferIndirect(criticOutputReward, **m_minibatchIndexBuffer, **m_replayBufferFlat);
		m_terminalBatch->CopyBufferIndirect(criticOutputTerminal, **m_minibatchIndexBuffer, **m_replayBufferFlat);
		m_policyTrainer->MakePrediction();

		for (ndInt32 i = 0; i < ND_SAC_NUMBER_OF_CRITICS; ++i)
		{
			criticInputBuffer[i]->CopyBuffer(criticInputAction, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);
			criticInputBuffer[i]->CopyBuffer(criticInputObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);
			m_referenceCriticTrainer[i]->MakePrediction();

			ndBrainFloatBuffer& qValueBuffer = *criticOutputBuffer[i];
			m_context->Mul(qValueBuffer, **m_terminalBatch);
			m_context->Scale(qValueBuffer, m_parameters.m_discountRewardFactor);
			m_context->Add(qValueBuffer, **m_rewardBatch);
		}

		ndBrainFloatBuffer& minExpectedReward = *criticOutputBuffer[0];
		for (ndInt32 i = 1; i < ND_SAC_NUMBER_OF_CRITICS; ++i)
		{
			ndBrainFloatBuffer& qValueBuffer1 = *criticOutputBuffer[i];
			m_context->Min(minExpectedReward, qValueBuffer1);
		}
		expectedReward.m_dstOffsetInByte = ndInt32(sizeof(ndReal)) * n * m_parameters.m_miniBatchSize;
		m_expectedRewards->CopyBuffer(expectedReward, m_parameters.m_miniBatchSize, minExpectedReward);
	}
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::LearnPolicyFunction()
{
	ndAssert(0);
	//const ndBrain& brain = **m_policyTrainer->GetBrain();
	//ndInt32 criticInputSize = brain.GetInputSize() + brain.GetOutputSize();
	//
	//m_actionBatch.SetCount(m_parameters.m_miniBatchSize * brain.GetOutputSize());
	//m_obsevationsBatch.SetCount(m_parameters.m_miniBatchSize * brain.GetInputSize());
	//m_policyGradientBatch.SetCount(m_parameters.m_miniBatchSize * brain.GetOutputSize());
	//m_criticObservationActionBatch.SetCount(m_parameters.m_miniBatchSize* criticInputSize);
	//
	//for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
	//{
	//	m_criticValue[i].SetCount(m_parameters.m_miniBatchSize);
	//	m_criticOutputGradients[i].SetCount(m_parameters.m_miniBatchSize);
	//	m_criticInputGradients[i].SetCount(m_parameters.m_miniBatchSize * criticInputSize);
	//}
	//
	//for (ndInt32 n = 0; n < m_parameters.m_policyUpdatesCount; ++n)
	//{
	//	ndAssert(0);
	//	//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	//	//{
	//	//	const ndInt32 index = m_miniBatchIndices[n * m_parameters.m_miniBatchSize + i];
	//	//
	//	//	ndBrainMemVector observation(&m_obsevationsBatch[i * brain.GetInputSize()], brain.GetInputSize());
	//	//	ndMemCpy(&observation[0], m_replayBuffer.GetObservations(index), brain.GetInputSize());
	//	//}
	//	//m_policyTrainer->MakePrediction(m_obsevationsBatch);
	//	//m_policyTrainer->GetOutput(m_actionBatch);
	//	//
	//	//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	//	//{
	//	//	ndBrainMemVector criticObservationAction(&m_criticObservationActionBatch[i * criticInputSize], criticInputSize);
	//	//	ndMemCpy(&criticObservationAction[0], &m_actionBatch[i * brain.GetOutputSize()], brain.GetOutputSize());
	//	//	ndMemCpy(&criticObservationAction[brain.GetOutputSize()], &m_obsevationsBatch[i * brain.GetInputSize()], brain.GetInputSize());
	//	//}
	//	//
	//	//for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
	//	//{
	//	//	m_criticTrainer[i]->MakePrediction(m_criticObservationActionBatch);
	//	//	m_criticTrainer[i]->GetOutput(m_criticValue[i]);
	//	//	m_criticOutputGradients[i].Set(ndBrainFloat(1.0f));
	//	//
	//	//	if (m_parameters.m_entropyRegularizerCoef > ndBrainFloat(1.0e-6f))
	//	//	{
	//	//		ndAssert(0);
	//	//	}
	//	//	m_criticTrainer[i]->BackPropagate(m_criticOutputGradients[i]);
	//	//	m_criticTrainer[i]->GetInput(m_criticInputGradients[i]);
	//	//}
	//	//
	//	//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	//	//{
	//	//	for (ndInt32 j = 1; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	//	//	{
	//	//		if (m_criticValue[j][i] < m_criticValue[0][i])
	//	//		{
	//	//			ndBrainMemVector dstObservationAction(&m_criticInputGradients[0][i * criticInputSize], criticInputSize);
	//	//			const ndBrainMemVector srcObservationAction(&m_criticInputGradients[j][i * criticInputSize], criticInputSize);
	//	//			dstObservationAction.Set(srcObservationAction);
	//	//		}
	//	//	}
	//	//	const ndBrainMemVector srcGradient(&m_criticInputGradients[0][i * criticInputSize], brain.GetOutputSize());
	//	//	ndBrainMemVector policyGradient(&m_policyGradientBatch[i * brain.GetOutputSize()], brain.GetOutputSize());
	//	//	policyGradient.Set(srcGradient);
	//	//}
	//	//m_policyGradientBatch.Scale(ndBrainFloat(-1.0f));
	//	//m_policyTrainer->BackPropagate(m_policyGradientBatch);
	//	//ndAssert(0);
	//	////m_policyTrainer->ApplyLearnRate(m_parameters.m_policyLearnRate);
	//}
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::Optimize()
{
	CalculateExpectedRewards();
	//ndTrace(("todo: ndBrainAgentDeterministicPolicyGradient_Trainer::Optimize()\n"));
	//for (ndInt32 k = 0; k < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++k)
	//{
	//	LearnQvalueFunction(k);
	//}
	//
	//if (!ndPolycyDelayMod)
	//{
	//	LearnPolicyFunction();
	//}
	//for (ndInt32 k = 0; k < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++k)
	//{
	//	ndAssert(0);
	//	m_referenceCriticTrainer[k]->SoftCopyParameters(**m_criticTrainer[k], m_parameters.m_criticMovingAverageFactor);
	//}
	//ndPolycyDelayMod = (ndPolycyDelayMod + 1) % ND_SAC_POLICY_DELAY_MOD;
}

//#pragma optimize( "", off )
void ndBrainAgentDeterministicPolicyGradient_Trainer::OptimizeStep()
{
	//ndTrace(("todo: ndBrainAgentDeterministicPolicyGradient_Trainer::OptimizeStep()\n"))
	m_context->SyncBufferCommandQueue();

	SaveTrajectory();
	if (m_startOptimization)
	{
		Optimize();
	}
	m_frameCount++;
	m_framesAlive++;
}