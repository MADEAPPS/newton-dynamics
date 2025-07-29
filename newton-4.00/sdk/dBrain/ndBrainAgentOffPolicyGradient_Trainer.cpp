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
#include "ndBrainAgentOffPolicyGradient_Trainer.h"

#define ND_HIDEN_LAYERS_ACTIVATION				ndBrainLayerActivationRelu
//#define ND_HIDEN_LAYERS_ACTIVATION			ndBrainLayerActivationTanh
//#define ND_HIDEN_LAYERS_ACTIVATION			ndBrainLayerActivationLeakyRelu
 
//#define ND_SAC_MAX_ENTROPY_COEFFICIENT		ndBrainFloat(2.0e-5f)

#define ND_POLICY_CONSTANT_SIGMA				ndBrainFloat(0.5f)
#define ND_POLICY_MIN_SIGMA						(ndBrainFloat(0.5f) * ND_POLICY_CONSTANT_SIGMA)
#define ND_POLICY_MAX_SIGMA						(ndBrainFloat(2.0f) * ND_POLICY_CONSTANT_SIGMA)

#define ND_POLICY_TRAINING_EXPLORATION_NOISE	ndBrainFloat(0.2f)

ndBrainAgentOffPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = ndBrainFloat(1.0e-4f);
	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-4f);
	m_discountRewardFactor = ndBrainFloat(0.99f);

	m_entropyRegularizerCoef = ndBrainFloat(0.0f);
	//m_entropyRegularizerCoef = ndBrainFloat(0.25f);
	m_movingAverageFactor = ndBrainFloat(0.005f);

	m_useGpuBackend = true;
	//m_usePerActionSigmas = false;
	m_usePerActionSigmas = true;
	m_actionFixSigma = ND_POLICY_CONSTANT_SIGMA;

	m_policyRegularizerType = m_ridge;
	m_criticRegularizerType = m_ridge;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_randomSeed = 47;
	m_numberOfUpdates = 16;
	m_numberOfHiddenLayers = 3;
	m_maxTrajectorySteps = 1024 * 4;
	m_replayBufferSize = 1024 * 1024;
	m_hiddenLayersNumberOfNeurons = 256;
	m_replayBufferStartOptimizeSize = 1024 * 64;

//m_useGpuBackend = false;
//m_miniBatchSize = 16;
//m_miniBatchSize = 128;
//m_hiddenLayersNumberOfNeurons = 64;
//m_replayBufferStartOptimizeSize = 1024 * 8;
}

class ndBrainAgentOffPolicyGradient_Trainer::ndActivation : public ndBrainLayerActivation
{
	public:
	ndActivation(ndInt32 neurons)
		:ndBrainLayerActivation(neurons)
	{
	}

	ndActivation(const ndActivation& src)
		:ndBrainLayerActivation(src)
	{
	}

	virtual ndBrainLayer* Clone() const override
	{
		return new ndActivation(*this);
	}

	static ndBrainLayer* Load(const ndBrainLoad* const loadSave)
	{
		char buffer[1024];
		loadSave->ReadString(buffer);

		loadSave->ReadString(buffer);
		ndInt32 inputs = loadSave->ReadInt();
		ndActivation* const layer = new ndActivation(inputs);
		loadSave->ReadString(buffer);
		return layer;
	}

	virtual const char* GetLabelId() const override
	{
		return ND_OFF_POLICY_ACTIVATION_NAME;
	}

	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override
	{
		ndAssert(input.GetCount() == output.GetCount());
		const ndInt32 size = ndInt32 (input.GetCount() / 2);
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			output[i] = input[i];
			output[size + i] = ndExp_VSFix(input[size + i]);
		}
	}

	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override
	{
		ndAssert(input.GetCount() == output.GetCount());
		ndAssert(input.GetCount() == outputDerivative.GetCount());
		const ndInt32 size = ndInt32(input.GetCount() / 2);
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			inputDerivative[i] = outputDerivative[i];
			inputDerivative[size + i] = output[size + i] * outputDerivative[size + i];
		}
	}

	virtual bool HasGpuSupport() const override
	{
		return true;
	}

	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const override
	{
		const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
		const ndCommandSharedInfo& info = desc.m_info;
		ndBrainTrainerInference* const trainer = desc.m_owner;
		const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();

		ndInt32 inputSize = info.m_inputSize;
		ndInt32 outputSize = info.m_outputSize;
		ndInt32 inputOutputSize = info.m_inputOutputSize;
		ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

		ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
		ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);

		const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
		ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);

		ndAssert(input.GetCount() == output.GetCount());
		const ndInt32 size = ndInt32(input.GetCount() / 2);
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			output[i] = input[i];
			output[size + i] = ndExp_VSFix(input[size + i]);
		}
	}

	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const override
	{
		const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
		const ndCommandSharedInfo& info = desc.m_info;
		ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

		const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
		const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();

		ndInt32 inputSize = info.m_inputSize;
		ndInt32 inputOutputSize = info.m_inputOutputSize;
		ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

		ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
		ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
		ndAssert(srcBase >= 0);
		ndAssert(dstBase >= 0);
		ndAssert(inputSize == info.m_outputSize);

		const ndBrainMemVector output(&inputOutputBuffer[dstBase], inputSize);
		const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], inputSize);
		ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);

		ndAssert(inputDerivative.GetCount() == output.GetCount());
		ndAssert(inputDerivative.GetCount() == outputDerivative.GetCount());
		const ndInt32 size = ndInt32(output.GetCount() / 2);
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			inputDerivative[i] = outputDerivative[i];
			inputDerivative[size + i] = output[size + i] * outputDerivative[size + i];
		}
	}

	virtual ndCommandArray CreateGpuFeedForwardCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const override
	{
		ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
			owner, context, info, miniBatchSize, 0,
			inputOutputData, weightsAndBias));

		ndBrainBufferCommand* command = nullptr;
		if (context->GetAsCpuContext())
		{
			command = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
		}
		else
		{
			descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerOffPolicyActivation;
			command = new ndBrainGpuCommand(descriptor);
		}
		ndCommandArray commandArray(0);
		commandArray.PushBack(command);
		return commandArray;
	}

	virtual ndCommandArray CreateGpuBackPropagateCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias,
		ndBrainFloatBuffer* const inputOutputGradients,
		ndBrainFloatBuffer* const weightsAndBiasGradients) const override
	{
		ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
			owner, context, info, miniBatchSize, 0,
			inputOutputData, weightsAndBias,
			inputOutputGradients, weightsAndBiasGradients));

		ndCommandArray commands(0);

		if (context->GetAsCpuContext())
		{
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}
		else
		{
			descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerOffPolicyBackPropagate;
			ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
			commands.PushBack(command);
		}
		return commands;
	}
};

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

ndBrainAgentOffPolicyGradient_Agent::ndBrainAgentOffPolicyGradient_Agent(const ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master)
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

ndBrainAgentOffPolicyGradient_Agent::~ndBrainAgentOffPolicyGradient_Agent()
{
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
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainAgentOffPolicyGradient_Trainer* const owner = *m_owner;

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

ndBrainAgentOffPolicyGradient_Trainer::ndBrainAgentOffPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndClassAlloc()
	,m_name()
	,m_parameters(parameters)
	,m_context()
	,m_agent(nullptr)
	,m_randomGenerator(std::random_device{}())
	,m_uniformDistribution(ndFloat32(0.0f), ndFloat32(1.0f))
	,m_minibatchMean(nullptr)
	,m_minibatchSigma(nullptr)
	,m_uniformRandom0(nullptr)
	,m_replayBufferFlat(nullptr)
	,m_minibatchRewards(nullptr)
	,m_minibatchNoTerminal(nullptr)
	,m_minibatchOfTransitions(nullptr)
	,m_minibatchExpectedRewards(nullptr)
	,m_minibatchCriticInputTest(nullptr)
	,m_minibatchUniformRandomDistribution(nullptr)
	,m_randomShuffleBuffer(nullptr)
	,m_minibatchIndexBuffer(nullptr)
	,m_scratchBuffer()
	,m_shuffleBuffer()
	,m_miniBatchIndices()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_policyDelayMod(0)
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
	ndBrainAgentOffPolicyGradient_Agent::ndTrajectory trajectory(outputSize, inputSize);

	m_minibatchRewards = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchNoTerminal = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchIndexBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchExpectedRewards = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));

	m_replayBufferFlat = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_replayBufferSize * trajectory.GetStride()));
	m_minibatchOfTransitions = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * trajectory.GetStride()));
	m_minibatchCriticInputTest = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, (outputSize + inputSize) * m_parameters.m_miniBatchSize));
	m_randomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_numberOfUpdates * m_parameters.m_miniBatchSize));

	m_minibatchMean = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize / 2));
	m_minibatchSigma = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize / 2));
	m_minibatchMeanAction = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize / 2));
	m_minibatchUniformRandomDistribution = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_miniBatchSize / 2));
	m_uniformRandom0 = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, outputSize * m_parameters.m_numberOfUpdates * m_parameters.m_miniBatchSize / 2));
}

ndBrainAgentOffPolicyGradient_Trainer::~ndBrainAgentOffPolicyGradient_Trainer()
{
}

ndBrainLayer* ndBrainAgentOffPolicyGradient_Trainer::LoadActivation(const ndBrainLoad* const loadSave)
{
	return ndActivation::Load(loadSave);
}

ndBrain* ndBrainAgentOffPolicyGradient_Trainer::GetPolicyNetwork()
{
	return *m_policyTrainer->GetBrain();
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
	//ndInt32 numberOfOutput = m_parameters.m_usePerActionSigmas ? 2 * m_parameters.m_numberOfActions : m_parameters.m_numberOfActions;
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions * 2));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	ndBrainVector bias;
	ndBrainVector slope;
	bias.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
	slope.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());

	bias.Set(ndBrainFloat(0.0f));
	slope.Set(ndBrainFloat(1.0f));
	ndInt32 elements = ndInt32(bias.GetCount() / 2);
	ndBrainMemVector biasVariance(&bias[elements], elements);
	ndBrainMemVector slopeVariance(&slope[elements], elements);
	if (m_parameters.m_usePerActionSigmas)
	{
		ndBrainFloat minLogSigma = ndLog(ND_POLICY_MIN_SIGMA);
		ndBrainFloat maxLogSigma = ndLog(ND_POLICY_MAX_SIGMA);
		biasVariance.Set((maxLogSigma + minLogSigma) * ndBrainFloat(0.5f));
		slopeVariance.Set((maxLogSigma - minLogSigma) * ndBrainFloat(0.5f));
	}
	else
	{
		slopeVariance.Set(ndBrainFloat(0.0f));
		biasVariance.Set(ndLog (m_parameters.m_actionFixSigma));
	}
	layers.PushBack(new ndBrainLayerActivationLinear(slope, bias));
	layers.PushBack(new ndActivation(layers[layers.GetCount() - 1]->GetOutputSize()));

	ndSharedPtr<ndBrain> policy (new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		policy->AddLayer(layers[i]);
	}
	policy->InitWeights();

	ndSharedPtr<ndBrain> referencePolicy(new ndBrain(**policy));
	ndTrainerDescriptor referenceDescriptor(referencePolicy, m_context, m_parameters.m_miniBatchSize, m_parameters.m_policyLearnRate);
	referenceDescriptor.m_regularizer = m_parameters.m_policyRegularizer;
	referenceDescriptor.m_regularizerType = m_parameters.m_policyRegularizerType;
	m_referencePolicyTrainer = ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainer(referenceDescriptor));

	ndTrainerDescriptor descriptor(policy, m_context, m_parameters.m_miniBatchSize, m_parameters.m_policyLearnRate);
	descriptor.m_regularizer = m_parameters.m_policyRegularizer;
	descriptor.m_regularizerType = m_parameters.m_policyRegularizerType;
	m_policyTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
}

void ndBrainAgentOffPolicyGradient_Trainer::BuildCriticClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	const ndBrain& policy = **m_policyTrainer->GetBrain();
	for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	{
		layers.SetCount(0);
		layers.PushBack(new ndBrainLayerLinear(policy.GetOutputSize() + policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	
		for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers-1; ++i)
		{
			ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
			layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
			layers.PushBack(new ND_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
		}

		// prevent exploding gradients. 
		// it does not seem to make a diference bu the weighs and bias are much smaller. 
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

		ndSharedPtr<ndBrain> referenceCritic(new ndBrain(**critic));
		ndTrainerDescriptor referenceDescriptor(referenceCritic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_policyLearnRate);
		referenceDescriptor.m_regularizer = m_parameters.m_criticRegularizer;
		referenceDescriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_referenceCriticTrainer[j] = ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainerInference(referenceDescriptor));

		ndTrainerDescriptor descriptor(critic, m_context, m_parameters.m_miniBatchSize, m_parameters.m_policyLearnRate);
		descriptor.m_regularizer = m_parameters.m_criticRegularizer;
		descriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_criticTrainer[j] = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor));
	}
}

//ndBrainFloat ndBrainAgentOffPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution) const
ndBrainFloat ndBrainAgentOffPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32, const ndBrainVector&) const
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

//ndBrainFloat ndBrainAgentOffPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32 index) const
ndBrainFloat ndBrainAgentOffPolicyGradient_Trainer::CalculatePolicyProbability(ndInt32) const
{
	ndAssert(0);
	return 0;
	//const ndBrainMemVector sampledProbabilities(m_replayBuffer.GetActions(index), m_policy.GetOutputSize());
	//return CalculatePolicyProbability(index, sampledProbabilities);
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
	return m_averageExpectedRewards.GetAverage();
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
		ndInt32 horizonSteps = ndInt32 (ndFloat32(1.0f) / (ndFloat32(1.0f) - m_parameters.m_discountRewardFactor));
		numberOfSteps -= horizonSteps;
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

void ndBrainAgentOffPolicyGradient_Trainer::SaveTrajectoryLoadBuffer()
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
	SaveTrajectoryLoadBuffer();

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
	m_context->CopyBuffer(*criticMinibatchOutputGradientBuffer, *criticMinibatchOutputBuffer);
	m_context->Sub(*criticMinibatchOutputGradientBuffer, **m_minibatchExpectedRewards);
	
	// backpropagete thes loss
	critic.BackPropagate();
	critic.ApplyLearnRate();
}

//#pragma optimize( "", off )
void ndBrainAgentOffPolicyGradient_Trainer::CalculateExpectedRewards()
{
	// Get the rewards for this minibatch, try dispaching the entire epoch
	ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_referencePolicyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_referencePolicyTrainer->GetOuputBuffer();

	const ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	ndInt32 transitionStrideInBytes = ndInt32(trajectory.GetStride() * sizeof(ndReal));

	ndCopyBufferCommandInfo policyNextObservation;
	policyNextObservation.m_srcStrideInByte = transitionStrideInBytes;
	policyNextObservation.m_srcOffsetInByte = ndInt32(trajectory.GetNextObsevationOffset() * sizeof(ndReal));
	policyNextObservation.m_dstOffsetInByte = 0;
	policyNextObservation.m_dstStrideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyNextObservation.m_strideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyMinibatchInputBuffer->CopyBuffer(policyNextObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
	m_referencePolicyTrainer->MakePrediction();

	ndCopyBufferCommandInfo minibatchSigma;
	minibatchSigma.m_srcOffsetInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
	minibatchSigma.m_srcStrideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
	minibatchSigma.m_dstOffsetInByte = 0;
	minibatchSigma.m_dstStrideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
	minibatchSigma.m_strideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
	m_minibatchSigma->CopyBuffer(minibatchSigma, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

	ndCopyBufferCommandInfo minibatchMean;
	minibatchSigma.m_srcOffsetInByte = 0;
	minibatchSigma.m_srcStrideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
	minibatchSigma.m_dstOffsetInByte = 0;
	minibatchSigma.m_dstStrideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
	minibatchSigma.m_strideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
	m_minibatchMeanAction->CopyBuffer(minibatchSigma, m_parameters.m_miniBatchSize, *policyMinibatchOutputBuffer);

	m_context->GaussianSample(**m_minibatchUniformRandomDistribution, **m_minibatchSigma, **m_minibatchUniformRandomDistribution);

	// clip exploration noise
	m_context->Min(**m_minibatchUniformRandomDistribution, ND_POLICY_TRAINING_EXPLORATION_NOISE);
	m_context->Max(**m_minibatchUniformRandomDistribution, -ND_POLICY_TRAINING_EXPLORATION_NOISE);

	m_context->Add(**m_minibatchMeanAction, **m_minibatchUniformRandomDistribution);
	m_context->Min(**m_minibatchMeanAction, ndBrainFloat(1.0f));
	m_context->Max(**m_minibatchMeanAction, ndBrainFloat(-1.0f));

	ndCopyBufferCommandInfo criticOutputReward;
	criticOutputReward.m_srcOffsetInByte = ndInt32(trajectory.GetRewardOffset() * sizeof(ndReal));
	criticOutputReward.m_srcStrideInByte = transitionStrideInBytes;
	criticOutputReward.m_dstOffsetInByte = 0;
	criticOutputReward.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	criticOutputReward.m_strideInByte = ndInt32(sizeof(ndReal));
	m_minibatchRewards->CopyBuffer(criticOutputReward, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
	
	ndCopyBufferCommandInfo criticOutputTerminal;
	criticOutputTerminal.m_srcOffsetInByte = ndInt32(trajectory.GetTerminalOffset() * sizeof(ndReal));
	criticOutputTerminal.m_srcStrideInByte = transitionStrideInBytes;
	criticOutputTerminal.m_dstOffsetInByte = 0;
	criticOutputTerminal.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	criticOutputTerminal.m_strideInByte = ndInt32(sizeof(ndReal));
	m_minibatchNoTerminal->CopyBuffer(criticOutputTerminal, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);

	ndInt32 criticInputSize = m_referencePolicyTrainer->GetBrain()->GetInputSize() + m_referencePolicyTrainer->GetBrain()->GetOutputSize();
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
	{
		ndBrainTrainerInference& referenceCritic = **m_referenceCriticTrainer[i];
		ndBrainFloatBuffer& criticInputBuffer = *referenceCritic.GetInputBuffer();
	
		ndCopyBufferCommandInfo criticInputNextActionMean;
		criticInputNextActionMean.m_srcOffsetInByte = 0;
		criticInputNextActionMean.m_srcStrideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
		criticInputNextActionMean.m_dstOffsetInByte = 0;
		criticInputNextActionMean.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputNextActionMean.m_strideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
		criticInputBuffer.CopyBuffer(criticInputNextActionMean, m_parameters.m_miniBatchSize, **m_minibatchMeanAction);

		ndCopyBufferCommandInfo criticInputNextActionSigma;
		criticInputNextActionSigma.m_srcOffsetInByte = 0;
		criticInputNextActionSigma.m_srcStrideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
		criticInputNextActionSigma.m_dstOffsetInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
		criticInputNextActionSigma.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputNextActionSigma.m_strideInByte = ndInt32(m_referencePolicyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal) / 2);
		criticInputBuffer.CopyBuffer(criticInputNextActionSigma, m_parameters.m_miniBatchSize, **m_minibatchSigma);

		ndCopyBufferCommandInfo criticInputNextObservation;
		criticInputNextObservation.m_srcOffsetInByte = 0;
		criticInputNextObservation.m_srcStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
		criticInputNextObservation.m_dstStrideInByte = ndInt32(criticInputSize * sizeof(ndReal));
		criticInputNextObservation.m_dstOffsetInByte = ndInt32(m_policyTrainer->GetBrain()->GetOutputSize() * sizeof(ndReal));
		criticInputNextObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
		criticInputBuffer.CopyBuffer(criticInputNextObservation, m_parameters.m_miniBatchSize, *policyMinibatchInputBuffer);
		m_referenceCriticTrainer[i]->MakePrediction();

		ndBrainFloatBuffer& qValueBuffer = *referenceCritic.GetOuputBuffer();
		m_context->Mul(qValueBuffer, **m_minibatchNoTerminal);
		m_context->Scale(qValueBuffer, m_parameters.m_discountRewardFactor);
		m_context->Add(qValueBuffer, **m_minibatchRewards);
	}
		
	ndBrainFloatBuffer& minExpectedReward = *m_referenceCriticTrainer[0]->GetOuputBuffer();
	for (ndInt32 i = 1; i < ndInt32(sizeof(m_criticTrainer) / sizeof(m_criticTrainer[0])); ++i)
	{
		const ndBrainFloatBuffer& qValueBuffer1 = *m_referenceCriticTrainer[i]->GetOuputBuffer();
		m_context->Min(minExpectedReward, qValueBuffer1);
	}
	m_context->Set(**m_minibatchExpectedRewards, minExpectedReward);
}

//#pragma optimize( "", off )
void ndBrainAgentOffPolicyGradient_Trainer::TrainPolicy()
{
	const ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
		
	ndInt32 criticInputSize = m_policyTrainer->GetBrain()->GetInputSize() + m_policyTrainer->GetBrain()->GetOutputSize();

	ndCopyBufferCommandInfo policyObservation;
	policyObservation.m_srcStrideInByte = ndInt32(trajectory.GetStride() * sizeof(ndReal));
	policyObservation.m_srcOffsetInByte = ndInt32(trajectory.GetObsevationOffset() * sizeof(ndReal));
	policyObservation.m_dstOffsetInByte = 0;
	policyObservation.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyObservation.m_strideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	ndBrainFloatBuffer* const policyMinibatchInputBuffer = m_policyTrainer->GetInputBuffer();
	policyMinibatchInputBuffer->CopyBuffer(policyObservation, m_parameters.m_miniBatchSize, **m_minibatchOfTransitions);
	m_policyTrainer->MakePrediction();

#if 1
	// original paper uses the first critic
	ndBrainFloatBuffer* const policyMinibatchOutputBuffer = m_policyTrainer->GetOuputBuffer();

	ndBrainTrainer& critic = **m_criticTrainer[0];
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

	//the gradient is just 1.0
	ndBrainFloatBuffer* const criticMinibatchInputGradientBuffer = critic.GetInputGradientBuffer();
	ndBrainFloatBuffer* const criticMinibatchOutputGradientBuffer = critic.GetOuputGradientBuffer();
	m_context->Set(*criticMinibatchOutputGradientBuffer, ndBrainFloat(1.0f));
	critic.BackPropagate();
	
#else
	// using the critic with the lower gradient
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
	
		// re using m_minibatchRewards because is not use anymore 
		m_context->Set(**m_minibatchRewards, *criticMinibatchOutputBuffer1);
		m_context->LessEqual(**m_minibatchRewards, *criticMinibatchOutputBuffer);
		m_context->BroadcastScaler(**m_minibatchCriticInputTest, criticInputSize, **m_minibatchRewards);
	
		m_context->Blend(*criticMinibatchOutputBuffer, *criticMinibatchOutputBuffer1, **m_minibatchRewards);
		m_context->Blend(*criticMinibatchInputGradientBuffer, *criticMinibatchInputGradientBuffer1, **m_minibatchCriticInputTest);
	}
 #endif

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

//#pragma optimize( "", off )
void ndBrainAgentOffPolicyGradient_Trainer::Optimize()
{
	// get the number of indirect transitions 
	m_miniBatchIndices.SetCount(0);
	ndInt32 numberOfSamples = m_parameters.m_numberOfUpdates * m_parameters.m_miniBatchSize;

	if ((numberOfSamples + m_shuffleBatchIndex) >= (m_shuffleBuffer.GetCount() - numberOfSamples))
	{
		// re shuffle after every pass.
		m_shuffleBatchIndex = 0;
		m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	}
	for (ndInt32 i = numberOfSamples - 1; i >= 0; --i)
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

	// load all the shuffled indices, they use GPU command to get a minibatch
	ndAssert(m_randomShuffleBuffer->SizeInBytes() == m_miniBatchIndices.GetCount() * sizeof(ndInt32));
	m_randomShuffleBuffer->MemoryToDevice(0, m_randomShuffleBuffer->SizeInBytes(), &m_miniBatchIndices[0]);

	// get a vector of randon numbers
	m_scratchBuffer.SetCount(0);

	ndInt32 policyOutputSize = m_referencePolicyTrainer->GetBrain()->GetOutputSize();
	for (ndInt32 i = numberOfSamples - 1; i >= 0; --i)
	{
		for (ndInt32 j = policyOutputSize / 2 - 1; j >= 0; --j)
		{
			m_scratchBuffer.PushBack(m_uniformDistribution(m_randomGenerator));
		}
	}
	m_uniformRandom0->VectorToDevice(m_scratchBuffer);

	const ndBrainAgentOffPolicyGradient_Agent::ndTrajectory& trajectory = m_agent->m_trajectory;
	ndInt32 transitionSizeInBytes = ndInt32(trajectory.GetStride() * sizeof(ndInt32));
	ndInt32 copyIndicesStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));

	ndInt32 twinDelayModule = ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0]));
	for (ndInt32 i = 0; i < m_parameters.m_numberOfUpdates; ++i)
	{
		// sample a random minibatch of transitions
		ndCopyBufferCommandInfo copyIndicesInfo;
		copyIndicesInfo.m_dstOffsetInByte = 0;
		copyIndicesInfo.m_dstStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_srcOffsetInByte = ndInt32(i * copyIndicesStrideInBytes);
		copyIndicesInfo.m_srcStrideInByte = copyIndicesStrideInBytes;
		copyIndicesInfo.m_strideInByte = copyIndicesStrideInBytes;
		m_minibatchIndexBuffer->CopyBuffer(copyIndicesInfo, 1, **m_randomShuffleBuffer);

		ndCopyBufferCommandInfo minibatchOfTransitions;
		minibatchOfTransitions.m_dstOffsetInByte = 0;
		minibatchOfTransitions.m_dstStrideInByte = transitionSizeInBytes;
		minibatchOfTransitions.m_srcOffsetInByte = ndInt32(i * transitionSizeInBytes * m_parameters.m_miniBatchSize);
		minibatchOfTransitions.m_srcStrideInByte = transitionSizeInBytes;
		minibatchOfTransitions.m_strideInByte = transitionSizeInBytes;
		m_minibatchOfTransitions->CopyBufferIndirect(minibatchOfTransitions, **m_minibatchIndexBuffer, **m_replayBufferFlat);

		// get a minibatch of uniform distributed random numbe form 0.0 to 1.0
		ndCopyBufferCommandInfo minibatchReparametization;
		ndInt32 strideSizeInBytes = ndInt32(sizeof (ndInt32)) * policyOutputSize * m_parameters.m_miniBatchSize / 2;
		minibatchReparametization.m_dstOffsetInByte = 0;
		minibatchReparametization.m_dstStrideInByte = strideSizeInBytes;
		minibatchReparametization.m_srcOffsetInByte = i * strideSizeInBytes;
		minibatchReparametization.m_srcStrideInByte = strideSizeInBytes;
		minibatchReparametization.m_strideInByte = strideSizeInBytes;
		m_minibatchUniformRandomDistribution->CopyBuffer(copyIndicesInfo, 1, **m_uniformRandom0);

		CalculateExpectedRewards();
		for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
		{
			TrainCritics(j);
		}

		m_policyDelayMod = (m_policyDelayMod + 1) % twinDelayModule;
		if (!m_policyDelayMod)
		{
			TrainPolicy();

			for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
			{
				ndBrainTrainer* const criticTrainer = *m_criticTrainer[j];
				ndBrainTrainerInference* const referenceCritic = *m_referenceCriticTrainer[j];

				const ndBrainFloatBuffer* const parameterBuffer = criticTrainer->GetWeightAndBiasBuffer();
				ndBrainFloatBuffer* const referenceParameterBuffer = referenceCritic->GetWeightAndBiasBuffer();
				m_context->Blend(*referenceParameterBuffer, *parameterBuffer, m_parameters.m_movingAverageFactor);
			}

			const ndBrainFloatBuffer* const parameterBuffer = m_policyTrainer->GetWeightAndBiasBuffer();
			ndBrainFloatBuffer* const referenceParameterBuffer = m_referencePolicyTrainer->GetWeightAndBiasBuffer();
			m_context->Blend(*referenceParameterBuffer, *parameterBuffer, m_parameters.m_movingAverageFactor);
		}
	}
	
	m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_scratchBuffer);
	m_policyTrainer->UpdateParameters(m_scratchBuffer);
}

void ndBrainAgentOffPolicyGradient_Trainer::OptimizeStep()
{
	SaveTrajectory();
	if (m_startOptimization)
	{
		Optimize();
		m_frameCount++;
	}
}