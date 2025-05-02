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
#include "ndBrainTrainer.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainAgentContinuePolicyGradient_Trainer.h"
#include "ndBrainLayerActivationPolicyGradientMeanSigma.h"

#define ND_CONTINUE_POLICY_STATE_VALUE_ITERATIONS				5
#define ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE					(1024 * 256)
#define ND_CONTINUE_POLICY_FIX_SIGMA							ndBrainFloat(0.5f)

#define ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION		ndBrainLayerActivationRelu
//#define ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationTanh
//#define ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationLeakyRelu

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters::HyperParameters()
{
	m_randomSeed = 47;
	m_numberOfLayers = 3;
	m_maxTrajectorySteps = 4096;
	m_batchTrajectoryCount = 1000;
	m_hiddenLayersNumberOfNeurons = 64;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;
	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = m_policyLearnRate * ndBrainFloat(0.5f);

	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(5.0e-3f);
	m_regularizerType = ndBrainOptimizer::m_ridge;
	
	m_useFixSigma = false;
	m_useConstantBaseLineStateValue = false;
	m_actionFixSigma = ND_CONTINUE_POLICY_FIX_SIGMA;

	m_discountRewardFactor = ndBrainFloat(0.99f);
	m_generalizedAdvangeDiscount = ndBrainFloat(0.99f);
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);

//m_threadsCount = 1;
//m_useFixSigma = true;
//m_batchTrajectoryCount = 2;
//m_useConstantBaseLineStateValue = false;
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::ndTrajectoryTransition()
	:ndBrainVector()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize;
	m_obsevationsSize = obsevationsSize;
}

ndInt64 ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::CalculateStride() const
{
	return m_transitionSize + m_actionsSize + m_obsevationsSize * 2;
}

ndInt32 ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetCount() const
{
	const ndInt64 stride = CalculateStride();
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	const ndInt64 stride = CalculateStride();
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return me[stride * entry + m_reward];
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	me[stride * entry + m_reward] = reward;
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetExpectedReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return me[stride * entry + m_expectedReward];
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetExpectedReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	me[stride * entry + m_expectedReward] = reward;
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize];
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize];
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize + m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return &me[stride * entry + m_transitionSize + m_actionsSize + m_obsevationsSize];
}

bool ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	return (me[stride * entry + m_isterminalState] == ndBrainFloat(999.0f)) ? true : false;
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	me[stride * entry + m_isterminalState] = isTernimal ? ndBrainFloat(999.0f) : ndBrainFloat(-999.0f);
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	const ndInt64 stride = CalculateStride();
	ndMemSet(&me[stride * entry], ndBrainFloat(0.0f), stride);
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry)
{
	const ndInt64 stride = CalculateStride();
	ndTrajectoryTransition& me = *this;
	ndMemCpy(&me[stride * entry], &src[stride * srcEntry], stride);
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_TrainerMaster::MemoryStateValues::MemoryStateValues(ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_obsevationsSize(obsevationsSize)
{
	SetCount(ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE * (m_obsevationsSize + 1));
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_TrainerMaster::MemoryStateValues::GetReward(ndInt32 index) const
{
	const ndBrainVector& me = *this;
	return me[index * (m_obsevationsSize + 1)];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_TrainerMaster::MemoryStateValues::GetObservations(ndInt32 index) const
{
	const ndBrainVector& me = *this;
	return &me[index * (m_obsevationsSize + 1) + 1];
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::MemoryStateValues::SaveTransition(ndInt32 index, ndBrainFloat reward, const ndBrainFloat* const observations)
{
	ndBrainVector& me = *this;
	ndInt64 stride = m_obsevationsSize + 1;
	me[index * stride] = reward;
	ndMemCpy(&me[index * stride + 1], observations, m_obsevationsSize);
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_Agent::ndBrainAgentContinuePolicyGradient_Agent(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
	:ndBrainAgent()
	,m_workingBuffer()
	,m_trajectory()
	,m_master(master)
	,m_randomGenerator(nullptr)
	,m_isDead(false)
{
	//std::mt19937 m_gen0(m_rd());
	//std::mt19937 m_gen1(m_rd());
	//m_gen0.seed(m_master->m_randomSeed);
	//m_gen1.seed(m_master->m_randomSeed);
	//std::uniform_real_distribution<ndFloat32> uniform0(ndFloat32(0.0f), ndFloat32(1.0f));
	//std::uniform_real_distribution<ndFloat32> uniform1(ndFloat32(0.0f), ndFloat32(1.0f));
	//ndFloat32 xxx0 = uniform0(m_gen0);
	//ndFloat32 xxx1 = uniform1(m_gen1);

	m_master->m_agents.Append(this);
	m_trajectory.SetCount(0);
	m_randomGenerator = m_master->GetRandomGenerator();
	m_trajectory.Init(m_master->m_policy.GetOutputSize(), m_master->m_policy.GetInputSize());
}

ndBrainAgentContinuePolicyGradient_Agent::~ndBrainAgentContinuePolicyGradient_Agent()
{
	for (ndList<ndBrainAgentContinuePolicyGradient_Agent*>::ndNode* node = m_master->m_agents.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo() == this)
		{
			m_master->m_agents.Remove(node);
			break;
		}
	}
}

ndBrain* ndBrainAgentContinuePolicyGradient_Agent::GetActor()
{ 
	return m_master->GetPolicyNetwork(); 
}

bool ndBrainAgentContinuePolicyGradient_Agent::IsTerminal() const
{
	return false;
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_Agent::SampleActions(ndBrainVector& actions) const
{
	ndRandomGenerator& generator = *m_randomGenerator;
	const ndInt32 count = ndInt32(actions.GetCount()) / 2;
	const ndInt32 start = ndInt32(actions.GetCount()) / 2;
	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		ndFloat32 sigma = actions[start + i];
		ndBrainFloat unitVar = generator.m_d(generator.m_gen);
		ndBrainFloat sample = ndBrainFloat(actions[i]) + unitVar * sigma;
		ndBrainFloat squashedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = squashedAction;
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrain& policy = m_master->m_policy;
	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), policy.GetOutputSize());
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), policy.GetInputSize());
	
	GetObservation(&observation[0]);
	policy.MakePrediction(observation, actions, m_workingBuffer);
	 
	SampleActions(actions);
	ApplyActions(&actions[0]);

	bool isDead = IsTerminal();
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, isDead);

	// remember if this agent died inside a sub step
	m_isDead = m_isDead || isDead;
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_Agent::SaveTrajectory()
{
	for (ndInt32 i = 0; i < m_trajectory.GetCount(); ++i)
	{
		if (m_trajectory.GetTerminalState(i))
		{
			m_trajectory.SetCount(i + 1);
			break;
		}
	}

	if (m_trajectory.GetCount() >= 10)
	{
		m_master->m_batchTrajectoryIndex++;

		for (ndInt32 i = 1; i < m_trajectory.GetCount(); ++i)
		{
			ndMemCpy(m_trajectory.GetNextObservations(i - 1), m_trajectory.GetObservations(i), m_master->m_parameters.m_numberOfObservations);
		}

		// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
		ndBrainFloat gamma = m_master->m_parameters.m_discountRewardFactor;
		ndBrainFloat expectedRewrad = m_trajectory.GetReward(m_trajectory.GetCount() - 1);
		m_trajectory.SetExpectedReward(m_trajectory.GetCount() - 1, expectedRewrad);
		for (ndInt32 i = m_trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndBrainFloat r0 = m_trajectory.GetReward(i);
			expectedRewrad = r0 + gamma * expectedRewrad;
			m_trajectory.SetExpectedReward(i, expectedRewrad);
		}

		const ndInt32 maxSteps = ndMin(m_trajectory.GetCount(), m_master->m_parameters.m_maxTrajectorySteps);
		ndAssert(maxSteps > 0);
		m_trajectory.SetTerminalState(maxSteps - 1, true);

		ndTrajectoryTransition& trajectoryAccumulator = m_master->m_trajectoryAccumulator;
		ndInt32 accumulatorIndex = trajectoryAccumulator.GetCount();
		for (ndInt32 i = 0; i < maxSteps; ++i)
		{
			trajectoryAccumulator.SetCount(accumulatorIndex + 1);
			trajectoryAccumulator.CopyFrom(accumulatorIndex, m_trajectory, i);
			accumulatorIndex++;
		}
	}
	m_isDead = false;
	m_trajectory.SetCount(0);
}

ndInt32 ndBrainAgentContinuePolicyGradient_Agent::GetEpisodeFrames() const
{
	return ndInt32(m_trajectory.GetCount());
}

// ***************************************************************************************
//
// ***************************************************************************************
ndBrainAgentContinuePolicyGradient_TrainerMaster::ndBrainAgentContinuePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainThreadPool()
	,m_policy()
	,m_critic()
	,m_parameters(hyperParameters)
	,m_criticTrainers()
	,m_policyTrainers()
	,m_policyWeightedTrainer()
	,m_policyAuxiliaryTrainers()
	,m_criticOptimizer()
	,m_policyOptimizer()
	,m_advantage()
	,m_randomPermutation()
	,m_randomGenerator()
	,m_trajectoryAccumulator()
	,m_framesAlive(0)
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_extraTrajectorySteps(0)
	,m_batchTrajectoryIndex(0)
	,m_baseValueWorkingBufferSize(0)
	,m_randomSeed(m_parameters.m_randomSeed)
	,m_workingBuffer()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_agents()
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);
	ndSetRandSeed(m_randomSeed);
	m_randomSeed++;

	ndInt32 extraSteps = 0;
	ndFloat32 gamma = 1.0f;
	ndFloat32 totalReward = 0.0f;
	ndFloat32 horizon = ndFloat32(0.99f) / (ndFloat32(1.0f) - m_parameters.m_discountRewardFactor);
	for (; totalReward < horizon; extraSteps++)
	{
		totalReward += gamma;
		gamma *= m_parameters.m_discountRewardFactor;
	}
	m_extraTrajectorySteps = extraSteps;

	// build policy neural net
	SetThreadCount(m_parameters.m_threadsCount);

	BuildPolicyClass();
	BuildCriticClass();
}

ndBrainAgentContinuePolicyGradient_TrainerMaster::~ndBrainAgentContinuePolicyGradient_TrainerMaster()
{
	for (ndInt32 i = 0; i < m_policyTrainers.GetCount(); ++i)
	{
		delete m_policyTrainers[i];
		delete m_policyAuxiliaryTrainers[i];
	}
	
	for (ndInt32 i = 0; i < m_criticTrainers.GetCount(); ++i)
	{
		delete m_criticTrainers[i];
	}
}

ndBrain* ndBrainAgentContinuePolicyGradient_TrainerMaster::GetPolicyNetwork()
{ 
	return &m_policy; 
}

ndBrain* ndBrainAgentContinuePolicyGradient_TrainerMaster::GetValueNetwork()
{
	return &m_critic;
}

const ndString& ndBrainAgentContinuePolicyGradient_TrainerMaster::GetName() const
{
	return m_name;
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::SetName(const ndString& name)
{
	m_name = name;
}

//#pragma optimize( "", off )
ndBrainAgentContinuePolicyGradient_Agent::ndRandomGenerator* ndBrainAgentContinuePolicyGradient_TrainerMaster::GetRandomGenerator()
{
	ndList<ndBrainAgentContinuePolicyGradient_Agent::ndRandomGenerator>::ndNode* const node = m_randomGenerator.Append();
	ndBrainAgentContinuePolicyGradient_Agent::ndRandomGenerator* const generator = &node->GetInfo();
	generator->m_gen.seed(m_randomSeed);
	m_randomSeed ++;
	return generator;
}

ndUnsigned32 ndBrainAgentContinuePolicyGradient_TrainerMaster::GetFramesCount() const
{
	return m_frameCount;
}

bool ndBrainAgentContinuePolicyGradient_TrainerMaster::IsSampling() const
{
	return false;
}

ndUnsigned32 ndBrainAgentContinuePolicyGradient_TrainerMaster::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentContinuePolicyGradient_TrainerMaster::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

ndFloat32 ndBrainAgentContinuePolicyGradient_TrainerMaster::GetAverageScore() const
{
	return m_averageExpectedRewards.GetAverage();
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::BuildPolicyClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < m_parameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(m_parameters.m_hiddenLayersNumberOfNeurons, m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
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

	//m_policy.SaveToFile("xxxx.dnn");
	//ndSharedPtr<ndBrain> xxx(ndBrainLoad::Load("xxxx.dnn"));

	m_policyTrainers.SetCount(0);
	m_policyAuxiliaryTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_policy);
		m_policyTrainers.PushBack(trainer);

		ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(&m_policy);
		m_policyAuxiliaryTrainers.PushBack(auxiliaryTrainer);
	}

	m_policyWeightedTrainer.PushBack(m_policyTrainers[0]);
	m_policyOptimizer = ndSharedPtr<ndBrainOptimizerAdam> (new ndBrainOptimizerAdam());
	m_policyOptimizer->SetRegularizer(m_parameters.m_policyRegularizer);
	m_policyOptimizer->SetRegularizerType(m_parameters.m_regularizerType);

	m_trajectoryAccumulator.Init(m_policy.GetOutputSize(), m_policy.GetInputSize());
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::BuildCriticClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	// build state value critic neural net
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	for (ndInt32 i = 0; i < m_parameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	layers.PushBack(new ndBrainLayerActivationLeakyRelu(layers[layers.GetCount() - 1]->GetOutputSize()));

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_critic.AddLayer(layers[i]);
	}
	m_critic.InitWeights();

	ndAssert(m_critic.GetOutputSize() == 1);
	ndAssert(m_critic.GetInputSize() == m_policy.GetInputSize());

	m_criticTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic);
		m_criticTrainers.PushBack(trainer);
	}

	m_criticOptimizer = ndSharedPtr<ndBrainOptimizerAdam> (new ndBrainOptimizerAdam());
	m_criticOptimizer->SetRegularizer(m_parameters.m_criticRegularizer);
	m_criticOptimizer->SetRegularizerType(m_parameters.m_regularizerType);

	m_baseValueWorkingBufferSize = m_critic.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * m_parameters.m_threadsCount);
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeCritic()
{
	if (!m_parameters.m_useConstantBaseLineStateValue)
	{
		m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount() - 1);
		for (ndInt32 i = ndInt32(m_randomPermutation.GetCount()) - 1; i >= 0; --i)
		{
			m_randomPermutation[i] = i;
		}
		if (m_randomPermutation.GetCount() < m_parameters.m_miniBatchSize)
		{
			ndInt32 mod = ndInt32(m_randomPermutation.GetCount());
			ndInt32 padding = m_parameters.m_miniBatchSize - ndInt32(m_randomPermutation.GetCount());
			for (ndInt32 i = 0; i < padding; ++i)
			{
				m_randomPermutation.PushBack(i % mod);
			}
		}

		// generalize state value function, in theory more noisy but yields better results than Monte Carlos.
		if (m_randomPermutation.GetCount() > ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE)
		{
			m_randomPermutation.SetCount(ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE);
		}
		else
		{
			m_randomPermutation.SetCount(m_randomPermutation.GetCount() & -m_parameters.m_miniBatchSize);
		}

		ndAtomic<ndInt32> iterator(0);
		for (ndInt32 i = 0; i < ND_CONTINUE_POLICY_STATE_VALUE_ITERATIONS; ++i)
		{
			m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
			for (ndInt32 base = 0; base < m_randomPermutation.GetCount(); base += m_parameters.m_miniBatchSize)
			{
				auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
				{
					ndBrainLossLeastSquaredError loss(1);
					ndBrainFixSizeVector<1> stateValue;
					ndBrainFixSizeVector<1> stateQValue;

					// calculate GAE(l, 1) // very, very noisy
					// calculate GAE(l, 0) // too smooth, and does not work either
					ndFloat32 gamma = m_parameters.m_discountRewardFactor * m_parameters.m_generalizedAdvangeDiscount;

					for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
					{
						const ndInt32 index = m_randomPermutation[base + i];
						ndBrainTrainer& trainer = *m_criticTrainers[i];

						stateValue[0] = m_trajectoryAccumulator.GetReward(index);
						if (!m_trajectoryAccumulator.GetTerminalState(index))
						{
							const ndBrainMemVector nextObservation(m_trajectoryAccumulator.GetNextObservations(index), m_parameters.m_numberOfObservations);
							m_critic.MakePrediction(nextObservation, stateQValue);
							stateValue[0] += gamma * stateQValue[0];
						}

						loss.SetTruth(stateValue);
						const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
						trainer.BackPropagate(observation, loss);
					}
				});

				iterator = 0;
				ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
				m_criticOptimizer->Update(this, m_criticTrainers, m_parameters.m_criticLearnRate);
			}
		}
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::CalculateAdvange()
{
	if (m_parameters.m_useConstantBaseLineStateValue)
	{
		ndFloat64 averageSum = ndBrainFloat(0.0f);
		const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = 0; i < stepNumber; ++i)
		{
			ndFloat32 expectedReward = m_trajectoryAccumulator.GetExpectedReward(i);
			averageSum += expectedReward;
		}
		ndFloat32 baseLineReward = ndFloat32(averageSum / ndFloat32(stepNumber));
		m_averageExpectedRewards.Update(baseLineReward);
		m_averageFramesPerEpisodes.Update(ndFloat32(stepNumber) / ndFloat32(m_parameters.m_batchTrajectoryCount));

#if 0
		// using minimum variance normalization. 
		// I actually see far more oscillation of variance with this method that just fitting a unit Gaussian 	
		ndAtomic<ndInt32> iterator(0);
		m_advantage.SetCount(m_trajectoryAccumulator.GetCount());
		auto CalculateMinimumVariaceBaseLine = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			ndBrainFixSizeVector<256> probabilities;
			ndInt32 numberOfActions = m_policy.GetOutputSize();
			ndInt32 numberOfObservations = m_policy.GetInputSize();
			probabilities.SetCount(numberOfActions);

			ndInt32 const count = m_trajectoryAccumulator.GetCount();
			for (ndInt32 i = iterator.fetch_add(m_parameters.m_miniBatchSize); i < count; i = iterator.fetch_add(m_parameters.m_miniBatchSize))
			{
				const ndInt32 batchCount = ((i + m_parameters.m_miniBatchSize) < m_trajectoryAccumulator.GetCount()) ? m_parameters.m_miniBatchSize : m_trajectoryAccumulator.GetCount() - i;
				for (ndInt32 k = 0; k < batchCount; ++k)
				{
					ndInt32 index = i + k;
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), numberOfObservations);
					m_policy.MakePrediction(observation, probabilities);

					ndBrainFloat z2 = 0.0f;
					const ndBrainMemVector sampledProbabilities(m_trajectoryAccumulator.GetActions(index), numberOfActions);
					ndBrainFloat sigma2 = probabilities[numberOfActions - 1];
					ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / ndSqrt(2.0f * ndPi * sigma2);
					ndBrainFloat invSigma2Det = ndBrainFloat(1.0f);
					for (ndInt32 j = numberOfActions - 2; j >= 0; --j)
					{
						ndBrainFloat z = sampledProbabilities[j] - probabilities[j];
						z2 += z * z;
						invSigma2Det *= invSigma2;
					}
					ndBrainFloat exponent = ndBrainFloat(0.5f) * z2 / sigma2;
					ndBrainFloat prob = invSigma2Det * ndExp(-exponent);
					ndBrainFloat logProb = ndLog(prob);
					m_advantage[index] = logProb * logProb;
				}
			}
		});
		ndBrainThreadPool::ParallelExecute(CalculateMinimumVariaceBaseLine);

		ndBrainFloat logProbSum2 = 1.0e-4f;
		ndBrainFloat logProbSum2Reward = 0.0f;
		for (ndInt32 i = ndInt32(m_advantage.GetCount()) - 1; i >= 0; --i)
		{
			ndBrainFloat variace2 = m_advantage[i];
			logProbSum2 += variace2;
			logProbSum2Reward += variace2 * m_trajectoryAccumulator.GetExpectedReward(i);
		}

		ndBrainFloat baseLine = logProbSum2Reward / logProbSum2;
		for (ndInt32 i = ndInt32(m_advantage.GetCount()) - 1; i >= 0; --i)
		{
			m_advantage[i] = m_trajectoryAccumulator.GetExpectedReward(i) - baseLine;
		}
#else
		// just fit a unit Gaussian, reduces variance, 
		// so far, it seems more stable that all other methods.
		ndFloat64 varianceSum2 = ndBrainFloat(0.0f);
		for (ndInt32 i = 0; i < stepNumber; ++i)
		{
			ndFloat32 expectedReward = m_trajectoryAccumulator.GetExpectedReward(i);
			varianceSum2 += expectedReward * expectedReward;
		}
		m_advantage.SetCount(0);
		ndFloat32 variance = ndSqrt(ndMax(ndFloat32(varianceSum2 / ndFloat32(stepNumber)), ndFloat32(1.0e-4f)));
		ndFloat32 invVariance = ndBrainFloat(1.0f) / variance;
		for (ndInt32 i = 0; i < stepNumber; ++i)
		{
			ndFloat32 expectedReward = m_trajectoryAccumulator.GetExpectedReward(i);
			m_advantage.PushBack((expectedReward - baseLineReward) * invVariance);
		}
#endif
	}
	else
	{
		ndBrainFloat averageSum = ndBrainFloat(0.0f);
		const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = stepNumber - 1; i >= 0; --i)
		{
			averageSum += m_trajectoryAccumulator.GetExpectedReward(i);
		}
		m_averageExpectedRewards.Update(averageSum / ndBrainFloat(stepNumber));
		m_averageFramesPerEpisodes.Update(ndBrainFloat(stepNumber) / ndBrainFloat(m_parameters.m_batchTrajectoryCount));

		ndAtomic<ndInt32> iterator(0);
		m_advantage.SetCount(m_trajectoryAccumulator.GetCount());

		m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
		auto CalculateAdvantage = ndMakeObject::ndFunction([this, &iterator](ndInt32 threadIndex, ndInt32)
		{
			// using Monte Carlos 
			ndBrainFixSizeVector<1> actions;
			ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);

			ndInt32 const count = m_trajectoryAccumulator.GetCount();
			for (ndInt32 i = iterator++; i < count; i = iterator++)
			{
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
				m_critic.MakePrediction(observation, actions, workingBuffer);
				ndBrainFloat stateQValue = actions[0];

				// calculate GAE(l, 1) // very, very noisy
				//ndBrainFloat trajectoryExpectedReward = m_trajectoryAccumulator.GetExpectedReward(i);
				//ndBrainFloat advantage = trajectoryExpectedReward - stateValue;

				// calculate GAE(l, 0) // too smooth, and does not work either
				const ndBrainMemVector nextObservation(m_trajectoryAccumulator.GetNextObservations(i), m_parameters.m_numberOfObservations);
				m_critic.MakePrediction(nextObservation, actions, workingBuffer);
				ndBrainFloat nextStateQValue = actions[0];
				ndBrainFloat advantage1 = m_trajectoryAccumulator.GetReward(i) + nextStateQValue - stateQValue;

				m_advantage[i] = advantage1;
			}
		});
		ndBrainThreadPool::ParallelExecute(CalculateAdvantage);
	}

	if (m_advantage.GetCount() < m_parameters.m_miniBatchSize)
	{
		ndInt32 start = 0;
		for (ndInt32 i = ndInt32(m_advantage.GetCount()); i < m_parameters.m_miniBatchSize; ++i)
		{
			m_trajectoryAccumulator.SetCount(i + 1);
			m_trajectoryAccumulator.CopyFrom(i, m_trajectoryAccumulator, start);
			m_advantage.PushBack(m_advantage[start]);
			start++;
		}
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizePolicy()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_policyTrainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	if (m_trajectoryAccumulator.GetCount() < m_parameters.m_miniBatchSize)
	{
		ndInt32 padTransitions = m_parameters.m_miniBatchSize - m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = 0; i < padTransitions; ++i)
		{
			ndInt32 index = m_trajectoryAccumulator.GetCount();
			m_trajectoryAccumulator.SetCount(index + 1);
			m_trajectoryAccumulator.CopyFrom(index, m_trajectoryAccumulator, i);

			ndBrainFloat advantage = m_advantage[i];
			m_advantage.PushBack(advantage);
		}
	}

	const ndInt32 steps = m_trajectoryAccumulator.GetCount() & -m_parameters.m_miniBatchSize;
	for (ndInt32 base = 0; base < steps; base += m_parameters.m_miniBatchSize)
	{

		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class MaxLikelihoodLoss : public ndBrainLoss
			{
				public:
				MaxLikelihoodLoss(ndBrainTrainer& trainer, ndBrainAgentContinuePolicyGradient_TrainerMaster* const owner, ndInt32 index)
					:ndBrainLoss()
					,m_trainer(trainer)
					,m_owner(owner)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					// as I understand it, this is just a special case of maximum likelihood optimization.
					// given a multivariate Gaussian process with zero cross covariance to the actions.
					// calculate the log of prob of a multivariate Gaussian

					const ndInt32 numberOfActions = m_owner->m_policy.GetOutputSize();
					const ndBrainMemVector sampledProbability(m_owner->m_trajectoryAccumulator.GetActions(m_index), numberOfActions);

					const ndInt32 count = ndInt32(m_owner->m_policy.GetOutputSize()) / 2;
					const ndInt32 start = ndInt32(m_owner->m_policy.GetOutputSize()) / 2;
					for (ndInt32 i = count - 1; i >= 0; --i)
					{
						ndBrainFloat sigma = probabilityDistribution[i + start];
						ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
						ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
						ndBrainFloat meanGrad = z * invSigma * invSigma;
						ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

						loss[i] = meanGrad;
						loss[i + start] = sigmaGrad;
					}
					const ndBrainFloat advantage = m_owner->m_advantage[m_index];

					//negate the gradient for gradient ascend?
					ndBrainFloat ascend = ndBrainFloat(-1.0f) * advantage;
					loss.Scale(ascend);
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinuePolicyGradient_TrainerMaster* m_owner;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				ndInt32 index = base + i;
				MaxLikelihoodLoss loss(trainer, this, index);
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
				trainer.BackPropagate(observation, loss);
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_policyTrainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}

	m_policyOptimizer->AccumulateGradients(this, m_policyTrainers);
	m_policyWeightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(steps));
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, m_parameters.m_policyLearnRate);
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::Optimize()
{
	OptimizeCritic();
	CalculateAdvange();
	OptimizePolicy();
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeStep()
{
	for (ndList<ndBrainAgentContinuePolicyGradient_Agent*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentContinuePolicyGradient_Agent* const agent = node->GetInfo();
		ndAssert(agent->m_trajectory.GetCount());

		bool isTeminal = agent->m_isDead;
		isTeminal = isTeminal || (agent->m_trajectory.GetCount() >= (m_parameters.m_maxTrajectorySteps + m_extraTrajectorySteps));
		if (isTeminal)
		{
			agent->SaveTrajectory();
			agent->ResetModel();
		}
		m_frameCount++;
		m_framesAlive++;
	}

	if (m_batchTrajectoryIndex >= m_parameters.m_batchTrajectoryCount)
	{
		Optimize();

		m_eposideCount++;
		m_framesAlive = 0;
		m_batchTrajectoryIndex = 0;
		m_trajectoryAccumulator.SetCount(0);
		for (ndList<ndBrainAgentContinuePolicyGradient_Agent*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentContinuePolicyGradient_Agent* const agent = node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
		}
	}
}