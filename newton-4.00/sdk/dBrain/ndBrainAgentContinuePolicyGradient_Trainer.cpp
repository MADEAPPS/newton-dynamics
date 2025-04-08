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
#include "ndBrainLayerLinear.h"
#include "ndBrainOptimizerSgd.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainAgentContinuePolicyGradient_Trainer.h"

#define ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE		(1024 * 256)
#define ND_CONTINUE_POLICY_STATE_VALUE_ITERATIONS	5
#define ND_CONTINUE_POLICY_MIN_SIGMA				ndBrainFloat(0.125f)

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters::HyperParameters()
{
	m_randomSeed = 47;
	m_numberOfLayers = 3;
	m_bashBufferSize = 256;
	m_neuronPerLayers = 64;
	m_maxTrajectorySteps = 4096;
	m_bashTrajectoryCount = 500;
	m_extraTrajectorySteps = 1024;

	m_numberOfActions = 0;
	m_numberOfObservations = 0;
	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = m_policyLearnRate * ndBrainFloat(1.0f);

	m_policyRegularizer = ndBrainFloat(1.0e-5f);
	m_criticRegularizer = ndBrainFloat(1.0e-3f);

	m_discountFactor = ndBrainFloat(0.99f);
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize);
//m_threadsCount = 1;
}

//*********************************************************************************************
//
//*********************************************************************************************
ndPolicyGradientActivation::ndPolicyGradientActivation(ndInt32 neurons)
	:ndBrainLayerActivationTanh(neurons)
{
}

ndPolicyGradientActivation::ndPolicyGradientActivation(const ndPolicyGradientActivation& src)
	:ndBrainLayerActivationTanh(src)
{
}

ndBrainLayer* ndPolicyGradientActivation::Clone() const
{
	return new ndPolicyGradientActivation(*this);
}

void ndPolicyGradientActivation::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerActivation::Save(loadSave);
}

ndBrainFloat ndPolicyGradientActivation::GetMinSigma() const
{
	return ND_CONTINUE_POLICY_MIN_SIGMA;
}

ndBrainLayer* ndPolicyGradientActivation::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndPolicyGradientActivation* const layer = new ndPolicyGradientActivation(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

const char* ndPolicyGradientActivation::GetLabelId() const
{
	return "ndPolicyGradientActivation";
}

//#pragma optimize( "", off )
void ndPolicyGradientActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndBrainLayerActivationTanh::MakePrediction(input, output);
	const ndInt32 base = m_neurons / 2;
	ndBrainFloat minSigma = GetMinSigma();
	for (ndInt32 i = base - 1; i >= 0; --i)
	{
		// this is fucking wrong, but it is yeding better results than teh correct version.
		output[i + base] = ndMax(input[i + base], minSigma);
		// correct version, but les has less performance, it need to test this more.
		//output[i + base] = (input[i + base] >= ndBrainFloat(0.0f)) ? input[i + base] + minSigma : minSigma;
	}
}

//#pragma optimize( "", off )
void ndPolicyGradientActivation::InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndBrainLayerActivationTanh::InputDerivative(input, output, outputDerivative, inputDerivative);
	const ndInt32 base = m_neurons / 2;
	for (ndInt32 i = base - 1; i >= 0; --i)
	{
		ndBrainFloat x = (input[i + base] >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		inputDerivative[i + base] = outputDerivative[i + base] * x;
	}
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::ndTrajectoryTransition(ndInt32 actionsSize, ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_actionsSize(actionsSize)
	,m_obsevationsSize(obsevationsSize)
{
}

ndInt32 ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetCount() const
{
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return me[stride * entry + m_reward];
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry + m_reward] = reward;
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetAdvantage(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return me[stride * entry + m_stateAdvantage];
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::SetAdvantage(ndInt32 entry, ndBrainFloat advantage)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry + m_stateAdvantage] = advantage;
}

bool ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return (me[stride * entry + m_isterminalState] == ndBrainFloat(100.0f)) ? true : false;
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry + m_isterminalState] = isTernimal ? ndBrainFloat(100.0f) : ndBrainFloat(-100.0f);
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	ndMemSet(&me[stride * entry], ndBrainFloat(0.0f), stride);
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry)
{
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	ndTrajectoryTransition& me = *this;
	ndMemCpy(&me[stride * entry], &src[stride * srcEntry], stride);
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + m_transitionSize];
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetActions(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + m_obsevationsSize + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetActions(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + m_obsevationsSize + m_transitionSize];
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetStateReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return me[stride * entry + m_stateReward];
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::SetStateReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry + m_stateReward] = reward;
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetStateValue(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return me[stride * entry + m_stateValue];
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::SetStateValue(ndInt32 entry, ndBrainFloat stateValue)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry + m_stateValue] = stateValue;
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetProbabilityDistribution(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + m_obsevationsSize + m_actionsSize * 2 + m_transitionSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryTransition::GetProbabilityDistribution(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = m_transitionSize + m_actionsSize * 2 + m_obsevationsSize;
	return &me[stride * entry + m_obsevationsSize + m_actionsSize * 2 + m_transitionSize];
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
ndBrainAgentContinuePolicyGradient_Trainer::ndBrainAgentContinuePolicyGradient_Trainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
	:ndBrainAgent()
	,m_workingBuffer()
	,m_trajectory(master->m_numberOfActions, master->m_numberOfObservations)
	,m_master(master)
	,m_randomGenerator(nullptr)
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
}

ndBrainAgentContinuePolicyGradient_Trainer::~ndBrainAgentContinuePolicyGradient_Trainer()
{
	for (ndList<ndBrainAgentContinuePolicyGradient_Trainer*>::ndNode* node = m_master->m_agents.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo() == this)
		{
			m_master->m_agents.Remove(node);
			break;
		}
	}
}

ndBrain* ndBrainAgentContinuePolicyGradient_Trainer::GetActor()
{ 
	return m_master->GetPolicyNetwork(); 
}

bool ndBrainAgentContinuePolicyGradient_Trainer::IsTerminal() const
{
	return false;
}

void ndBrainAgentContinuePolicyGradient_Trainer::SelectAction(ndBrainVector& actions) const
{
	const ndInt32 numberOfActions = m_master->m_numberOfActions;

	ndRandomGenerator& generator = *m_randomGenerator;
	for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
	{
		ndBrainFloat sample = ndBrainFloat(actions[i] + generator.m_d(generator.m_gen) * actions[i + numberOfActions]);
		ndBrainFloat squashedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = squashedAction;
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_Trainer::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), m_master->m_numberOfActions * 2);
	ndBrainMemVector probabilityDistribution(m_trajectory.GetProbabilityDistribution(entryIndex), m_master->m_numberOfActions * 2);
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), m_master->m_numberOfObservations);
	
	GetObservation(&observation[0]);
	m_master->m_policy.MakePrediction(observation, actions, m_workingBuffer);
	probabilityDistribution.Set(actions);
	 
	SelectAction(actions);
	ApplyActions(&actions[0]);

	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetStateReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_Trainer::SaveTrajectory()
{
	if (m_trajectory.GetCount())
	{
		m_master->m_bashTrajectoryIndex++;
		while (m_trajectory.GetTerminalState(m_trajectory.GetCount() - 2))
		{
			m_trajectory.SetCount(m_trajectory.GetCount() - 1);
		}
		m_trajectory.SetTerminalState(m_trajectory.GetCount() - 1, true);

		// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
		ndBrainFloat gamma = m_master->m_gamma;
		for (ndInt32 i = m_trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndBrainFloat r0 = m_trajectory.GetStateReward(i);
			ndBrainFloat r1 = m_trajectory.GetStateReward(i + 1);
			m_trajectory.SetStateReward(i, r0 + gamma * r1);
		}
		
		// get the max trajectory steps
		const ndInt32 maxSteps = ndMin(m_trajectory.GetCount(), m_master->m_maxTrajectorySteps);
		ndAssert(maxSteps > 0);
		ndTrajectoryTransition& trajectoryAccumulator = m_master->m_trajectoryAccumulator;
		for (ndInt32 i = 0; i < maxSteps; ++i)
		{
			ndInt32 index = trajectoryAccumulator.GetCount();
			trajectoryAccumulator.SetCount(index + 1);
			trajectoryAccumulator.CopyFrom(index, m_trajectory, i);
		}
	}
	m_trajectory.SetCount(0);
}

ndInt32 ndBrainAgentContinuePolicyGradient_Trainer::GetEpisodeFrames() const
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
	,m_criticTrainers()
	,m_policyTrainers()
	,m_policyWeightedTrainer()
	,m_policyAuxiliaryTrainers()
	,m_criticOptimizer(nullptr)
	,m_policyOptimizer(nullptr)
	,m_randomPermutation()
	,m_trajectoryAccumulator(hyperParameters.m_numberOfActions, hyperParameters.m_numberOfObservations)
	,m_gamma(hyperParameters.m_discountFactor)
	,m_policyLearnRate(hyperParameters.m_policyLearnRate)
	,m_criticLearnRate(hyperParameters.m_criticLearnRate)
	,m_numberOfActions(hyperParameters.m_numberOfActions)
	,m_numberOfObservations(hyperParameters.m_numberOfObservations)
	,m_framesAlive(0)
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_bashTrajectoryIndex(0)
	,m_bashTrajectoryCount(hyperParameters.m_bashTrajectoryCount)
	,m_bashTrajectorySteps(hyperParameters.m_bashTrajectoryCount * m_maxTrajectorySteps)
	,m_baseValueWorkingBufferSize(0)
	,m_randomSeed(hyperParameters.m_randomSeed)
	,m_workingBuffer()
	,m_averageScore()
	,m_averageFramesPerEpisodes()
	,m_agents()
{
	ndAssert(m_numberOfActions);
	ndAssert(m_numberOfObservations);
	ndSetRandSeed(m_randomSeed);

	m_randomGenerator = new ndBrainAgentContinuePolicyGradient_Trainer::ndRandomGenerator[size_t(hyperParameters.m_bashTrajectoryCount)];
	//for (ndInt32 i = 0; i < hyperParameters.m_bashTrajectoryCount; ++i)
	//{
	//	m_randomSeed++;
	//	m_randomGenerator[i].m_gen.seed(m_randomSeed);
	//}
	//m_randomSeed = 47;

	// build policy neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	//layers.PushBack(new ndBrainLayerActivationTanh(m_numberOfObservations));
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, hyperParameters.m_neuronPerLayers));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, hyperParameters.m_neuronPerLayers));
		layers.PushBack(new ndBrainLayerActivationTanh(hyperParameters.m_neuronPerLayers));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, m_numberOfActions * 2));
	layers.PushBack(new ndPolicyGradientActivation(m_numberOfActions * 2));

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_policy.AddLayer(layers[i]);
	}

	m_policy.InitWeights();

	//NormalizePolicy();
	//m_policy.SaveToFile("xxxx.dnn");
	//ndSharedPtr<ndBrain> xxx(ndBrainLoad::Load("xxxx.dnn"));

	m_policyTrainers.SetCount(0);
	m_policyAuxiliaryTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_policy);
		m_policyTrainers.PushBack(trainer);
	
		ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(&m_policy);
		m_policyAuxiliaryTrainers.PushBack(auxiliaryTrainer);
	}
	
	m_policyWeightedTrainer.PushBack(m_policyTrainers[0]);
	m_policyOptimizer = new ndBrainOptimizerAdam();
	m_policyOptimizer->SetRegularizer(hyperParameters.m_policyRegularizer);

	// build state value critic neural net
	layers.SetCount(0);
	ndInt32 criticNeurons = hyperParameters.m_neuronPerLayers * 1;
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, criticNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == criticNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), criticNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_critic.AddLayer(layers[i]);
	}
	m_critic.InitWeights();
	//NormalizeCritic();
	
	ndAssert(m_critic.GetOutputSize() == 1);
	ndAssert(m_critic.GetInputSize() == m_policy.GetInputSize());
	ndAssert(!strcmp((m_critic[m_critic.GetCount() - 1])->GetLabelId(), "ndBrainLayerLinear"));
	
	m_criticTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic);
		m_criticTrainers.PushBack(trainer);
	}
	
	m_criticOptimizer = new ndBrainOptimizerAdam();
	m_criticOptimizer->SetRegularizer(hyperParameters.m_criticRegularizer);
	
	m_baseValueWorkingBufferSize = m_critic.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
}

ndBrainAgentContinuePolicyGradient_TrainerMaster::~ndBrainAgentContinuePolicyGradient_TrainerMaster()
{
	for (ndInt32 i = 0; i < m_policyTrainers.GetCount(); ++i)
	{
		delete m_policyTrainers[i];
		delete m_policyAuxiliaryTrainers[i];
	}
	delete m_policyOptimizer;
	
	for (ndInt32 i = 0; i < m_criticTrainers.GetCount(); ++i)
	{
		delete m_criticTrainers[i];
	}
	delete m_criticOptimizer;

	delete[] m_randomGenerator;
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::NormalizePolicy()
{
	// using supervised learning make sure that the m_policy has zero mean and standard deviation 
	class SupervisedTrainer : public ndBrainThreadPool
	{
		public:
		SupervisedTrainer(ndBrain* const brain)
			:ndBrainThreadPool()
			,m_brain(*brain)
			,m_trainer(new ndBrainTrainer(&m_brain))
			,m_learnRate(ndReal(1.0e-2f))
		{
			SetThreadCount(1);
			m_partialGradients.PushBack(*m_trainer);
		}

		void Optimize()
		{
			ndAtomic<ndInt32> iterator(0);

			ndBrainFloat* const inMemory = ndAlloca(ndBrainFloat, m_brain.GetInputSize());
			ndBrainFloat* const outMemory = ndAlloca(ndBrainFloat, m_brain.GetOutputSize());
			ndBrainMemVector input(inMemory, m_brain.GetInputSize());
			ndBrainMemVector groundTruth(outMemory, m_brain.GetOutputSize());

			bool stops = false;
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, &stops, &input, &groundTruth](ndInt32, ndInt32)
			{
				class PolicyLoss : public ndBrainLossLeastSquaredError
				{
					public:
					PolicyLoss(ndInt32 size)
						:ndBrainLossLeastSquaredError(size)
						,m_stop (false)
					{
					}
			
					void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
					{
						ndBrainLossLeastSquaredError::GetLoss(output, loss);
						ndBrainFloat error2 = loss.Dot(loss);
						m_stop = error2 < ndBrainFloat(1.0e-8f);
					}

					bool m_stop;
				};

				ndBrainTrainer& trainer = *(*m_trainer);
				PolicyLoss loss(m_brain.GetOutputSize());
				loss.SetTruth(groundTruth);
				trainer.BackPropagate(input, loss);
				stops = loss.m_stop;
			});

			ndBrainOptimizerSgd optimizer;
			optimizer.SetRegularizer(ndBrainFloat(1.0e-5f));

			input.Set(ndBrainFloat(0.0f));
			const ndInt32 base = ndInt32 (groundTruth.GetCount()) / 2;
			for (ndInt32 i = base - 1; i >= 0; --i)
			{
				groundTruth[i] = ndBrainFloat(0.0f);
				groundTruth[i + base] = ((ndPolicyGradientActivation*) m_brain[m_brain.GetCount() - 1])->GetMinSigma();
			}

			int xxxxx = 0;
			for (ndInt32 i = 0; (i < 10000) && !stops; ++i)
			{
				xxxxx++;
				ndBrainThreadPool::ParallelExecute(BackPropagateBash);
				optimizer.Update(this, m_partialGradients, m_learnRate);
			}

			ndBrainFloat* const outMemory1 = ndAlloca(ndBrainFloat, m_brain.GetOutputSize());
			ndBrainMemVector output1(outMemory1, m_brain.GetOutputSize());
			m_brain.MakePrediction(input, output1);
			input[0] *= 1.0f;
		}

		ndBrain& m_brain;
		ndSharedPtr<ndBrainTrainer> m_trainer;
		ndArray<ndBrainTrainer*> m_partialGradients;
		ndReal m_learnRate;
	};

	SupervisedTrainer optimizer(&m_policy);
	optimizer.Optimize();
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::NormalizeCritic()
{
	// using supervised learning make sure that the m_policy has zero mean and standard deviation 
	class SupervisedTrainer : public ndBrainThreadPool
	{
		public:
		SupervisedTrainer(ndBrain* const brain)
			:ndBrainThreadPool()
			,m_brain(*brain)
			,m_trainer(new ndBrainTrainer(&m_brain))
			,m_learnRate(ndReal(1.0e-2f))
		{
			SetThreadCount(1);
			m_partialGradients.PushBack(*m_trainer);
		}

		void Optimize()
		{
			ndAtomic<ndInt32> iterator(0);

			ndBrainFloat* const inMemory = ndAlloca(ndBrainFloat, m_brain.GetInputSize());
			ndBrainFloat* const outMemory = ndAlloca(ndBrainFloat, m_brain.GetOutputSize());
			ndBrainMemVector input(inMemory, m_brain.GetInputSize());
			ndBrainMemVector groundTruth(outMemory, m_brain.GetOutputSize());

			bool stops = false;
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, &stops, &input, &groundTruth](ndInt32, ndInt32)
			{
				class PolicyLoss : public ndBrainLossLeastSquaredError
				{
					public:
					PolicyLoss(ndInt32 size)
						:ndBrainLossLeastSquaredError(size)
						,m_stop(false)
					{
					}

					void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
					{
						ndBrainLossLeastSquaredError::GetLoss(output, loss);

						ndBrainFloat error2 = loss.Dot(loss);
						m_stop = error2 < ndBrainFloat(1.0e-8f);
					}

					bool m_stop = false;
				};

				ndBrainTrainer& trainer = *(*m_trainer);
				PolicyLoss loss(m_brain.GetOutputSize());
				loss.SetTruth(groundTruth);
				trainer.BackPropagate(input, loss);
				stops = loss.m_stop;
			});

			ndBrainOptimizerSgd optimizer;
			optimizer.SetRegularizer(ndBrainFloat(1.0e-5f));

			input.Set(0.0f);
			groundTruth.Set(0.0f);
			for (ndInt32 i = 0; (i < 10000) && !stops; ++i)
			{
				ndBrainThreadPool::ParallelExecute(BackPropagateBash);
				optimizer.Update(this, m_partialGradients, m_learnRate);
			}
			//ndBrainFloat* const outMemory1 = ndAlloca(ndBrainFloat, m_brain.GetOutputSize());
			//ndBrainMemVector output1(outMemory1, m_brain.GetOutputSize());
			//m_brain.MakePrediction(input, output1);
			//m_brain.MakePrediction(input, output1);
		}

		ndBrain& m_brain;
		ndSharedPtr<ndBrainTrainer> m_trainer;
		ndArray<ndBrainTrainer*> m_partialGradients;
		ndReal m_learnRate;
	};

	SupervisedTrainer optimizer(&m_critic);
	optimizer.Optimize();
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

ndBrainAgentContinuePolicyGradient_Trainer::ndRandomGenerator* ndBrainAgentContinuePolicyGradient_TrainerMaster::GetRandomGenerator()
{
	ndInt32 index = m_agents.GetCount() - 1;
	m_randomGenerator[index].m_gen.seed(m_randomSeed);
	m_randomSeed ++;
	return &m_randomGenerator[index];
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
	return m_averageScore.GetAverage();
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizePolicy()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_policyTrainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount() - 1) & -m_bashBufferSize;
	for (ndInt32 base = 0; base < steps; base += m_bashBufferSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class MaxLikelihoodLoss : public ndBrainLoss
			{
				public:
				MaxLikelihoodLoss(ndBrainTrainer& trainer, ndBrainAgentContinuePolicyGradient_TrainerMaster* const agent, ndInt32 index)
					:ndBrainLoss()
					,m_trainer(trainer)
					,m_agent(agent)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					// basically this fits a multivariate Gaussian process with zero cross covariance to the actions.
					// calculate the log of prob of a multivariate Gaussian
					const ndInt32 numberOfActions = m_agent->m_numberOfActions;
					const ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					const ndBrainMemVector sampledProbability(m_agent->m_trajectoryAccumulator.GetActions(m_index), numberOfActions * 2);

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = probabilityDistribution[i];
						const ndBrainFloat sigma1 = probabilityDistribution[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat num = sampledProbability[i] - mean;

						ndBrainFloat meanGradient = num / sigma1;
						ndBrainFloat sigmaGradient = ndBrainFloat(0.5f) * (num * num / sigma2 - ndBrainFloat(1.0f) / sigma1);

						loss[i] = meanGradient * advantage;
						loss[i + numberOfActions] = sigmaGradient * advantage;
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinuePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				//ndInt32 index = m_randomPermutation[base + i];
				ndInt32 index = base + i;
				MaxLikelihoodLoss loss(trainer, this, index);
				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
				trainer.BackPropagate(observation, loss);
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
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
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, -m_policyLearnRate);
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeCritic()
{
	m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount() - 1);
	for (ndInt32 i = ndInt32(m_randomPermutation.GetCount()) - 1; i >= 0; --i)
	{
		m_randomPermutation[i] = i;
	}

#if 1
	// using value functions, seem very noisy
	if (m_randomPermutation.GetCount() > ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE)
	{
		m_randomPermutation.SetCount(ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE);
	}
	else
	{
		m_randomPermutation.SetCount((m_randomPermutation.GetCount() - 1) & -m_bashBufferSize);
	}

	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 i = 0; i < ND_CONTINUE_POLICY_STATE_VALUE_ITERATIONS; ++i)
	{ 
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
		for (ndInt32 base = 0; base < m_randomPermutation.GetCount(); base += m_bashBufferSize)
		{
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainLossLeastSquaredError loss(1);
				ndBrainFixSizeVector<1> stateValue;
				ndBrainFixSizeVector<1> stateQValue;
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_criticTrainers[i];

					stateValue[0] = m_trajectoryAccumulator.GetReward(index);
					if (!m_trajectoryAccumulator.GetTerminalState(index))
					{
						const ndBrainMemVector qObservation(m_trajectoryAccumulator.GetObservations(index + 1), m_numberOfObservations);
						m_critic.MakePrediction(qObservation, stateQValue);
						stateValue[0] += m_gamma * stateQValue[0];
					}

					loss.SetTruth(stateValue);

					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			ndBrainThreadPool::ParallelExecute(BackPropagateBash);
			m_criticOptimizer->Update(this, m_criticTrainers, m_criticLearnRate);
		}
	}
#else

	m_randomPermutation.SetCount((m_randomPermutation.GetCount() - 1) & -m_bashBufferSize);

	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 i = 0; i < 1; ++i)
	{
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
		for (ndInt32 base = 0; base < m_randomPermutation.GetCount(); base += m_bashBufferSize)
		{
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainFixSizeVector<1> stateValue;
				ndBrainLossLeastSquaredError loss(1);
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_criticTrainers[i];

					stateValue[0] = m_trajectoryAccumulator.GetStateReward(index);
					loss.SetTruth(stateValue);

					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			ndBrainThreadPool::ParallelExecute(BackPropagateBash);
			m_criticOptimizer->Update(this, m_criticTrainers, m_criticLearnRate);
		}
	}

#endif

}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::CalculateAdvange()
{
	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
	for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator.GetStateReward(i);
	}
	m_averageScore.Update(averageSum / ndBrainFloat(stepNumber));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(stepNumber) / ndBrainFloat(m_bashTrajectoryIndex));
	
	ndAtomic<ndInt32> iterator(0);
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
	auto CalculateAdvantage = ndMakeObject::ndFunction([this, &iterator](ndInt32 threadIndex, ndInt32)
	{
#if 0
		ndBrainFixSizeVector<1> actions;
		ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);
	
		ndInt32 const count = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			m_critic.MakePrediction(observation, actions, workingBuffer);
			ndBrainFloat stateValue = actions[0];
			ndBrainFloat stateReward = m_trajectoryAccumulator.GetStateReward(i);
			ndBrainFloat advantage = stateReward - stateValue;
			m_trajectoryAccumulator.SetAdvantage(i, advantage);
			m_trajectoryAccumulator.SetStateValue(i, stateValue);
		}
#else
		ndBrainFixSizeVector<1> actions;
		ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);

		ndInt32 const count = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation0(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			m_critic.MakePrediction(observation0, actions, workingBuffer);

			ndBrainFloat stateValue0 = actions[0];
			ndBrainFloat stateReward = m_trajectoryAccumulator.GetReward(i);
			ndBrainFloat stateValue1 = ndBrainFloat(0.0f);
			if (!m_trajectoryAccumulator.GetTerminalState(i))
			{
				const ndBrainMemVector observation1(m_trajectoryAccumulator.GetObservations(i + 1), m_numberOfObservations);
				m_critic.MakePrediction(observation1, actions);
				stateValue1 = actions[0];
			}

			ndBrainFloat advantage = stateReward + m_gamma * stateValue1 - stateValue0;
			m_trajectoryAccumulator.SetAdvantage(i, advantage);
			m_trajectoryAccumulator.SetStateValue(i, stateValue0);
		}
#endif
	});
	ndBrainThreadPool::ParallelExecute(CalculateAdvantage);

#if 0
	ndFloat64 advantageVariance2 = ndBrainFloat(0.0f);
	for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	{
		ndBrainFloat advantage = m_trajectoryAccumulator.GetAdvantage(i);
		advantageVariance2 += advantage * advantage;
	}

	advantageVariance2 /= ndBrainFloat(stepNumber);
	advantageVariance2 = ndMax(advantageVariance2, ndFloat64(1.0e2f));
	ndBrainFloat invVariance = ndBrainFloat(1.0f) / ndBrainFloat(ndSqrt(advantageVariance2));
	for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	{
		const ndBrainFloat normalizedAdvantage = m_trajectoryAccumulator.GetAdvantage(i) * invVariance;
		m_trajectoryAccumulator.SetAdvantage(i, normalizedAdvantage);
	}
#endif
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::Optimize()
{
	CalculateAdvange();
	OptimizePolicy();
	OptimizeCritic();
}

#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeStep()
{
	for (ndList<ndBrainAgentContinuePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentContinuePolicyGradient_Trainer* const agent = node->GetInfo();

		ndAssert(agent->m_trajectory.GetCount());
		bool isTeminal = agent->m_trajectory.GetTerminalState(agent->m_trajectory.GetCount() - 1);
		isTeminal = isTeminal || (agent->m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
		if (isTeminal)
		{
			agent->SaveTrajectory();
			agent->ResetModel();
			agent->m_randomGenerator = GetRandomGenerator();
		}
		m_frameCount++;
		m_framesAlive++;
	}

	//ndInt32 trajectoryAccumulatorCount = m_trajectoryAccumulator.GetCount();
	//if ((m_bashTrajectoryIndex >= m_bashTrajectoryCount) && (trajectoryAccumulatorCount >= m_bashTrajectorySteps))
	if (m_bashTrajectoryIndex >= m_bashTrajectoryCount)
	{
		Optimize();

		m_eposideCount++;
		m_framesAlive = 0;
		m_bashTrajectoryIndex = 0;
		m_trajectoryAccumulator.SetCount(0);
		for (ndList<ndBrainAgentContinuePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentContinuePolicyGradient_Trainer* const agent = node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
		}
	}
}