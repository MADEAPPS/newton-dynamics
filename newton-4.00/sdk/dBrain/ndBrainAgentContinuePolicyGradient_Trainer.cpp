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
#include "ndBrainAgentContinuePolicyGradient_Trainer.h"

#define ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE		(1024 * 128)
#define ND_CONTINUE_POLICY_STATE_VALUE_ITERATIONS	10
#define ND_CONTINUE_POLICY_GRADIENT_MIN_SIGMA		ndBrainFloat(1.0e-1f)

#define ND_CONTINUE_POLICY_PROXIMA_ITERATIONS		20
#define ND_CONTINUE_POLICY_KL_DIVERGENCE			ndBrainFloat(1.0e-3f)


//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters::HyperParameters()
{
	m_randomSeed = 47;
	m_numberOfLayers = 4;
	m_bashBufferSize = 256;
	m_neuronPerLayers = 64;
	m_maxTrajectorySteps = 4096;
	m_extraTrajectorySteps = 1024;
	//m_bashTrajectoryCount = 100;
	m_bashTrajectoryCount = 500;

	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	//m_criticLearnRate = ndBrainFloat(0.0001f);
	//m_policyLearnRate = ndBrainFloat(0.0001f);
	m_criticLearnRate = ndBrainFloat(0.0001f);
	m_policyLearnRate = ndBrainFloat(0.0002f);

	m_regularizer = ndBrainFloat(1.0e-6f);
	m_discountFactor = ndBrainFloat(0.99f);
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize);
//m_threadsCount = 1;
}

//*********************************************************************************************
//
//*********************************************************************************************
class ndBrainAgentContinuePolicyGradient_TrainerMaster::ndPolicyGradientActivation : public ndBrainLayerActivationTanh
{
	public:
	//#define USE_TANH_FOR_SIGMA

	ndPolicyGradientActivation(ndInt32 neurons)
		:ndBrainLayerActivationTanh(neurons * 2)
		,m_minimumSigma(ND_CONTINUE_POLICY_GRADIENT_MIN_SIGMA)
	{
	}

	ndPolicyGradientActivation(const ndPolicyGradientActivation& src)
		:ndBrainLayerActivationTanh(src)
		,m_minimumSigma(src.m_minimumSigma)
	{
	}

	ndBrainLayer* Clone() const
	{
		return new ndPolicyGradientActivation(*this);
	}

	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
	{
		ndBrainLayerActivationTanh::MakePrediction(input, output);
		for (ndInt32 j = m_neurons / 2 - 1; j >= 0; --j)
		{
			ndInt32 i = j + m_neurons / 2;

			#ifdef USE_TANH_FOR_SIGMA
				//ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
				//ndBrainFloat x0 = m_minimumSigma + 0.5f + 0.5f * ndBrainFloat(ndTanh(value));
				//ndBrainFloat x1 = m_minimumSigma + ndBrainFloat(0.5f) * (ndBrainFloat(1.0f) + output[i]);
				output[i] = m_minimumSigma + ndBrainFloat(0.5f) * (ndBrainFloat(1.0f) + output[i]);
			#else
				output[i] = ndMax(input[i], m_minimumSigma);
			#endif
		}
	}

	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
	{
		ndBrainLayerActivationTanh::InputDerivative(input, output, outputDerivative, inputDerivative);
		for (ndInt32 j = m_neurons / 2 - 1; j >= 0; --j)
		{
			ndInt32 i = j + m_neurons / 2;
			
			#ifdef USE_TANH_FOR_SIGMA
				ndBrainFloat x = output[i] - m_minimumSigma - ndBrainFloat(0.5f);
				ndBrainFloat derivative = ndBrainFloat(0.5f) - ndBrainFloat(2.0f) * x * x;
			#else
				ndBrainFloat derivative = (input[i] >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
			#endif
			inputDerivative[i] = outputDerivative[i] * derivative;
		}
	}

	ndBrainFloat m_minimumSigma;
};

//*********************************************************************************************
//
//*********************************************************************************************
#define ND_EXTRA_MEMBERS 3
ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::ndTrajectoryStep(ndInt32 actionsSize, ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_actionsSize(actionsSize)
	,m_obsevationsSize(obsevationsSize)
{
}

ndInt32 ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetCount() const
{
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::SetCount(ndInt32 count)
{
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetReward(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return me[stride * entry];
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry] = reward;
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetAdvantage(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return me[stride * entry + 1];
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::SetAdvantage(ndInt32 entry, ndBrainFloat advantage)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry + 1] = advantage;
}

bool ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetTerminalState(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return (me[stride * entry + 2] == ndBrainFloat(100.0f)) ? true : false;
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	me[stride * entry + 2] = isTernimal ? ndBrainFloat(100.0f) : ndBrainFloat(-100.0f);
}

void ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::Clear(ndInt32 entry)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	ndMemSet(&me[stride * entry], ndBrainFloat(0.0f), stride);
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetObservations(ndInt32 entry)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + ND_EXTRA_MEMBERS];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetObservations(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + ND_EXTRA_MEMBERS];
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetActions(ndInt32 entry)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + m_obsevationsSize + ND_EXTRA_MEMBERS];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetActions(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry + m_obsevationsSize + ND_EXTRA_MEMBERS];
}
ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetProbabilities(ndInt32 entry)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 4 + m_obsevationsSize;
	return &me[stride * entry+ m_obsevationsSize + m_actionsSize * 2 + ND_EXTRA_MEMBERS];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::GetProbabilities(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_actionsSize * 2 + m_obsevationsSize;
	return &me[stride * entry + m_obsevationsSize+ m_actionsSize * 2 + ND_EXTRA_MEMBERS];
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

void ndBrainAgentContinuePolicyGradient_Trainer::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), m_master->m_numberOfActions * 2);
	ndBrainMemVector probabilities(m_trajectory.GetProbabilities(entryIndex), m_master->m_numberOfActions * 2);
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), m_master->m_numberOfObservations);
	
	GetObservation(&observation[0]);
	m_master->m_policy.MakePrediction(observation, actions, m_workingBuffer);
	probabilities.Set(actions);
	 
	SelectAction(actions);
	ApplyActions(&actions[0]);

	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetAdvantage(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());
}

void ndBrainAgentContinuePolicyGradient_Trainer::SaveTrajectory()
{
	if (m_trajectory.GetCount())
	{
		m_master->m_bashTrajectoryIndex++;
		
		m_trajectory.SetTerminalState(m_trajectory.GetCount() - 1, true);

		// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
		ndBrainFloat gamma = m_master->m_gamma;
		for (ndInt32 i = m_trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndBrainFloat r0 = m_trajectory.GetAdvantage(i);
			ndBrainFloat r1 = m_trajectory.GetAdvantage(i + 1);
			m_trajectory.SetAdvantage(i, r0 + gamma * r1);
		}
		
		// get the max trajectory steps
		const ndInt32 maxSteps = ndMin(m_trajectory.GetCount(), m_master->m_maxTrajectorySteps);
		ndAssert(maxSteps > 0);
		ndTrajectoryStep& trajectoryAccumulator = m_master->m_trajectoryAccumulator;
		for (ndInt32 i = 0; i < maxSteps; ++i)
		{
			ndInt32 index = trajectoryAccumulator.GetCount();
			trajectoryAccumulator.SetCount(index + 1);
			trajectoryAccumulator.SetReward(index, m_trajectory.GetReward(i));
			trajectoryAccumulator.SetAdvantage(index, m_trajectory.GetAdvantage(i));
			trajectoryAccumulator.SetTerminalState(index, m_trajectory.GetTerminalState(i));
			ndMemCpy(trajectoryAccumulator.GetActions(index), m_trajectory.GetActions(i), m_master->m_numberOfActions * 2);
			ndMemCpy(trajectoryAccumulator.GetObservations(index), m_trajectory.GetObservations(i), m_master->m_numberOfObservations);
			ndMemCpy(trajectoryAccumulator.GetProbabilities(index), m_trajectory.GetProbabilities(i), m_master->m_numberOfActions * 2);
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
	,m_optimizer(nullptr)
	,m_trainers()
	,m_weightedTrainer()
	,m_auxiliaryTrainers()
	,m_baseLineValueOptimizer(nullptr)
	,m_baseLineValueTrainers()
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
	for (ndInt32 i = 0; i < hyperParameters.m_bashTrajectoryCount; ++i)
	{
		m_randomSeed++;
		m_randomGenerator[i].m_gen.seed(m_randomSeed);

		//ndBrainFloat xxx = m_randomGenerator[i].m_d(m_randomGenerator[i].m_gen);
		//xxx *= 1;
	}
	m_randomSeed = 0;

	// build policy neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;

	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, hyperParameters.m_neuronPerLayers));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, hyperParameters.m_neuronPerLayers));
		layers.PushBack(new ndBrainLayerActivationTanh(hyperParameters.m_neuronPerLayers));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, m_numberOfActions * 2));
	layers.PushBack(new ndPolicyGradientActivation(m_numberOfActions));

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_policy.AddLayer(layers[i]);
	}

	m_policy.InitWeights();
	NormalizePolicy();
	ndAssert(!strcmp((m_policy[m_policy.GetCount() - 1])->GetLabelId(), "ndBrainLayerActivationTanh"));

	m_trainers.SetCount(0);
	m_auxiliaryTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_policy);
		m_trainers.PushBack(trainer);
	
		ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(&m_policy);
		m_auxiliaryTrainers.PushBack(auxiliaryTrainer);
	}
	
	m_weightedTrainer.PushBack(m_trainers[0]);
	m_optimizer = new ndBrainOptimizerAdam();
	m_optimizer->SetRegularizer(hyperParameters.m_regularizer);

	// build state value critic neural net
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, hyperParameters.m_neuronPerLayers * 2));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers * 2);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), hyperParameters.m_neuronPerLayers * 2));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_critic.AddLayer(layers[i]);
	}
	m_critic.InitWeights();
	NormalizeCritic();
	
	ndAssert(m_critic.GetOutputSize() == 1);
	ndAssert(m_critic.GetInputSize() == m_policy.GetInputSize());
	ndAssert(!strcmp((m_critic[m_critic.GetCount() - 1])->GetLabelId(), "ndBrainLayerLinear"));
	
	m_baseLineValueTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic);
		m_baseLineValueTrainers.PushBack(trainer);
	}
	
	m_baseLineValueOptimizer = new ndBrainOptimizerAdam();
	m_baseLineValueOptimizer->SetRegularizer(ndBrainFloat(1.0e-4f));
	
	m_baseValueWorkingBufferSize = m_critic.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
}

ndBrainAgentContinuePolicyGradient_TrainerMaster::~ndBrainAgentContinuePolicyGradient_TrainerMaster()
{
	for (ndInt32 i = 0; i < m_trainers.GetCount(); ++i)
	{
		delete m_trainers[i];
		delete m_auxiliaryTrainers[i];
	}
	delete m_optimizer;
	
	for (ndInt32 i = 0; i < m_baseLineValueTrainers.GetCount(); ++i)
	{
		delete m_baseLineValueTrainers[i];
	}

	delete m_baseLineValueOptimizer;
	delete[] m_randomGenerator;
}

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

			ndPolicyGradientActivation* const lastLayer = (ndPolicyGradientActivation*)m_brain[m_brain.GetCount() - 1];
			ndBrainFloat sigma = lastLayer->m_minimumSigma;
			input.Set(0.0f);
			for (ndInt32 i = 0; i < groundTruth.GetCount() / 2; ++i)
			{
				groundTruth[i] = ndBrainFloat(0.0f);
				groundTruth[i + groundTruth.GetCount() / 2] = sigma;
			}

			ndBrainFloat* const outMemory1 = ndAlloca(ndBrainFloat, m_brain.GetOutputSize());
			ndBrainMemVector output1(outMemory1, m_brain.GetOutputSize());

			for (ndInt32 i = 0; (i < 10000) && !stops; ++i)
			{
				ndBrainThreadPool::ParallelExecute(BackPropagateBash);
				optimizer.Update(this, m_partialGradients, m_learnRate);
			}
			m_brain.MakePrediction(input, output1);
			sigma *= 1.0f;
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
			ndBrainFloat* const outMemory1 = ndAlloca(ndBrainFloat, m_brain.GetOutputSize());
			ndBrainMemVector output1(outMemory1, m_brain.GetOutputSize());
			m_brain.MakePrediction(input, output1);
			m_brain.MakePrediction(input, output1);
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
	m_randomSeed = (m_randomSeed + 1) % m_bashTrajectoryCount;
	return &m_randomGenerator[m_randomSeed];
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

#ifdef ND_CONTINUE_PROXIMA_POLICY_GRADIENT
//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentContinuePolicyGradient_TrainerMaster::CalculateKLdivergence()
{
	//https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	// since I am using a diagnall sigma, I do not have to use cholesky 

	ndAtomic<ndInt32> iterator(0);
	ndFloat64 partialDivergence[256];
	auto ParcialDivergence = ndMakeObject::ndFunction([this, &iterator, &partialDivergence](ndInt32 threadIndex, ndInt32)
	{
		ndFloat64 totalDivergece = ndFloat32(0.0f);
		ndInt32 size = m_trajectoryAccumulator.GetCount();

		ndBrainFloat crossProbabilitiesBuffer[256];
		ndBrainMemVector crossProbabilities(&crossProbabilitiesBuffer[0], m_numberOfActions * 2);
		for (ndInt32 i = iterator++; i < size; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			ndBrainMemVector entropyProbabilities(m_trajectoryAccumulator.GetProbabilities(i), m_numberOfActions * 2);

			m_policy.MakePrediction(observation, crossProbabilities);

			// caculate t0 = trace(inv(Sigma_q) * Sigma_p
			// caculate t1 = trans(Uq - Up) * inv(Sigma_q) * (Uq - Up)
			// caculate t2 = log(det(Sigma_q) /det(Sigma_p))
			ndFloat64 t0 = 0.0f;
			ndFloat64 t1 = 0.0f;
			ndFloat64 det_p = 1.0f;
			ndFloat64 det_q = 1.0f;
			for (ndInt32 j = m_numberOfActions - 1; j >= 0; --j)
			{
				ndBrainFloat sigma_q = crossProbabilities[j + m_numberOfActions];
				ndBrainFloat sigma_p = entropyProbabilities[j + m_numberOfActions];
				ndBrainFloat invSigma_q = 1.0f / sigma_q;

				det_p *= sigma_p;
				det_q *= sigma_q;
				t0 += sigma_p * invSigma_q;
				ndBrainFloat meanError(crossProbabilities[j] - entropyProbabilities[j]);
				t1 += meanError * invSigma_q * meanError;
			}
			ndFloat64 t2 = ndLog(det_q / det_p);
			totalDivergece += ndBrainFloat(0.5f) * (t0 - ndBrainFloat(m_numberOfActions) + t1 + t2);
		}
		partialDivergence[threadIndex] = totalDivergece;
	});
	ndBrainThreadPool::ParallelExecute(ParcialDivergence);

	ndFloat64 divergence = ndFloat32(0.0f);
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		divergence += partialDivergence[i];
	}
	ndAssert(divergence >= 0.0f);
	divergence /= ndFloat64(m_trajectoryAccumulator.GetCount());
	return ndBrainFloat(divergence);
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizePolicyPPOstep()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_trainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount());
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

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					// basically this fits a multivariate Gaussian process with zero cross covariance to the actions.
					const ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					const ndBrainFloat* const actions = m_agent->m_trajectoryAccumulator.GetActions(m_index);
					const ndInt32 numberOfActions = m_agent->m_numberOfActions;

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = output[i];
						const ndBrainFloat sigma1 = output[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat sigma3 = sigma2 * sigma1;
						const ndBrainFloat num = (actions[i] - mean);

						// this was a huge bug, it is gradient ascend
						ndBrainFloat meanGradient = -num / sigma2;
						ndBrainFloat sigmaGradient = num * num / sigma3 - ndBrainFloat(1.0f) / sigma1;

						loss[i] = -meanGradient * advantage;
						loss[i + numberOfActions] = -sigmaGradient * advantage;
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinuePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_auxiliaryTrainers[i];
				MaxLikelihoodLoss loss(trainer, this, base + i);
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(base + i), m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
				else
				{
					trainer.ClearGradients();
				}
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_trainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_auxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}

	m_optimizer->AccumulateGradients(this, m_trainers);
	m_weightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_optimizer->Update(this, m_weightedTrainer, -m_policyLearnRate);
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizePolicy()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_trainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount());
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

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					// basically this fits a multivariate Gaussian process with zero cross covariance to the actions.
					const ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					const ndBrainFloat* const actions = m_agent->m_trajectoryAccumulator.GetActions(m_index);
					const ndInt32 numberOfActions = m_agent->m_numberOfActions;

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = output[i];
						const ndBrainFloat sigma1 = output[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat sigma3 = sigma2 * sigma1;
						const ndBrainFloat num = (actions[i] - mean);

						// this was a huge bug, it is gradient ascend
						ndBrainFloat meanGradient = -num / sigma2;
						ndBrainFloat sigmaGradient = num * num / sigma3 - ndBrainFloat(1.0f) / sigma1;

						loss[i] = -meanGradient * advantage;
						loss[i + numberOfActions] = -sigmaGradient * advantage;
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinuePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_auxiliaryTrainers[i];
				MaxLikelihoodLoss loss(trainer, this, base + i);
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(base + i), m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
				else
				{
					trainer.ClearGradients();
				}
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_trainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_auxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}

	m_optimizer->AccumulateGradients(this, m_trainers);
	m_weightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_optimizer->Update(this, m_weightedTrainer, -m_policyLearnRate);
}

#else

void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizePolicy()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_trainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount());
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

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					// basically this fits a multivariate Gaussian process with zero cross covariance to the actions.
					const ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);
					const ndBrainFloat* const actions = m_agent->m_trajectoryAccumulator.GetActions(m_index);
					const ndInt32 numberOfActions = m_agent->m_numberOfActions;

					for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = output[i];
						const ndBrainFloat sigma1 = output[i + numberOfActions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat sigma3 = sigma2 * sigma1;
						const ndBrainFloat num = (actions[i] - mean);

						// this was a huge bug, it is gradient ascend
						ndBrainFloat meanGradient = -num / sigma2;
						ndBrainFloat sigmaGradient = num * num / sigma3 - ndBrainFloat(1.0f) / sigma1;

						loss[i] = -meanGradient * advantage;
						loss[i + numberOfActions] = -sigmaGradient * advantage;
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinuePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_auxiliaryTrainers[i];
				MaxLikelihoodLoss loss(trainer, this, base + i);
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(base + i), m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
				else
				{
					trainer.ClearGradients();
				}
			}
		});

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_trainers[i];
				const ndBrainTrainer* const auxiliaryTrainer = m_auxiliaryTrainers[i];
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}

	m_optimizer->AccumulateGradients(this, m_trainers);
	m_weightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_optimizer->Update(this, m_weightedTrainer, -m_policyLearnRate);
}
#endif

void ndBrainAgentContinuePolicyGradient_TrainerMaster::UpdateBaseLineValue()
{
	m_trajectoryAccumulator.SetTerminalState(m_trajectoryAccumulator.GetCount() - 2, true);

	m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount() - 1);
	for (ndInt32 i = ndInt32(m_randomPermutation.GetCount()) - 1; i >= 0; --i)
	{
		m_randomPermutation[i] = i;
	}
	m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());

	if (m_randomPermutation.GetCount() > ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE)
	{
		m_randomPermutation.SetCount(ND_CONTINUE_POLICY_GRADIENT_BUFFER_SIZE);
	}
	else
	{
		ndInt32 smallSize = ndInt32(m_randomPermutation.GetCount()) & -m_bashBufferSize;
		m_randomPermutation.SetCount(smallSize);
	}

	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 i = 0; i < ND_CONTINUE_POLICY_STATE_VALUE_ITERATIONS; ++i)
	{ 
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
					ndBrainTrainer& trainer = *m_baseLineValueTrainers[i];

					stateValue[0] = ndBrainFloat(0.0f);
					if (!m_trajectoryAccumulator.GetTerminalState(index))
					{
						stateValue[0] = m_trajectoryAccumulator.GetReward(index);
						if (!m_trajectoryAccumulator.GetTerminalState(index + 1))
						{
							const ndBrainMemVector qObservation(m_trajectoryAccumulator.GetObservations(index + 1), m_numberOfObservations);
							m_critic.MakePrediction(qObservation, stateQValue);
							stateValue[0] += m_gamma * stateQValue[0];
						}
					}

					loss.SetTruth(stateValue);

					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			ndBrainThreadPool::ParallelExecute(BackPropagateBash);
			m_baseLineValueOptimizer->Update(this, m_baseLineValueTrainers, m_criticLearnRate);
		}
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
	}
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeCritic()
{
	UpdateBaseLineValue();

	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
	for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator.GetAdvantage(i);
	}
	m_averageScore.Update(averageSum / ndBrainFloat(stepNumber));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(stepNumber) / ndBrainFloat(m_bashTrajectoryIndex));
	
	ndAtomic<ndInt32> iterator(0);
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
	auto CalculateAdvantage = ndMakeObject::ndFunction([this, &iterator](ndInt32 threadIndex, ndInt32)
	{
		ndBrainFixSizeVector<1> actions;
		ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);
	
		ndInt32 const count = m_trajectoryAccumulator.GetCount();
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			m_critic.MakePrediction(observation, actions, workingBuffer);
			ndBrainFloat baseLine = actions[0];
			ndBrainFloat reward = m_trajectoryAccumulator.GetAdvantage(i);
			ndBrainFloat advantage = reward - baseLine;
			m_trajectoryAccumulator.SetAdvantage(i, advantage);
		}
	});
	ndBrainThreadPool::ParallelExecute(CalculateAdvantage);
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::Optimize()
{
	OptimizeCritic();
#ifdef ND_CONTINUE_PROXIMA_POLICY_GRADIENT
	OptimizePolicy();
	for (ndInt32 i = 0; (i < ND_CONTINUE_POLICY_PROXIMA_ITERATIONS) && (CalculateKLdivergence() < ND_CONTINUE_POLICY_KL_DIVERGENCE); ++i)
	{
		OptimizePolicyPPOstep();
	}
#else
	OptimizePolicy();
#endif
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeStep()
{
	for (ndList<ndBrainAgentContinuePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentContinuePolicyGradient_Trainer* const agent = node->GetInfo();

		bool isTeminal = agent->IsTerminal() || (agent->m_trajectory.GetCount() >= (m_extraTrajectorySteps + m_maxTrajectorySteps));
		if (isTeminal)
		{
			agent->SaveTrajectory();
			agent->ResetModel();
			agent->m_randomGenerator = GetRandomGenerator();
		}
		m_frameCount++;
		m_framesAlive++;
	}

	//if ((m_bashTrajectoryIndex >= (m_bashTrajectoryCount * 10)) || (m_trajectoryAccumulator.GetCount() >= m_bashTrajectorySteps))
	if ((m_bashTrajectoryIndex >= m_bashTrajectoryCount) && (m_trajectoryAccumulator.GetCount() >= m_bashTrajectorySteps))
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