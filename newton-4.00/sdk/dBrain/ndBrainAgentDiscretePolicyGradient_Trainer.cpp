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
#include "ndBrainOptimizerAdamLegacy.h"
#include "ndBrainLayerActivationSoftmax.h"
#include "ndBrainAgentDiscretePolicyGradient_Trainer.h"

#define ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE		(1024 * 128)
#define ND_DISCRETE_CRITIC_STATE_VALUE_ITERATIONS	10
#define ND_DISCRETE_POLICY_PROXIMA_ITERATIONS		20
#define ND_DISCRETE_POLICY_KL_DIVERGENCE			ndBrainFloat(1.0e-3f)

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentDiscretePolicyGradient_TrainerMaster::HyperParameters::HyperParameters()
{
	m_randomSeed = 47;
	m_numberOfHiddenLayers = 3;
	m_miniBatchSize = 256;
	m_hiddenLayersNumberOfNeurons = 64;
	m_maxTrajectorySteps = 4096;
	m_extraTrajectorySteps = 1024;
	//m_batchTrajectoryCount = 100;
	m_batchTrajectoryCount = 500;

	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	//m_criticLearnRate = ndBrainFloat(0.0004f);
	//m_policyLearnRate = ndBrainFloat(0.0002f);
	m_criticLearnRate = ndBrainFloat(0.0001f);
	m_policyLearnRate = ndBrainFloat(0.0002f);

	m_regularizer = ndBrainFloat(1.0e-5f);
	m_discountRewardFactor = ndBrainFloat(0.99f);
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);
//m_threadsCount = 1;
}

//*********************************************************************************************
//
//*********************************************************************************************
#define ND_EXTRA_MEMBERS 4
ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::ndTrajectoryTransition(ndInt32 obsevationsSize, ndInt32 actionsSize)
	:ndBrainVector()
	,m_actionbsSize(actionsSize)
	,m_obsevationsSize(obsevationsSize)
{
}

ndInt32 ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetCount() const
{
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return me[stride * entry];
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	me[stride * entry] = reward;
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetAdvantage(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return me[stride * entry + 1];
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::SetAdvantage(ndInt32 entry, ndBrainFloat advantage)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	me[stride * entry + 1] = advantage;
}

bool ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return (me[stride * entry + 2] == ndBrainFloat(100.0f)) ? true : false;
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	me[stride * entry + 2] = isTernimal ? ndBrainFloat(100.0f) : ndBrainFloat(-100.0f);
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetAction(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return me[stride * entry + 3];
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	ndMemSet(&me[stride * entry], ndBrainFloat(0.0f), stride);
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::SetAction(ndInt32 entry, ndBrainFloat actionIndex)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	me[stride * entry + 3] = actionIndex;
}

ndBrainFloat* ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return &me[stride * entry + ND_EXTRA_MEMBERS];
}

const ndBrainFloat* ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return &me[stride * entry + ND_EXTRA_MEMBERS];
}

ndBrainFloat* ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetProbabilityDistribution(ndInt32 entry)
{
	ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return &me[stride * entry + ND_EXTRA_MEMBERS + m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition::GetProbabilityDistribution(ndInt32 entry) const
{
	const ndTrajectoryTransition& me = *this;
	ndInt64 stride = ND_EXTRA_MEMBERS + m_obsevationsSize + m_actionbsSize;
	return &me[stride * entry + ND_EXTRA_MEMBERS + m_obsevationsSize];
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentDiscretePolicyGradient_TrainerMaster::MemoryStateValues::MemoryStateValues(ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_obsevationsSize(obsevationsSize)
{
	SetCount(ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE * (m_obsevationsSize + 1));
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_TrainerMaster::MemoryStateValues::GetReward(ndInt32 index) const
{
	const ndBrainVector& me = *this;
	return me[index * (m_obsevationsSize + 1)];
}

const ndBrainFloat* ndBrainAgentDiscretePolicyGradient_TrainerMaster::MemoryStateValues::GetObservations(ndInt32 index) const
{
	const ndBrainVector& me = *this;
	return &me[index * (m_obsevationsSize + 1) + 1];
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::MemoryStateValues::SaveTransition(ndInt32 index, ndBrainFloat reward, const ndBrainFloat* const observations)
{
	ndBrainVector& me = *this;
	ndInt64 stride = m_obsevationsSize + 1;
	me[index * stride] = reward;
	ndMemCpy(&me[index * stride + 1], observations, m_obsevationsSize);
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentDiscretePolicyGradient_Trainer::ndBrainAgentDiscretePolicyGradient_Trainer(const ndSharedPtr<ndBrainAgentDiscretePolicyGradient_TrainerMaster>& master)
	:ndBrainAgent()
	,m_workingBuffer()
	,m_trajectory(master->m_numberOfObservations, master->m_numberOfActions)
	,m_master(master)
	,m_randomGenerator(nullptr)
{
	m_master->m_agents.Append(this);
	m_trajectory.SetCount(0);
	m_randomGenerator = m_master->GetRandomGenerator();
}

ndBrainAgentDiscretePolicyGradient_Trainer::~ndBrainAgentDiscretePolicyGradient_Trainer()
{
	for (ndList<ndBrainAgentDiscretePolicyGradient_Trainer*>::ndNode* node = m_master->m_agents.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo() == this)
		{
			m_master->m_agents.Remove(node);
			break;
		}
	}
}

ndBrain* ndBrainAgentDiscretePolicyGradient_Trainer::GetActor()
{
	return m_master->GetPolicyNetwork();
}

bool ndBrainAgentDiscretePolicyGradient_Trainer::IsTerminal() const
{
	return false;
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::SampleActions(const ndBrainVector& probabilities) const
{
	ndBrainFixSizeVector<256> pdf;
	
	pdf.SetCount(0);
	ndBrainFloat sum = ndBrainFloat(0.0f);
	for (ndInt32 i = 0; i < m_master->m_numberOfActions; ++i)
	{
		pdf.PushBack(sum);
		sum += probabilities[i];
	}
	pdf.PushBack(sum);
	
	ndRandomGenerator& generator = *m_randomGenerator;
	ndFloat32 r = generator.m_d(generator.m_gen);
	ndInt32 index = m_master->m_numberOfActions - 1;
	for (ndInt32 i = index; i >= 0; --i)
	{
		index = i;
		if (pdf[i] < r)
		{
			break;
		}
	}
	return ndBrainFloat(index);
}

void ndBrainAgentDiscretePolicyGradient_Trainer::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);


	ndBrainMemVector probability(m_trajectory.GetProbabilityDistribution(entryIndex), m_master->m_numberOfActions);
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), m_master->m_numberOfObservations);

	GetObservation(&observation[0]);
	probability.SetCount(m_master->m_numberOfActions);
	m_master->m_policy.MakePrediction(observation, probability, m_workingBuffer);

	ndBrainFloat action = ndBrainFloat(SampleActions(probability));
	m_trajectory.SetAction(entryIndex, action);
	ApplyActions(&action);

	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetAdvantage(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, IsTerminal());
}

void ndBrainAgentDiscretePolicyGradient_Trainer::SaveTrajectory()
{
	if (m_trajectory.GetCount())
	{
		m_master->m_batchTrajectoryIndex++;

		while (m_trajectory.GetTerminalState(m_trajectory.GetCount() - 2))
		{
			m_trajectory.SetCount(m_trajectory.GetCount() - 1);
		}
		//ndAssert(m_trajectory.GetTerminalState(m_trajectory.GetCount() - 1));
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
		ndTrajectoryTransition& trajectoryAccumulator = m_master->m_trajectoryAccumulator;
		for (ndInt32 i = 0; i < maxSteps; ++i)
		{
			ndInt32 index = trajectoryAccumulator.GetCount();
			trajectoryAccumulator.SetCount(index + 1);
			trajectoryAccumulator.SetReward(index, m_trajectory.GetReward(i));
			trajectoryAccumulator.SetAction(index, m_trajectory.GetAction(i));
			trajectoryAccumulator.SetAdvantage(index, m_trajectory.GetAdvantage(i));
			trajectoryAccumulator.SetTerminalState(index, m_trajectory.GetTerminalState(i));
			ndMemCpy(trajectoryAccumulator.GetProbabilityDistribution(index), m_trajectory.GetProbabilityDistribution(i), m_master->m_numberOfActions);
			ndMemCpy(trajectoryAccumulator.GetObservations(index), m_trajectory.GetObservations(i), m_master->m_numberOfObservations);
		}
	}
	m_trajectory.SetCount(0);
}

ndInt32 ndBrainAgentDiscretePolicyGradient_Trainer::GetEpisodeFrames() const
{
	return ndInt32(m_trajectory.GetCount());
}

// ***************************************************************************************
//
// ***************************************************************************************
ndBrainAgentDiscretePolicyGradient_TrainerMaster::ndBrainAgentDiscretePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
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
	,m_trajectoryAccumulator(hyperParameters.m_numberOfObservations, hyperParameters.m_numberOfActions)
	,m_gamma(hyperParameters.m_discountRewardFactor)
	,m_policyLearnRate(hyperParameters.m_policyLearnRate)
	,m_criticLearnRate(hyperParameters.m_criticLearnRate)
	,m_numberOfActions(hyperParameters.m_numberOfActions)
	,m_numberOfObservations(hyperParameters.m_numberOfObservations)
	,m_framesAlive(0)
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_miniBatchSize(hyperParameters.m_miniBatchSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_batchTrajectoryIndex(0)
	,m_batchTrajectoryCount(hyperParameters.m_batchTrajectoryCount)
	,m_batchTrajectorySteps(hyperParameters.m_batchTrajectoryCount* m_maxTrajectorySteps)
	,m_baseValueWorkingBufferSize(0)
	,m_randomSeed(hyperParameters.m_randomSeed)
	,m_workingBuffer()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_agents()
{
	ndAssert(0);
#if 0
	ndAssert(m_numberOfActions);
	ndAssert(m_numberOfObservations);
	ndSetRandSeed(m_randomSeed);
	
	m_randomGenerator = new ndBrainAgentDiscretePolicyGradient_Trainer::ndRandomGenerator[size_t(hyperParameters.m_batchTrajectoryCount)];
	for (ndInt32 i = 0; i < hyperParameters.m_batchTrajectoryCount; ++i)
	{
		m_randomSeed++;
		m_randomGenerator[i].m_gen.seed(m_randomSeed);
	}

	m_randomSeed = 0;

	// build policy neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, hyperParameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < hyperParameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, hyperParameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(hyperParameters.m_hiddenLayersNumberOfNeurons));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_hiddenLayersNumberOfNeurons, m_numberOfActions));
	layers.PushBack(new ndBrainLayerActivationSoftmax(m_numberOfActions));

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_policy.AddLayer(layers[i]);
	}
	
	m_policy.InitWeights();
	//Normalize(m_policy);
	ndAssert(!strcmp((m_policy[m_policy.GetCount() - 1])->GetLabelId(), "ndBrainLayerActivationSoftmax"));

	m_policyTrainers.SetCount(0);
	m_policyAuxiliaryTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_policy);
		m_policyTrainers.PushBack(trainer);
	
		ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(&m_policy);
		m_policyAuxiliaryTrainers.PushBack(auxiliaryTrainer);
	}
	
	m_policyWeightedTrainer.PushBack(m_policyTrainers[0]);
	m_policyOptimizer = new ndBrainOptimizerAdam();
	m_policyOptimizer->SetRegularizer(hyperParameters.m_regularizer);
	
	// build state value critic neural net
	layers.SetCount(0);
	ndInt32 criticNeurons = hyperParameters.m_hiddenLayersNumberOfNeurons * 1;
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, criticNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 0; i < hyperParameters.m_numberOfHiddenLayers; ++i)
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
	//Normalize(m_critic);
	
	ndAssert(m_critic.GetOutputSize() == 1);
	ndAssert(m_critic.GetInputSize() == m_policy.GetInputSize());
	ndAssert(!strcmp((m_critic[m_critic.GetCount() - 1])->GetLabelId(), ND_BRAIN_LAYER_LINEAR_NAME));
	
	m_criticTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_miniBatchSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_critic);
		m_criticTrainers.PushBack(trainer);
	}
	
	m_criticOptimizer = new ndBrainOptimizerAdam();
	m_criticOptimizer->SetRegularizer(ndBrainFloat(1.0e-3f));
	
	m_baseValueWorkingBufferSize = m_critic.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
#endif
}

ndBrainAgentDiscretePolicyGradient_TrainerMaster::~ndBrainAgentDiscretePolicyGradient_TrainerMaster()
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

//void ndBrainAgentDiscretePolicyGradient_TrainerMaster::Normalize(ndBrain& actor)
void ndBrainAgentDiscretePolicyGradient_TrainerMaster::Normalize(ndBrain&)
{
	ndAssert(0);
#if 0
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
			auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, &stops, &input, &groundTruth](ndInt32, ndInt32)
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
				ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
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

	SupervisedTrainer optimizer(&actor);
	optimizer.Optimize();
#endif
}

ndBrain* ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetPolicyNetwork()
{
	return &m_policy;
}

ndBrain* ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetValueNetwork()
{
	return &m_critic;
}

const ndString& ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetName() const
{
	return m_name;
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::SetName(const ndString& name)
{
	m_name = name;
}

ndBrainAgentDiscretePolicyGradient_Trainer::ndRandomGenerator* ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetRandomGenerator()
{
	m_randomSeed = (m_randomSeed + 1) % m_batchTrajectoryCount;
	return &m_randomGenerator[m_randomSeed];
}

ndUnsigned32 ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetFramesCount() const
{
	return m_frameCount;
}

bool ndBrainAgentDiscretePolicyGradient_TrainerMaster::IsSampling() const
{
	return false;
}

ndUnsigned32 ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

ndFloat32 ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetAverageScore() const
{
	return m_averageExpectedRewards.GetAverage();
}

#ifdef ND_DISCRETE_PROXIMA_POLICY_GRADIENT

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentDiscretePolicyGradient_TrainerMaster::CalculateKLdivergence()
{
	ndFloat64 partialEntropy[256];
	ndFloat64 partialCrossEntropy[256];

	ndAtomic<ndInt32> iterator(0);
	auto ParcialDivergence = ndMakeObject::ndFunction([this, &iterator, &partialEntropy, &partialCrossEntropy](ndInt32 threadIndex, ndInt32)
	{
		ndFloat64 totalEntropy = ndFloat32(0.0f);
		ndFloat64 totalCrossEntropy = ndFloat32(0.0f);
		ndInt32 size = m_trajectoryAccumulator.GetCount();

		ndBrainFloat crossProbabilitiesBuffer[256];
		ndBrainMemVector crossProbabilities(&crossProbabilitiesBuffer[0], m_numberOfActions);
		for (ndInt32 i = iterator++; i < size; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			ndFloat32 entropy = ndFloat32(0.0f);
			ndBrainMemVector entropyProbabilities(m_trajectoryAccumulator.GetProbabilityDistribution(i), m_numberOfActions);
			for (ndInt32 j = m_numberOfActions - 1; j >= 0; --j)
			{
				entropy += entropyProbabilities[j] * ndLog(entropyProbabilities[j]);
			}
			totalEntropy += entropy;

			ndFloat32 crossEntropy = ndFloat32(0.0f);
			m_policy.MakePrediction(observation, crossProbabilities);
			for (ndInt32 j = m_numberOfActions - 1; j >= 0; --j)
			{
				crossEntropy += entropyProbabilities[j] * ndLog(crossProbabilities[j]);
			}
			totalCrossEntropy += crossEntropy;
		}
		partialEntropy[threadIndex] = totalEntropy;
		partialCrossEntropy[threadIndex] = totalCrossEntropy;
	});
	ndBrainThreadPool::ParallelExecute(ParcialDivergence);

	ndFloat64 totalEntropy = ndFloat32(0.0f);
	ndFloat64 totalCrossEntropy = ndFloat32(0.0f);
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		totalEntropy += partialEntropy[i];
		totalCrossEntropy += partialCrossEntropy[i];
	}

	ndFloat32 divergence = ndFloat32((totalEntropy - totalCrossEntropy) / ndFloat64(m_trajectoryAccumulator.GetCount()));
	ndAssert(divergence >= 0.0f);
	return divergence;
}

//#pragma optimize( "", off )
void ndBrainAgentDiscretePolicyGradient_TrainerMaster::OptimizePolicyPPOstep()
{
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_policyTrainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount());

	for (ndInt32 base = 0; base < steps; base += m_miniBatchSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class Loss : public ndBrainLossLeastSquaredError
			{
				public:
				Loss(ndBrainTrainer& trainer, ndBrainAgentDiscretePolicyGradient_TrainerMaster* const agent, ndInt32 index)
					:ndBrainLossLeastSquaredError(trainer.GetBrain()->GetOutputSize())
					,m_trainer(trainer)
					,m_agent(agent)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					ndInt32 actionIndex = ndInt32(m_agent->m_trajectoryAccumulator.GetAction(m_index));
					const ndBrainMemVector observation(m_agent->m_trajectoryAccumulator.GetObservations(m_index), m_agent->m_numberOfObservations);

					const ndBrainMemVector oldProbabilities(m_agent->m_trajectoryAccumulator.GetProbabilityDistribution(m_index), m_agent->m_numberOfActions);
					ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);

					ndBrainFloat probability = probabilityDistribution[actionIndex];
					ndBrainFloat oldProbability = ndMax(oldProbabilities[actionIndex], ndBrainFloat(1.0e-5f));

					ndBrainFloat r = probability / oldProbability;
					ndBrainFloat g = (advantage > ndBrainFloat(0.0f)) ? ndBrainFloat(1.0 + 0.2f) : ndBrainFloat(1.0 - 0.2f);
					
					// clip the advantage
					advantage = ndMin(r * advantage, g * advantage);
					
					loss.Set(ndBrainFloat(0.0f));
					ndBrainFloat logProb = ndBrainFloat(-ndLog(probability));
					loss[actionIndex] = logProb * advantage;
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentDiscretePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					Loss loss(trainer, this, base + i);
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
			for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
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
	m_policyWeightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, -m_policyLearnRate);
}
#endif

//#pragma optimize( "", off )
void ndBrainAgentDiscretePolicyGradient_TrainerMaster::OptimizePolicy()
{
	ndAssert(0);
#if 0
	ndAtomic<ndInt32> iterator(0);
	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
		{
			ndBrainTrainer* const trainer = m_policyTrainers[i];
			trainer->ClearGradients();
		}
	});
	ndBrainThreadPool::ParallelExecute(ClearGradients);

	const ndInt32 steps = ndInt32(m_trajectoryAccumulator.GetCount());
	for (ndInt32 base = 0; base < steps; base += m_miniBatchSize)
	{
		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
		{
			class Loss : public ndBrainLossLeastSquaredError
			{
				public:
				Loss(ndBrainTrainer& trainer, ndBrainAgentDiscretePolicyGradient_TrainerMaster* const agent, ndInt32 index)
					:ndBrainLossLeastSquaredError(trainer.GetBrain()->GetOutputSize())
					,m_trainer(trainer)
					,m_agent(agent)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
				{
					ndBrainFloat advantage = m_agent->m_trajectoryAccumulator.GetAdvantage(m_index);

					loss.Set(ndBrainFloat(0.0f));
					ndInt32 actionIndex = ndInt32(m_agent->m_trajectoryAccumulator.GetAction(m_index));
					ndBrainFloat logProb = ndBrainFloat(-ndLog(probabilityDistribution[actionIndex]));
					loss[actionIndex] = logProb * advantage;
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentDiscretePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
				Loss loss(trainer, this, base + i);
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
			for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
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
	m_policyWeightedTrainer[0]->ScaleWeights(ndBrainFloat(1.0f) / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_policyOptimizer->Update(this, m_policyWeightedTrainer, -m_policyLearnRate);
#endif
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::UpdateBaseLineValue()
{
	ndAssert(0);
#if 0
	m_trajectoryAccumulator.SetTerminalState(m_trajectoryAccumulator.GetCount() - 2, true);

	m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount() - 1);
	for (ndInt32 i = ndInt32(m_randomPermutation.GetCount()) - 1; i >= 0; --i)
	{
		m_randomPermutation[i] = i;
	}
	m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());

	if (m_randomPermutation.GetCount() > ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE)
	{
		m_randomPermutation.SetCount(ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE);
	}
	else
	{
		ndInt32 smallSize = ndInt32(m_randomPermutation.GetCount()) & -m_miniBatchSize;
		m_randomPermutation.SetCount(smallSize);
	}

	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 i = 0; i < ND_DISCRETE_CRITIC_STATE_VALUE_ITERATIONS; ++i)
	{ 
		for (ndInt32 base = 0; base < m_randomPermutation.GetCount(); base += m_miniBatchSize)
		{
			auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainFixSizeVector<1> stateValue;
				ndBrainFixSizeVector<1> stateQValue;
				ndBrainLossLeastSquaredError loss(1);
				for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
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
			ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
			m_criticOptimizer->Update(this, m_criticTrainers, m_criticLearnRate);
		}
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
	}
#endif
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::OptimizeCritic()
{
	UpdateBaseLineValue();

	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
	for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator.GetAdvantage(i);
	}
	m_averageExpectedRewards.Update(averageSum / ndBrainFloat(stepNumber));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(stepNumber) / ndBrainFloat(m_batchTrajectoryIndex));
	
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

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::Optimize()
{
	OptimizeCritic();
#ifdef ND_DISCRETE_PROXIMA_POLICY_GRADIENT
	OptimizePolicy();
	for (ndInt32 i = 0; (i < ND_DISCRETE_POLICY_PROXIMA_ITERATIONS) && (CalculateKLdivergence() < ND_DISCRETE_POLICY_KL_DIVERGENCE); ++i)
	{
		OptimizePolicyPPOstep();
	}
#else
	OptimizePolicy();
#endif
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::OptimizeStep()
{
	for (ndList<ndBrainAgentDiscretePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentDiscretePolicyGradient_Trainer* const agent = node->GetInfo();

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

	ndInt32 trajectoryAccumulatorCount = m_trajectoryAccumulator.GetCount();
	if ((m_batchTrajectoryIndex >= m_batchTrajectoryCount) && (trajectoryAccumulatorCount >= m_batchTrajectorySteps))
	{
		Optimize();
		m_eposideCount++;
		m_framesAlive = 0;
		m_batchTrajectoryIndex = 0;
		m_trajectoryAccumulator.SetCount(0);
		for (ndList<ndBrainAgentDiscretePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentDiscretePolicyGradient_Trainer* const agent = node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
		}
	}
}
