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
#include "ndBrainLayerActivationSoftmax.h"
#include "ndBrainAgentDiscretePolicyGradient_Trainer.h"

#define ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE		(1024 * 256)

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentDiscretePolicyGradient_TrainerMaster::HyperParameters::HyperParameters()
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

	//m_criticLearnRate = ndBrainFloat(0.0004f);
	//m_policyLearnRate = ndBrainFloat(0.0002f);
	m_criticLearnRate = ndBrainFloat(0.0002f);
	m_policyLearnRate = ndBrainFloat(0.0001f);

	m_regularizer = ndBrainFloat(1.0e-6f);
	m_discountFactor = ndBrainFloat(0.99f);
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize);
	//m_threadsCount = 1;
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::ndTrajectoryStep(ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_obsevationsSize(obsevationsSize)
{
}

ndInt32 ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::GetCount() const
{
	ndInt64 stride = 3 + m_obsevationsSize;
	return ndInt32(ndBrainVector::GetCount() / stride);
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::SetCount(ndInt32 count)
{
	ndInt64 stride = 3 + m_obsevationsSize;
	ndBrainVector::SetCount(stride * count);
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::GetReward(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = 3 + m_obsevationsSize;
	return me[stride * entry];
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = 3 + m_obsevationsSize;
	me[stride * entry] = reward;
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::GetAdvantage(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = 3 + m_obsevationsSize;
	return me[stride * entry + 1];
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::SetAdvantage(ndInt32 entry, ndBrainFloat advantage)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = 3 + m_obsevationsSize;
	me[stride * entry + 1] = advantage;
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::GetAction(ndInt32 entry) const
{
	const ndTrajectoryStep& me = *this;
	ndInt64 stride = 3 + m_obsevationsSize;
	return me[stride * entry + 2];
}

void ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::SetAction(ndInt32 entry, ndBrainFloat actionIndex)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = 3 + m_obsevationsSize;
	me[stride * entry + 2] = actionIndex;
}

ndBrainFloat* ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryStep::GetObservations(ndInt32 entry)
{
	ndTrajectoryStep& me = *this;
	ndInt64 stride = 3 + m_obsevationsSize;
	return &me[stride * entry + 3];
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
	,m_trajectory(master->m_numberOfObservations)
	,m_master(master)
	,m_randomGenerator(nullptr)
{
	m_master->m_agents.Append(this);
	m_trajectory.SetCount(m_master->m_maxTrajectorySteps + m_master->m_extraTrajectorySteps);
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

ndInt32 ndBrainAgentDiscretePolicyGradient_Trainer::GetEpisodeFrames() const
{
	return ndInt32(m_trajectory.GetCount());
}

ndBrainFloat ndBrainAgentDiscretePolicyGradient_Trainer::SelectAction(const ndBrainVector& probabilities) const
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
	ndBrainFixSizeVector<256> probability;
	probability.SetCount(m_master->m_numberOfActions);

	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), m_master->m_numberOfObservations);

	GetObservation(&observation[0]);
	m_master->m_policy.MakePrediction(observation, probability, m_workingBuffer);

	ndBrainFloat action = ndBrainFloat(SelectAction(probability));
	m_trajectory.SetAction(entryIndex, action);

	ApplyActions(&action);
	m_trajectory.SetReward(entryIndex, CalculateReward());
	m_trajectory.SetAdvantage(entryIndex, ndBrainFloat(0.0f));
}

void ndBrainAgentDiscretePolicyGradient_Trainer::SaveTrajectory()
{
	if (m_trajectory.GetCount())
	{
		m_master->m_bashTrajectoryIndex++;
	}
	// remove last step because if it was a dead state, it will provide misleading feedback.
	//m_trajectory.SetCount(m_trajectory.GetCount() - 1);

	// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
	ndBrainFloat gamma = m_master->m_gamma;
	for (ndInt32 i = ndInt32(m_trajectory.GetCount()) - 2; i >= 0; --i)
	{
		//m_trajectory[i].m_reward += m_master->m_gamma * m_trajectory[i + 1].m_reward;
		ndBrainFloat r0 = m_trajectory.GetReward(i);
		ndBrainFloat r1 = m_trajectory.GetReward(i + 1);
		m_trajectory.SetReward(i, r0 + gamma * r1);
	}

	// get the max trajectory steps
	const ndInt32 maxSteps = ndMin(ndInt32(m_trajectory.GetCount()), m_master->m_maxTrajectorySteps);
	ndAssert(maxSteps > 0);
	ndTrajectoryStep& trajectoryAccumulator = m_master->m_trajectoryAccumulator;
	for (ndInt32 i = 0; i < maxSteps; ++i)
	{
		//m_master->m_trajectoryAccumulator.PushBack(m_trajectory[i]);
		ndInt32 index = trajectoryAccumulator.GetCount();
		trajectoryAccumulator.SetCount(index + 1);
		trajectoryAccumulator.SetAdvantage(index, ndBrainFloat(0.0f));
		trajectoryAccumulator.SetReward(index, m_trajectory.GetReward(i));
		trajectoryAccumulator.SetAction(index, m_trajectory.GetAction(i));
		ndMemCpy(trajectoryAccumulator.GetObservations(index), m_trajectory.GetObservations(i), m_master->m_numberOfObservations);
	}
	m_trajectory.SetCount(0);
}

// ***************************************************************************************
//
// ***************************************************************************************
ndBrainAgentDiscretePolicyGradient_TrainerMaster::ndBrainAgentDiscretePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainThreadPool()
	,m_policy()
	,m_value()
	,m_optimizer(nullptr)
	,m_trainers()
	,m_weightedTrainer()
	,m_auxiliaryTrainers()
	,m_baseLineValueOptimizer(nullptr)
	,m_baseLineValueTrainers()
	,m_stateValues(hyperParameters.m_numberOfObservations)
	,m_randomPermutation()
	,m_trajectoryAccumulator(hyperParameters.m_numberOfObservations)
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
	,m_bashTrajectorySteps(hyperParameters.m_bashTrajectoryCount* m_maxTrajectorySteps)
	,m_memoryStateIndex(0)
	,m_memoryStateIndexFull(0)
	,m_baseValueWorkingBufferSize(0)
	,m_randomSeed(hyperParameters.m_randomSeed)
	,m_workingBuffer()
	,m_averageScore()
	,m_averageFramesPerEpisodes()
	,m_agents()
{
	ndAssert(m_numberOfActions);
	ndAssert(m_numberOfObservations);
	
	m_randomGenerator = new ndBrainAgentDiscretePolicyGradient_Trainer::ndRandomGenerator[size_t(hyperParameters.m_bashTrajectoryCount)];
	for (ndInt32 i = 0; i < hyperParameters.m_bashTrajectoryCount; ++i)
	{
		m_randomSeed++;
		m_randomGenerator[i].m_gen.seed(m_randomSeed);
	}

	m_randomSeed = 0;
	// build policy neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	
	#define ACTIVATION_VPG_TYPE ndBrainLayerActivationTanh
	//#define ACTIVATION_VPG_TYPE ndBrainLayerActivationElu
	//#define ACTIVATION_VPG_TYPE ndBrainLayerActivationSigmoidLinear
	
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObservations, hyperParameters.m_neuronPerLayers));
	layers.PushBack(new ACTIVATION_VPG_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, hyperParameters.m_neuronPerLayers));
		layers.PushBack(new ACTIVATION_VPG_TYPE(hyperParameters.m_neuronPerLayers));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, m_numberOfActions));
	layers.PushBack(new ndBrainLayerActivationSoftmax(m_numberOfActions));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_policy.AddLayer(layers[i]);
	}
	
	m_policy.InitWeights();
	Normalize(m_policy);
	ndAssert(!strcmp((m_policy[m_policy.GetCount() - 1])->GetLabelId(), "ndBrainLayerActivationSoftmax"));
	
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
	layers.PushBack(new ACTIVATION_VPG_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers * 2);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), hyperParameters.m_neuronPerLayers * 2));
		layers.PushBack(new ACTIVATION_VPG_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_value.AddLayer(layers[i]);
	}
	m_value.InitWeights();
	Normalize(m_value);
	
	ndAssert(m_value.GetOutputSize() == 1);
	ndAssert(m_value.GetInputSize() == m_policy.GetInputSize());
	ndAssert(!strcmp((m_value[m_value.GetCount() - 1])->GetLabelId(), "ndBrainLayerLinear"));
	
	m_baseLineValueTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_value);
		m_baseLineValueTrainers.PushBack(trainer);
	}
	
	m_baseLineValueOptimizer = new ndBrainOptimizerAdam();
	m_baseLineValueOptimizer->SetRegularizer(ndBrainFloat(1.0e-4f));
	
	m_baseValueWorkingBufferSize = m_value.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
}

ndBrainAgentDiscretePolicyGradient_TrainerMaster::~ndBrainAgentDiscretePolicyGradient_TrainerMaster()
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

ndBrain* ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetPolicyNetwork()
{
	return &m_policy;
}

ndBrain* ndBrainAgentDiscretePolicyGradient_TrainerMaster::GetValueNetwork()
{
	return &m_value;
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::Normalize(ndBrain& actor)
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

	SupervisedTrainer optimizer(&actor);
	optimizer.Optimize();
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
	m_randomSeed = (m_randomSeed + 1) % m_bashTrajectoryCount;
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
	return m_averageScore.GetAverage();
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::UpdateBaseLineValue()
{
	m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount());
	for (ndInt32 i = m_trajectoryAccumulator.GetCount() - 1; i >= 0; --i)
	{
		m_randomPermutation[i] = i;
	}
	m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());

	const ndInt32 start = m_memoryStateIndex;
	const ndInt32 samplesCount = ndMin(m_trajectoryAccumulator.GetCount() / 5, ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE / 4);
	for (ndInt32 i = samplesCount; i >= 0; --i)
	{
		ndInt32 srcIndex = m_randomPermutation[i];
		m_stateValues.SaveTransition(m_memoryStateIndex, m_trajectoryAccumulator.GetReward(srcIndex), m_trajectoryAccumulator.GetObservations(srcIndex));
		m_memoryStateIndex = (m_memoryStateIndex + 1) % ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE;
	}

	if (!m_memoryStateIndexFull && (start < m_memoryStateIndex))
	{
		ndAtomic<ndInt32> iterator(0);
		const ndInt32 maxSteps = (m_trajectoryAccumulator.GetCount() & -m_bashBufferSize) - m_bashBufferSize;
		for (ndInt32 base = 0; base < maxSteps; base += m_bashBufferSize)
		{
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainLossLeastSquaredError loss(1);
				ndBrainFixSizeVector<1> stateValue;
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_baseLineValueTrainers[i];
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_numberOfObservations);
					stateValue[0] = m_trajectoryAccumulator.GetReward(index);
					loss.SetTruth(stateValue);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			ndBrainThreadPool::ParallelExecute(BackPropagateBash);
			m_baseLineValueOptimizer->Update(this, m_baseLineValueTrainers, m_criticLearnRate);
		}
	}
	else
	{
		m_memoryStateIndexFull = 1;
		m_randomPermutation.SetCount(ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE);
		for (ndInt32 i = ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE - 1; i >= 0; --i)
		{
			m_randomPermutation[i] = i;
		}
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());

		ndAtomic<ndInt32> iterator(0);
		const ndInt32 maxSteps = (ND_DISCRETE_POLICY_GRADIENT_BUFFER_SIZE & -m_bashBufferSize) - m_bashBufferSize;
		for (ndInt32 base = 0; base < maxSteps; base += m_bashBufferSize)
		{
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainLossLeastSquaredError loss(1);
				ndBrainFixSizeVector<1> stateValue;
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_baseLineValueTrainers[i];

					stateValue[0] = m_stateValues.GetReward(index);
					const ndBrainMemVector observation(m_stateValues.GetObservations(index), m_numberOfObservations);
					loss.SetTruth(stateValue);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			ndBrainThreadPool::ParallelExecute(BackPropagateBash);
			m_baseLineValueOptimizer->Update(this, m_baseLineValueTrainers, m_criticLearnRate);
		}
	}
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::OptimizeCritic()
{
	UpdateBaseLineValue();

	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount() - 1); i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator.GetReward(i);
	}
	m_averageScore.Update(averageSum / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(m_trajectoryAccumulator.GetCount()) / ndBrainFloat(m_bashTrajectoryIndex));

	ndBrainFixSizeVector<D_MAX_THREADS_COUNT> rewardVariance;

	ndAtomic<ndInt32> iterator(0);
	rewardVariance.Set(ndBrainFloat(0.0f));
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());

	auto CalculateAdvantage = ndMakeObject::ndFunction([this, &iterator, &rewardVariance](ndInt32 threadIndex, ndInt32)
	{
		ndBrainFixSizeVector<1> actions;
		ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);

		ndInt32 const count = ndInt32(m_trajectoryAccumulator.GetCount());
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_numberOfObservations);
			m_value.MakePrediction(observation, actions, workingBuffer);
			ndBrainFloat baseLine = actions[0];
			ndBrainFloat reward = m_trajectoryAccumulator.GetReward(i);
			ndBrainFloat advantage = reward - baseLine;
			m_trajectoryAccumulator.SetAdvantage(i, advantage);
			rewardVariance[threadIndex] += advantage * advantage;
		}
	});
	ndBrainThreadPool::ParallelExecute(CalculateAdvantage);

	ndBrainFloat rewardVarianceSum = ndBrainFloat(0.0f);
	for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	{
		rewardVarianceSum += rewardVariance[i];
	}

	rewardVarianceSum /= ndBrainFloat(m_trajectoryAccumulator.GetCount());
	ndBrainFloat invVariance = ndBrainFloat(1.0f) / ndBrainFloat(ndSqrt(rewardVarianceSum + ndBrainFloat(1.0e-4f)));
	ndInt32 newCount = ndInt32(m_trajectoryAccumulator.GetCount());
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount()) - 1; i >= 0; --i)
	{
		const ndBrainFloat normalizedAdvantage = m_trajectoryAccumulator.GetAdvantage(i) * invVariance;
		m_trajectoryAccumulator.SetAdvantage(i, normalizedAdvantage);
	}
	m_trajectoryAccumulator.SetCount(newCount);
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::OptimizePolicy()
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

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					//const ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectory____Step& trajectoryStep = m_agent->m_trajectoryAccumulator[m_index];
					//ndInt32 actionIndex = ndInt32(trajectoryStep.m_action);
					ndInt32 actionIndex = ndInt32(m_agent->m_trajectoryAccumulator.GetAction(m_index));

					loss.Set(ndBrainFloat(0.0f));
					ndBrainFloat negLogProb = ndBrainFloat(-ndLog(output[actionIndex]));

					//loss[actionIndex] = negLogProb * trajectoryStep.m_reward;
					loss[actionIndex] = negLogProb * m_agent->m_trajectoryAccumulator.GetReward(m_index);
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentDiscretePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_auxiliaryTrainers[i];
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

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::Optimize()
{
	OptimizeCritic();
	OptimizePolicy();
}

void ndBrainAgentDiscretePolicyGradient_TrainerMaster::OptimizeStep()
{
	for (ndList<ndBrainAgentDiscretePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentDiscretePolicyGradient_Trainer* const agent = node->GetInfo();

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

	if ((m_bashTrajectoryIndex >= (m_bashTrajectoryCount * 10)) || (m_trajectoryAccumulator.GetCount() >= m_bashTrajectorySteps))
	{
		Optimize();
		m_eposideCount++;
		m_framesAlive = 0;
		m_bashTrajectoryIndex = 0;
		m_trajectoryAccumulator.SetCount(0);
		for (ndList<ndBrainAgentDiscretePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentDiscretePolicyGradient_Trainer* const agent = node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
		}
	}
}
