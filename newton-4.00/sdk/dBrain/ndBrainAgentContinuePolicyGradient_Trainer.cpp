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
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerActivationElu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationSigmoidLinear.h"
#include "ndBrainAgentContinuePolicyGradient_Trainer.h"

#define ND_CONTINUE_POLICY_MIN_SIGMA	ndBrainFloat(0.01f)

ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::ndTrajectoryStep(ndInt32 startIndex)
	:m_reward(ndBrainFloat(0.0f))
	,m_advantage(ndBrainFloat(0.0f))
	,m_bufferStart(startIndex)
{
}

ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::ndTrajectoryStep(const ndTrajectoryStep& src)
	:m_reward(src.m_reward)
	,m_advantage(src.m_advantage)
	,m_bufferStart(src.m_bufferStart)
{
}

ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep& ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep::operator=(const ndTrajectoryStep& src)
{
	new (this) ndTrajectoryStep(src);
	return*this;
}

// *************************************************************************************
// 
// *************************************************************************************
ndBrainAgentContinuePolicyGradient_Trainer::ndBrainAgentContinuePolicyGradient_Trainer(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
	:ndBrainAgent()
	,m_workingBuffer()
	,m_trajectory()
	,m_master(master)
	,m_rd()
	,m_gen(m_rd())
	,m_d(ndFloat32(0.0f), ndFloat32(1.0f))
{
	m_gen.seed(m_master->m_randomSeed);
	m_master->m_randomSeed += 1;

	//std::mt19937 m_gen0(m_rd());
	//std::mt19937 m_gen1(m_rd());
	//m_gen0.seed(m_master->m_randomSeed);
	//m_gen1.seed(m_master->m_randomSeed);
	//std::uniform_real_distribution<ndFloat32> uniform0(ndFloat32(0.0f), ndFloat32(1.0f));
	//std::uniform_real_distribution<ndFloat32> uniform1(ndFloat32(0.0f), ndFloat32(1.0f));
	//ndFloat32 xxx0 = uniform0(m_gen0);
	//ndFloat32 xxx1 = uniform1(m_gen1);

	m_master->m_agents.Append(this);
	m_trajectory.SetCount(m_master->m_maxTrajectorySteps + m_master->m_extraTrajectorySteps);
	m_trajectory.SetCount(0);
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
	return m_master->GetActor();
}

bool ndBrainAgentContinuePolicyGradient_Trainer::IsTerminal() const
{
	return false;
}

void ndBrainAgentContinuePolicyGradient_Trainer::SelectAction(ndBrainVector& actions) const
{
	const ndInt32 numberOfActions = m_master->m_numberOfActions;
	ndAssert(numberOfActions * 2 == actions.GetCount());
	for (ndInt32 i = numberOfActions - 1; i >= 0; --i)
	{
		ndBrainFloat sample = ndBrainFloat(actions[i] + m_d(m_gen) * actions[i + numberOfActions]);
		ndBrainFloat squashedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = squashedAction;
	}
}

void ndBrainAgentContinuePolicyGradient_Trainer::SaveTrajectory()
{
	if (m_trajectory.GetCount())
	{
		m_master->m_bashTrajectoryIndex++;
		// remove last step because if it was a dead state, it will provide misleading feedback.
		m_trajectory.SetCount(m_trajectory.GetCount() - 1);

		// prune trajectory last steps with no rewards
		//for (ndInt32 i = ndInt32(m_trajectory.GetCount()) - 1; i >= 0; --i)
		//{
		//	if (m_trajectory[i].m_reward > ndBrainFloat(1.0e-3f))
		//	{
		//		m_trajectory.SetCount(ndMin (i + 2, ndInt32 (m_trajectory.GetCount())));
		//		break;
		//	}
		//}

		// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
		for (ndInt32 i = ndInt32(m_trajectory.GetCount()) - 2; i >= 0; --i)
		{
			m_trajectory[i].m_reward += m_master->m_gamma * m_trajectory[i + 1].m_reward;
		}

		// get the max trajectory steps
		const ndInt32 maxSteps = ndMin(ndInt32(m_trajectory.GetCount()), m_master->m_maxTrajectorySteps);
		ndAssert(maxSteps > 0);
		ndInt32 bufferStartIndex = ndInt32(m_master->m_trajectoryAccumulatorBuffer.GetCount());
		ndInt32 bufferDataSize = m_master->m_numberOfActions * 2 + m_master->m_numberOfObsevations;
		m_master->m_trajectoryAccumulatorBuffer.SetCount(bufferStartIndex + bufferDataSize * maxSteps);
		ndMemCpy(&m_master->m_trajectoryAccumulatorBuffer[bufferStartIndex], &m_trajectoryBuffer[0], bufferDataSize * maxSteps);
		for (ndInt32 i = 0; i < maxSteps; ++i)
		{
			m_trajectory[i].m_bufferStart = bufferStartIndex + i * bufferDataSize;
			m_master->m_trajectoryAccumulator.PushBack(m_trajectory[i]);
		}
	}

	m_trajectory.SetCount(0);
	m_trajectoryBuffer.SetCount(0);
}

void ndBrainAgentContinuePolicyGradient_Trainer::Step()
{
	ndInt32 startIndex = ndInt32 (m_trajectoryBuffer.GetCount());

	ndTrajectoryStep trajectoryStep(startIndex);
	m_trajectoryBuffer.SetCount(m_trajectoryBuffer.GetCount() + m_master->m_numberOfObsevations + m_master->m_numberOfActions * 2);

	ndBrainMemVector actions (&m_trajectoryBuffer[startIndex + m_master->m_numberOfObsevations], m_master->m_numberOfActions * 2);
	ndBrainMemVector observations (&m_trajectoryBuffer[startIndex], m_master->m_numberOfObsevations);
	
	GetObservation(&observations[0]);
	m_master->m_actor.MakePrediction(observations, actions, m_workingBuffer);
	SelectAction(actions);
	ApplyActions(&actions[0]);
	trajectoryStep.m_reward = CalculateReward();
	
	ndAssert(m_trajectory.GetCount() < m_trajectory.GetCapacity());
	m_trajectory.PushBack(trajectoryStep);
}


//****************************************************************************************
//
//****************************************************************************************

class ndBrainAgentContinuePolicyGradient_TrainerMaster::LastActivationLayer : public ndBrainLayerActivationTanh
{
	public:
	LastActivationLayer(ndInt32 neurons)
		:ndBrainLayerActivationTanh(neurons * 2)
	{
	}

	LastActivationLayer(const LastActivationLayer& src)
		:ndBrainLayerActivationTanh(src)
	{
	}

	ndBrainLayer* Clone() const
	{
		return new LastActivationLayer(*this);
	}

	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
	{
		ndBrainLayerActivationTanh::MakePrediction(input, output);
		for (ndInt32 i = m_neurons / 2 - 1; i >= 0; --i)
		{
			output[i + m_neurons / 2] = ndMax(input[i + m_neurons / 2], ND_CONTINUE_POLICY_MIN_SIGMA);
		}
	}

	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
	{
		ndBrainLayerActivationTanh::InputDerivative(input, output, outputDerivative, inputDerivative);
		for (ndInt32 i = m_neurons / 2 - 1; i >= 0; --i)
		{
			inputDerivative[i + m_neurons / 2] = (input[i + m_neurons / 2] > ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
			inputDerivative[i + m_neurons / 2] *= outputDerivative[i + m_neurons / 2];
		}
	}
};

ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters::HyperParameters()
{
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_randomSeed = 47;
	m_numberOfLayers = 4;
	m_bashBufferSize = 64;
	m_neuronPerLayers = 64;
	m_bashTrajectoryCount = 100;
	m_maxTrajectorySteps = 4096;
	m_extraTrajectorySteps = 1024;
	m_baseLineOptimizationPases = 4;

	//m_criticLearnRate = ndBrainFloat(0.0005f);
	//m_policyLearnRate = ndBrainFloat(0.0002f);
	m_criticLearnRate = ndBrainFloat(0.0001f);
	m_policyLearnRate = ndBrainFloat(0.0001f);

	m_regularizer = ndBrainFloat(1.0e-6f);
	m_discountFactor = ndBrainFloat(0.99f);
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_bashBufferSize);
	//m_threadsCount = 1;
}

ndBrainAgentContinuePolicyGradient_TrainerMaster::ndBrainAgentContinuePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters)
	:ndBrainThreadPool()
	,m_actor()
	,m_baseLineValue()
	,m_optimizer(nullptr)
	,m_trainers()
	,m_weightedTrainer()
	,m_auxiliaryTrainers()
	,m_baseLineValueOptimizer(nullptr)
	,m_baseLineValueTrainers()
	,m_randomPermutation()
	,m_trajectoryAccumulatorBuffer()
	,m_trajectoryAccumulator()
	,m_gamma(hyperParameters.m_discountFactor)
	,m_policyLearnRate(hyperParameters.m_policyLearnRate)
	,m_criticLearnRate(hyperParameters.m_criticLearnRate)
	,m_numberOfActions(hyperParameters.m_numberOfActions)
	,m_numberOfObsevations(hyperParameters.m_numberOfObservations)
	,m_frameCount(0)
	,m_framesAlive(0)
	,m_eposideCount(0)
	,m_bashBufferSize(hyperParameters.m_bashBufferSize)
	,m_maxTrajectorySteps(hyperParameters.m_maxTrajectorySteps)
	,m_extraTrajectorySteps(hyperParameters.m_extraTrajectorySteps)
	,m_bashTrajectoryIndex(0)
	,m_bashTrajectoryCount(hyperParameters.m_bashTrajectoryCount)
	,m_bashTrajectorySteps(hyperParameters.m_bashTrajectoryCount* m_maxTrajectorySteps)
	,m_baseLineOptimizationPases(hyperParameters.m_baseLineOptimizationPases)
	,m_baseValueWorkingBufferSize(0)
	,m_randomSeed(hyperParameters.m_randomSeed)
	,m_workingBuffer()
	,m_averageScore()
	,m_averageFramesPerEpisodes()
	,m_agents()
{
	ndAssert(m_numberOfActions);
	ndAssert(m_numberOfObsevations);

	ndSetRandSeed(m_randomSeed);
	
	// build policy neural net
	SetThreadCount(hyperParameters.m_threadsCount);
	ndFixSizeArray<ndBrainLayer*, 32> layers;
	
	#define ACTIVATION_VPG_TYPE ndBrainLayerActivationTanh
	//#define ACTIVATION_VPG_TYPE ndBrainLayerActivationElu
	//#define ACTIVATION_VPG_TYPE ndBrainLayerActivationSigmoidLinear
	
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObsevations, hyperParameters.m_neuronPerLayers));
	layers.PushBack(new ACTIVATION_VPG_TYPE(layers[layers.GetCount() - 1]->GetOutputSize()));
	for (ndInt32 i = 1; i < hyperParameters.m_numberOfLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == hyperParameters.m_neuronPerLayers);
		layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, hyperParameters.m_neuronPerLayers));
		layers.PushBack(new ACTIVATION_VPG_TYPE(hyperParameters.m_neuronPerLayers));
	}
	layers.PushBack(new ndBrainLayerLinear(hyperParameters.m_neuronPerLayers, m_numberOfActions * 2));
	layers.PushBack(new LastActivationLayer(m_numberOfActions));
	
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		m_actor.AddLayer(layers[i]);
	}
	
	m_actor.InitWeights();
	ndAssert(!strcmp((m_actor[m_actor.GetCount() - 1])->GetLabelId(), "ndBrainLayerActivationTanh"));
	
	m_trainers.SetCount(0);
	m_auxiliaryTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_actor);
		m_trainers.PushBack(trainer);
	
		ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(&m_actor);
		m_auxiliaryTrainers.PushBack(auxiliaryTrainer);
	}
	
	m_weightedTrainer.PushBack(m_trainers[0]);
	m_optimizer = new ndBrainOptimizerAdam();
	m_optimizer->SetRegularizer(hyperParameters.m_regularizer);
	
	// build state value critic neural net
	layers.SetCount(0);
	layers.PushBack(new ndBrainLayerLinear(m_numberOfObsevations, hyperParameters.m_neuronPerLayers * 2));
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
		m_baseLineValue.AddLayer(layers[i]);
	}
	m_baseLineValue.InitWeights();
	
	ndAssert(m_baseLineValue.GetOutputSize() == 1);
	ndAssert(m_baseLineValue.GetInputSize() == m_actor.GetInputSize());
	ndAssert(!strcmp((m_baseLineValue[m_baseLineValue.GetCount() - 1])->GetLabelId(), "ndBrainLayerLinear"));
	
	m_baseLineValueTrainers.SetCount(0);
	for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	{
		ndBrainTrainer* const trainer = new ndBrainTrainer(&m_baseLineValue);
		m_baseLineValueTrainers.PushBack(trainer);
	}
	
	m_baseLineValueOptimizer = new ndBrainOptimizerAdam();
	//m_baseLineValueOptimizer->SetRegularizer(hyperParameters.m_regularizer);
	m_baseLineValueOptimizer->SetRegularizer(ndBrainFloat(1.0e-4f));
	
	m_baseValueWorkingBufferSize = m_baseLineValue.CalculateWorkingBufferSize();
	m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * hyperParameters.m_threadsCount);
	
	//m_actor.SaveToFile("xxxx1.xxx");
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
}

ndBrain* ndBrainAgentContinuePolicyGradient_TrainerMaster::GetActor()
{
	return &m_actor;
}

ndBrain* ndBrainAgentContinuePolicyGradient_TrainerMaster::GetCritic()
{
	return &m_baseLineValue;
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::SetName(const ndString& name)
{
	m_name = name;
}

const ndString& ndBrainAgentContinuePolicyGradient_TrainerMaster::GetName() const
{
	return m_name;
}

bool ndBrainAgentContinuePolicyGradient_TrainerMaster::IsSampling() const
{
	return false;
}

ndInt32 ndBrainAgentContinuePolicyGradient_TrainerMaster::GetFramesCount() const
{
	return m_frameCount;
}

ndInt32 ndBrainAgentContinuePolicyGradient_TrainerMaster::GetEposideCount() const
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

void ndBrainAgentContinuePolicyGradient_TrainerMaster::Optimize()
{
	OptimizeCritic();
	OptimizePolicy();
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
		m_trajectoryAccumulatorBuffer.SetCount(0);
		for (ndList<ndBrainAgentContinuePolicyGradient_Trainer*>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentContinuePolicyGradient_Trainer* const agent = node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->m_trajectoryBuffer.SetCount(0);
		}
	}
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::UpdateBaseLineValue()
{
	m_randomPermutation.SetCount(m_trajectoryAccumulator.GetCount());
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount()) - 1; i >= 0; --i)
	{
		m_randomPermutation[i] = i;
	}

	ndAtomic<ndInt32> iterator(0);
	const ndInt32 maxSteps = (m_trajectoryAccumulator.GetCount() & -m_bashBufferSize) - m_bashBufferSize;
	for (ndInt32 passes = 0; passes < m_baseLineOptimizationPases; ++passes)
	{
		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
		for (ndInt32 base = 0; base < maxSteps; base += m_bashBufferSize)
		{
			auto BackPropagateBash = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				class ndPolicyLoss : public	ndBrainLossLeastSquaredError
				{
					public:
					ndPolicyLoss()
						:ndBrainLossLeastSquaredError(1)
					{
					}

					void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
					{
						ndBrainLossLeastSquaredError::GetLoss(output, loss);
					}
				};

				ndPolicyLoss loss;
				ndBrainFixSizeVector<1> stateValue;
				for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_baseLineValueTrainers[i];
					const ndInt32 bufferIndex = m_trajectoryAccumulator[i].m_bufferStart;
					const ndBrainMemVector observation(&m_trajectoryAccumulatorBuffer[bufferIndex], m_numberOfObsevations);
					stateValue[0] = m_trajectoryAccumulator[index].m_reward;
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

void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeCritic()
{
	UpdateBaseLineValue();

	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount() - 1); i >= 0; --i)
	{
		averageSum += m_trajectoryAccumulator[i].m_reward;
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
			const ndInt32 bufferIndex = m_trajectoryAccumulator[i].m_bufferStart;
			const ndBrainMemVector observation(&m_trajectoryAccumulatorBuffer[bufferIndex], m_numberOfObsevations);
			m_baseLineValue.MakePrediction(observation, actions, workingBuffer);
			ndBrainFloat baseLine = actions[0];
			ndBrainFloat reward = m_trajectoryAccumulator[i].m_reward;
			ndBrainFloat advantage = reward - baseLine;
			m_trajectoryAccumulator[i].m_advantage = advantage;
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
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount()) - 1; i >= 0; --i)
	{
		const ndBrainFloat normalizedAdvantage = m_trajectoryAccumulator[i].m_advantage * invVariance;
		m_trajectoryAccumulator[i].m_advantage = normalizedAdvantage;
	}
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
			class TrainedSigmaLoss : public ndBrainLoss
			{
				public:
				TrainedSigmaLoss(ndBrainTrainer& trainer, ndBrainAgentContinuePolicyGradient_TrainerMaster* const agent, ndInt32 index)
					:ndBrainLoss()
					,m_trainer(trainer)
					,m_agent(agent)
					,m_index(index)
				{
				}

				void GetLoss(const ndBrainVector& output, ndBrainVector& loss)
				{
					// basically this fits a multivariate Gaussian process with zero cross covariance to the actions.
					const ndBrainFloat* const actions = &m_agent->m_trajectoryAccumulatorBuffer[m_agent->m_numberOfObsevations];
					const ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep& trajectoryStep = m_agent->m_trajectoryAccumulator[m_index];
					const ndBrainFloat advantage = trajectoryStep.m_advantage;
					const ndInt32 numOfAcctions = m_agent->m_numberOfActions;
					for (ndInt32 i = numOfAcctions - 1; i >= 0; --i)
					{
						const ndBrainFloat mean = output[i];
						const ndBrainFloat sigma1 = output[i + numOfAcctions];
						const ndBrainFloat sigma2 = sigma1 * sigma1;
						const ndBrainFloat sigma3 = sigma2 * sigma1;
						const ndInt32 bufferIndex = trajectoryStep.m_bufferStart;
						const ndBrainFloat num = (actions[bufferIndex] - mean);
						ndAssert(sigma1 > ndFloat32(0.0f));

						loss[i] = advantage * num / sigma2;
						loss[i + numOfAcctions] = advantage * (num * num / sigma3 - ndBrainFloat(1.0f) / sigma1);
					}
				}

				ndBrainTrainer& m_trainer;
				ndBrainAgentContinuePolicyGradient_TrainerMaster* m_agent;
				ndInt32 m_index;
			};

			//ndBrainFixSizeVector<statesDim> observations;
			ndInt32 bufferStride = m_numberOfObsevations + m_numberOfActions * 2;
			for (ndInt32 i = iterator++; i < m_bashBufferSize; i = iterator++)
			{
				ndBrainTrainer& trainer = *m_auxiliaryTrainers[i];
				TrainedSigmaLoss loss(trainer, this, base + i);
				if ((base + i) < m_trajectoryAccumulator.GetCount())
				{
					const ndBrainMemVector observation(&m_trajectoryAccumulatorBuffer[bufferStride * (base + i)], m_numberOfObsevations);
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
