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
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainAgentContinuePolicyGradient_Trainer.h"

#define ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION		ndBrainLayerActivationRelu
//#define ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationTanh
//#define ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION	ndBrainLayerActivationLeakyRelu

#define ND_CONTINUE_POLICY_FIX_SIGMA					ndBrainFloat(0.2f)
#define ND_CONTINUE_POLICY_MIN_PER_ACTION_SIGMA			ndBrainFloat(0.01f)
#define ND_CONTINUE_POLICY_MAX_PER_ACTION_SIGMA			ndBrainFloat(1.0f)

#define ND_CONTINUE_PROXIMA_POLICY_ITERATIONS			10
#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON			ndBrainFloat(0.2f)
#define ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE		ndBrainFloat(0.001f)
#define ND_CONTINUE_PROXIMA_POLICY_ENTROPY_CONFICIENT	ndBrainFloat (2.0e-5f)

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters::HyperParameters()
{
	m_randomSeed = 47;
	m_numberOfHiddenLayers = 3;
	m_maxTrajectorySteps = 4096;
	m_batchTrajectoryCount = 1000;
	m_hiddenLayersNumberOfNeurons = 64;

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_policyLearnRate = ndBrainFloat(1.0e-4f);
	m_criticLearnRate = m_policyLearnRate;

	m_entropyTemperature = ndBrainFloat(0.0f);
	//m_entropyTemperature = ndBrainFloat(0.25f);

	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(5.0e-3f);

	m_policyRegularizerType = m_ridge;
	m_criticRegularizerType = m_ridge;
	
	m_usePerActionSigmas = false;
	m_actionFixSigma = ND_CONTINUE_POLICY_FIX_SIGMA;

	m_criticVelueIterations = 2;
	m_discountRewardFactor = ndBrainFloat(0.99f);
	m_generalizedAdvangeDiscount = ndBrainFloat(0.99f);
	m_threadsCount = ndMin(ndBrainThreadPool::GetMaxThreads(), m_miniBatchSize);

//m_threadsCount = 1;
//m_batchTrajectoryCount = 1;
//m_usePerActionSigmas = true;
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::ndTrajectoryTransition()
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_expectedReward()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize;
	m_obsevationsSize = obsevationsSize;
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::Clear(ndInt32 entry)
{
	m_reward[entry] = ndBrainFloat(0.0f);
	m_terminal[entry] = ndBrainFloat(0.0f);
	m_expectedReward[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

ndInt32 ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetCount() const
{
	return ndInt32(m_reward.GetCount());
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetCount(ndInt32 count)
{
	m_reward.SetCount(count);
	m_terminal.SetCount(count);
	m_expectedReward.SetCount(count);
	m_actions.SetCount(count * m_actionsSize);
	m_observations.SetCount(count * m_obsevationsSize);
	m_nextObservations.SetCount(count * m_obsevationsSize);
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry)
{
	m_reward[entry] = src.m_reward[srcEntry];
	m_terminal[entry] = src.m_terminal[srcEntry];
	m_expectedReward[entry] = src.m_expectedReward[srcEntry];;
	ndMemCpy(&m_actions[entry * m_actionsSize], &src.m_actions[srcEntry * m_actionsSize], m_actionsSize);
	ndMemCpy(&m_observations[entry * m_obsevationsSize], &src.m_observations[srcEntry * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&m_nextObservations[entry * m_obsevationsSize], &src.m_nextObservations[srcEntry * m_obsevationsSize], m_obsevationsSize);
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetReward(ndInt32 entry) const
{
	return m_reward[entry];
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	m_reward[entry] = reward;
}

ndBrainFloat ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetExpectedReward(ndInt32 entry) const
{
	return m_expectedReward[entry];
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetExpectedReward(ndInt32 entry, ndBrainFloat reward)
{
	m_expectedReward[entry] = reward;
}

bool ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetTerminalState(ndInt32 entry) const
{
	return (m_terminal[entry] == 999.0f) ? true : false;
}

void ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	m_terminal[entry] = isTernimal ? ndBrainFloat(999.0f) : ndBrainFloat(-999.0f);
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry)
{
	return &m_actions[entry * m_actionsSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetActions(ndInt32 entry) const
{
	return &m_actions[entry * m_actionsSize];
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry)
{
	return &m_observations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetObservations(ndInt32 entry) const
{
	return &m_observations[entry * m_obsevationsSize];
}

ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry)
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition::GetNextObservations(ndInt32 entry) const
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

//*********************************************************************************************
//
//*********************************************************************************************
ndBrainAgentContinuePolicyGradient_TrainerMaster::MemoryStateValues::MemoryStateValues(ndInt32 obsevationsSize)
	:ndBrainVector()
	,m_obsevationsSize(obsevationsSize)
{
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
ndBrainAgentContinuePolicyGradient_Agent::ndBrainAgentContinuePolicyGradient_Agent(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& owner)
	:ndBrainAgent()
	,m_workingBuffer()
	,m_trajectory()
	,m_owner(owner)
	,m_randomGenerator(nullptr)
	,m_isDead(false)
{
	//std::mt19937 m_gen0(m_rd());
	//std::mt19937 m_gen1(m_rd());
	//m_gen0.seed(m_owner->m_randomSeed);
	//m_gen1.seed(m_owner->m_randomSeed);
	//std::uniform_real_distribution<ndFloat32> uniform0(ndFloat32(0.0f), ndFloat32(1.0f));
	//std::uniform_real_distribution<ndFloat32> uniform1(ndFloat32(0.0f), ndFloat32(1.0f));
	//ndFloat32 xxx0 = uniform0(m_gen0);
	//ndFloat32 xxx1 = uniform1(m_gen1);

	m_owner->m_agents.Append(this);
	m_trajectory.SetCount(0);
	m_randomGenerator = m_owner->GetRandomGenerator();
	m_trajectory.Init(m_owner->m_policy->GetOutputSize(), m_owner->m_policy->GetInputSize());
}

ndBrainAgentContinuePolicyGradient_Agent::~ndBrainAgentContinuePolicyGradient_Agent()
{
	for (ndList<ndBrainAgentContinuePolicyGradient_Agent*>::ndNode* node = m_owner->m_agents.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo() == this)
		{
			m_owner->m_agents.Remove(node);
			break;
		}
	}
}

ndBrain* ndBrainAgentContinuePolicyGradient_Agent::GetActor()
{ 
	return m_owner->GetPolicyNetwork(); 
}

bool ndBrainAgentContinuePolicyGradient_Agent::IsTerminal() const
{
	return false;
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_Agent::SampleActions(ndBrainVector& actions) const
{
	ndRandomGenerator& generator = *m_randomGenerator;
	if (m_owner->m_parameters.m_usePerActionSigmas)
	{
		const ndInt32 size = ndInt32(actions.GetCount()) / 2;
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			ndBrainFloat sigma = actions[size + i];
			ndBrainFloat unitVarianceSample = generator.m_d(generator.m_gen);
			ndBrainFloat sample = ndBrainFloat(actions[i]) + unitVarianceSample * sigma;
			ndBrainFloat clippedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
			actions[i] = clippedAction;
		}
	}
	else
	{
		ndBrainFloat sigma = m_owner->m_parameters.m_actionFixSigma;
		const ndInt32 size = ndInt32(actions.GetCount());
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			ndBrainFloat unitVarianceSample = generator.m_d(generator.m_gen);
			ndBrainFloat sample = ndBrainFloat(actions[i]) + unitVarianceSample * sigma;
			ndBrainFloat clippedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
			actions[i] = clippedAction;
		}
	}
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrain& policy = **m_owner->m_policy;
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
		m_owner->m_batchTrajectoryIndex++;

		for (ndInt32 i = 1; i < m_trajectory.GetCount(); ++i)
		{
			ndMemCpy(m_trajectory.GetNextObservations(i - 1), m_trajectory.GetObservations(i), m_owner->m_parameters.m_numberOfObservations);
		}

		// using the Bellman equation to calculate trajectory rewards. (Monte Carlo method)
		ndBrainFloat gamma = m_owner->m_parameters.m_discountRewardFactor;
		ndBrainFloat expectedRewrad = m_trajectory.GetReward(m_trajectory.GetCount() - 1);
		m_trajectory.SetExpectedReward(m_trajectory.GetCount() - 1, expectedRewrad);
		for (ndInt32 i = m_trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndBrainFloat r0 = m_trajectory.GetReward(i);
			expectedRewrad = r0 + gamma * expectedRewrad;
			m_trajectory.SetExpectedReward(i, expectedRewrad);
		}

		const ndInt32 maxSteps = ndMin(m_trajectory.GetCount(), m_owner->m_parameters.m_maxTrajectorySteps);
		ndAssert(maxSteps > 0);
		m_trajectory.SetTerminalState(maxSteps - 1, true);

		ndTrajectoryTransition& trajectoryAccumulator = m_owner->m_trajectoryAccumulator;
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
	:ndClassAlloc()
	,m_policy()
	,m_critic()
	,m_parameters(hyperParameters)
	,m_criticTrainers()
	,m_policyTrainers()
	,m_policyAuxiliaryTrainers()
	//,m_criticOptimizer()
	//,m_policyOptimizer()
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
	,m_name()
	,m_policyActions()
	,m_policyDivergeActions()
	,m_referenceProbability()
	,m_agents()
{
	ndAssert(0);
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
	BuildPolicyClass();
	BuildCriticClass();

	ndBrainFloat unitEntropy = ndClamp(m_parameters.m_entropyTemperature, ndBrainFloat(0.0f), ndBrainFloat(1.0f));
	m_parameters.m_entropyTemperature = ND_CONTINUE_PROXIMA_POLICY_ENTROPY_CONFICIENT * unitEntropy;
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
	return *m_policy; 
}

ndBrain* ndBrainAgentContinuePolicyGradient_TrainerMaster::GetValueNetwork()
{
	return *m_critic;
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

ndBrainFloat ndBrainAgentContinuePolicyGradient_TrainerMaster::CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution) const
{
	ndBrainFloat z2 = ndBrainFloat(0.0f);
	ndBrainFloat invSigma2Det = ndBrainFloat(1.0f);
	ndBrainFloat invSqrtPi = ndBrainFloat(1.0f) / ndBrainFloat(ndSqrt(2.0f * ndPi));

	ndBrainFloat prob = 1.0f;
	if (m_parameters.m_usePerActionSigmas)
	{
		const ndInt32 size = ndInt32(distribution.GetCount()) / 2;

		const ndBrainMemVector sampledProbabilities(m_trajectoryAccumulator.GetActions(index), m_policy->GetOutputSize());
		for (ndInt32 i = size - 1; i >= 0; --i)
		{
			ndBrainFloat sigma = distribution[i + size];
			ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
			ndBrainFloat z = (sampledProbabilities[i] - distribution[i]) * invSigma;

			z2 += (z * z);
			invSigma2Det *= (invSqrtPi * invSigma);
		}
		ndBrainFloat exponent = ndBrainFloat(0.5f) * z2;
		prob = invSigma2Det * ndBrainFloat(ndExp(-exponent));
	}
	else
	{
		const ndInt32 count = ndInt32(distribution.GetCount());

		ndBrainFloat sigma = m_parameters.m_actionFixSigma;
		ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
		const ndBrainMemVector sampledProbabilities(m_trajectoryAccumulator.GetActions(index), m_policy->GetOutputSize());
		for (ndInt32 i = count - 1; i >= 0; --i)
		{
			ndBrainFloat z = (sampledProbabilities[i] - distribution[i]) * invSigma;
			z2 += z * z;
			invSigma2Det *= (invSqrtPi * invSigma);
		}
		ndBrainFloat exponent = ndBrainFloat(0.5f) * z2;
		prob = invSigma2Det * ndBrainFloat(ndExp(-exponent));
	}
	return ndMax(prob, ndBrainFloat(1.0e-4f));
}

//#pragma optimize( "", off )
ndBrainFloat ndBrainAgentContinuePolicyGradient_TrainerMaster::CalculateKLdivergence()
{
	ndAssert(0);
	return 0;
	//// https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	//// since I am using a diagonal sigma, I do not have to use Cholesky 
	//
	//ndAtomic<ndInt32> iterator(0);
	//ndFloat64 partialDivergence[256];
	//
	//m_policyDivergeActions.SetCount(m_trajectoryAccumulator.GetCount() * m_policy->GetOutputSize());
	//auto ParcialDivergence = ndMakeObject::ndFunction([this, &iterator, &partialDivergence](ndInt32 threadIndex, ndInt32)
	//{
	//	ndFloat64 totalDivergence = ndFloat32(0.0f);
	//	ndInt32 count = m_trajectoryAccumulator.GetCount();
	//
	//	ndInt32 numberOfActions = m_policy->GetOutputSize();
	//	for (ndInt32 i = iterator++; i < count; i = iterator++)
	//	{
	//		const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
	//		ndBrainMemVector crossProbabilities(&m_policyDivergeActions[i * numberOfActions], numberOfActions);
	//		m_policy->MakePrediction(observation, crossProbabilities);
	//		const ndBrainMemVector probabilities(&m_policyActions[i * numberOfActions], numberOfActions);
	//
	//		// calculate t0 = trace(inv(Sigma_q) * Sigma_p
	//		// calculate t1 = numberOfActions
	//		// calculate t2 = trans(Uq - Up) * inv(Sigma_q) * (Uq - Up)
	//		// calculate t3 = log(det(Sigma_q) /det(Sigma_p))
	//		ndFloat32 t0 = ndFloat32(0.0f);
	//		ndFloat32 t2 = ndFloat32(0.0f);
	//		ndFloat32 log_det_p = ndFloat32(0.0f);
	//		ndFloat32 log_det_q = ndFloat32(0.0f);
	//
	//		const ndInt32 size = numberOfActions / 2;
	//		for (ndInt32 j = size - 1; j >= 0; --j)
	//		{
	//			ndBrainFloat sigma_p = probabilities[size + j];
	//			ndBrainFloat sigma_q = crossProbabilities[size + j];
	//			ndBrainFloat invSigma_q = 1.0f / sigma_q;
	//
	//			log_det_p += ndLog(sigma_p);
	//			log_det_q += ndLog(sigma_q);
	//			t0 += sigma_p * invSigma_q;
	//			ndBrainFloat meanError(crossProbabilities[j] - probabilities[j]);
	//			t2 += meanError * invSigma_q * meanError;
	//		}
	//
	//		// it does not really matter  the ratio is inverted since KLis a distance, 
	//		// the only problem is that KL(p/q) is different that KL(q/p)
	//		// but the distance still represent how close are the two distributions.
	//		//ndFloat64 t3 = ndLog(det_q / det_p);
	//		ndFloat64 t3 = log_det_q - log_det_p;
	//		ndFloat64 t1 = ndBrainFloat(size);
	//		ndFloat64 divergence = t0 - t1 + t2 + t3;
	//		totalDivergence += divergence;
	//	}
	//	partialDivergence[threadIndex] = ndBrainFloat(0.5f) * totalDivergence;
	//});
	//
	//auto ParcialDivergenceFixSigma = ndMakeObject::ndFunction([this, &iterator, &partialDivergence](ndInt32 threadIndex, ndInt32)
	//{
	//	ndFloat64 totalDivergence = ndFloat32(0.0f);
	//	ndInt32 count = m_trajectoryAccumulator.GetCount();
	//
	//	ndInt32 numberOfActions = m_policy->GetOutputSize();
	//	for (ndInt32 i = iterator++; i < count; i = iterator++)
	//	{
	//		const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
	//		ndBrainMemVector crossProbabilities(&m_policyDivergeActions[i * numberOfActions], numberOfActions);
	//		m_policy->MakePrediction(observation, crossProbabilities);
	//		const ndBrainMemVector probabilities(&m_policyActions[i * numberOfActions], numberOfActions);
	//
	//		// calculate t0 = trace(inv(Sigma_q) * Sigma_p
	//		// calculate t1 = numberOfActions
	//		// calculate t2 = trans(Uq - Up) * inv(Sigma_q) * (Uq - Up)
	//		// calculate t3 = log(det(Sigma_q) /det(Sigma_p))
	//		ndFloat32 t0 = ndFloat32(0.0f);
	//		ndFloat32 t2 = ndFloat32(0.0f);
	//		ndFloat32 log_det_p = ndFloat32(0.0f);
	//		ndFloat32 log_det_q = ndFloat32(0.0f);
	//
	//		ndBrainFloat sigma_p = m_parameters.m_actionFixSigma;
	//		ndBrainFloat sigma_q = m_parameters.m_actionFixSigma;
	//		ndBrainFloat invSigma_q = 1.0f / sigma_q;
	//		ndFloat32 logSigmap = ndLog(sigma_p);
	//		ndFloat32 logSigmaq = ndLog(sigma_q);
	//
	//		const ndInt32 size = numberOfActions;
	//		for (ndInt32 j = size - 1; j >= 0; --j)
	//		{
	//			log_det_p += logSigmap;
	//			log_det_q += logSigmaq;
	//			t0 += sigma_p * invSigma_q;
	//			ndBrainFloat meanError(crossProbabilities[j] - probabilities[j]);
	//			t2 += meanError * invSigma_q * meanError;
	//		}
	//
	//		// it does not really matter  the ratio is inverted since KLis a distance, 
	//		// the only problem is that KL(p/q) is different that KL(q/p)
	//		// but the distance still represent how close are the two distributions.
	//		//ndFloat64 t3 = ndLog(det_q / det_p);
	//		ndFloat64 t3 = log_det_q - log_det_p;
	//		ndFloat64 t1 = ndBrainFloat(size);
	//		ndFloat64 divergence = t0 - t1 + t2 + t3;
	//		totalDivergence += divergence;
	//	}
	//	partialDivergence[threadIndex] = ndBrainFloat(0.5f) * totalDivergence;
	//});
	//
	//if (m_parameters.m_usePerActionSigmas)
	//{
	//	ndBrainThreadPool::ParallelExecute(ParcialDivergence);
	//}
	//else
	//{
	//	ndBrainThreadPool::ParallelExecute(ParcialDivergenceFixSigma);
	//}
	//
	//ndFloat64 divergence = ndFloat32(0.0f);
	//for (ndInt32 i = GetThreadCount() - 1; i >= 0; --i)
	//{
	//	divergence += partialDivergence[i];
	//}
	//ndAssert(divergence >= 0.0f);
	//divergence /= ndFloat64(m_trajectoryAccumulator.GetCount());
	//return ndBrainFloat(divergence);
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
	ndAssert(0);
	//ndFixSizeArray<ndBrainLayer*, 32> layers;
	//
	//layers.SetCount(0);
	//layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	//layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	//for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	//{
	//	ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
	//	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_hiddenLayersNumberOfNeurons, m_parameters.m_hiddenLayersNumberOfNeurons));
	//	layers.PushBack(new ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
	//}
	//
	//ndInt32 nunberOfOutput = m_parameters.m_usePerActionSigmas ? 2 * m_parameters.m_numberOfActions : m_parameters.m_numberOfActions;
	//layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), nunberOfOutput));
	//layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	//if (m_parameters.m_usePerActionSigmas)
	//{
	//	ndBrainFixSizeVector<256> bias;
	//	ndBrainFixSizeVector<256> slope;
	//	bias.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
	//	slope.SetCount(layers[layers.GetCount() - 1]->GetOutputSize());
	//
	//	ndInt32 sigmaSize = nunberOfOutput / 2;
	//	ndBrainFloat b = ndBrainFloat(0.5f) * (ND_CONTINUE_POLICY_MAX_PER_ACTION_SIGMA + ND_CONTINUE_POLICY_MIN_PER_ACTION_SIGMA);
	//	ndBrainFloat a = ndBrainFloat(0.5f) * (ND_CONTINUE_POLICY_MAX_PER_ACTION_SIGMA - ND_CONTINUE_POLICY_MIN_PER_ACTION_SIGMA);
	//
	//	bias.Set(ndBrainFloat(0.0f));
	//	slope.Set(ndBrainFloat(1.0f));
	//	ndMemSet(&bias[sigmaSize], b, sigmaSize);
	//	ndMemSet(&slope[sigmaSize], a, sigmaSize);
	//	layers.PushBack(new ndBrainLayerActivationLinear(slope, bias));
	//}
	//
	//m_policy = ndSharedPtr<ndBrain>(new ndBrain);
	//for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	//{
	//	m_policy->AddLayer(layers[i]);
	//}
	//m_policy->InitWeights();
	//
	////m_policy.SaveToFile("xxxx.dnn");
	////ndSharedPtr<ndBrain> xxx(ndBrainLoad::Load("xxxx.dnn"));
	//
	//m_policyTrainers.SetCount(0);
	//m_policyAuxiliaryTrainers.SetCount(0);
	//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	//{
	//	ndAssert(0);
	//	//ndBrainTrainer* const trainer = new ndBrainTrainer(m_policy);
	//	//m_policyTrainers.PushBack(trainer);
	//	//
	//	//ndBrainTrainer* const auxiliaryTrainer = new ndBrainTrainer(m_policy);
	//	//m_policyAuxiliaryTrainers.PushBack(auxiliaryTrainer);
	//}
	//
	//m_policyOptimizer = ndSharedPtr<ndBrainOptimizerAdamLegacy> (new ndBrainOptimizerAdamLegacy());
	//m_policyOptimizer->SetRegularizer(m_parameters.m_policyRegularizer);
	//m_policyOptimizer->SetRegularizerType(m_parameters.m_policyRegularizerType);
	//
	//m_trajectoryAccumulator.Init(m_policy->GetOutputSize(), m_policy->GetInputSize());
}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::BuildCriticClass()
{
	ndAssert(0);
	//ndFixSizeArray<ndBrainLayer*, 32> layers;
	//
	//// build state value critic neural net
	//layers.SetCount(0);
	//layers.PushBack(new ndBrainLayerLinear(m_policy->GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
	//layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	//
	//for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	//{
	//	ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
	//	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
	//	layers.PushBack(new ND_CONTINUE_POLICY_GRADIENT_HIDEN_LAYERS_ACTIVATION(layers[layers.GetCount() - 1]->GetOutputSize()));
	//}
	//layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	//layers.PushBack(new ndBrainLayerActivationLeakyRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	//
	//m_critic = ndSharedPtr<ndBrain>(new ndBrain);
	//for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	//{
	//	m_critic->AddLayer(layers[i]);
	//}
	//m_critic->InitWeights();
	//
	//ndAssert(m_critic->GetOutputSize() == 1);
	//ndAssert(m_critic->GetInputSize() == m_policy->GetInputSize());
	//
	//m_criticTrainers.SetCount(0);
	//for (ndInt32 i = 0; i < m_parameters.m_miniBatchSize; ++i)
	//{
	//	ndAssert(0);
	//	//ndBrainTrainer* const trainer = new ndBrainTrainer(m_critic);
	//	//m_criticTrainers.PushBack(trainer);
	//}
	//
	//m_criticOptimizer = ndSharedPtr<ndBrainOptimizerAdamLegacy> (new ndBrainOptimizerAdamLegacy());
	//m_criticOptimizer->SetRegularizer(m_parameters.m_criticRegularizer);
	//m_criticOptimizer->SetRegularizerType(m_parameters.m_criticRegularizerType);
	//
	//m_baseValueWorkingBufferSize = m_critic->CalculateWorkingBufferSize();
	//m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * m_parameters.m_threadsCount);
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::CalculateAdvange()
{
	ndAssert(0);
	//ndBrainFloat averageSum = ndBrainFloat(0.0f);
	//const ndInt32 stepNumber = m_trajectoryAccumulator.GetCount();
	//for (ndInt32 i = stepNumber - 1; i >= 0; --i)
	//{
	//	averageSum += m_trajectoryAccumulator.GetExpectedReward(i);
	//}
	//m_averageExpectedRewards.Update(averageSum / ndBrainFloat(stepNumber));
	//m_averageFramesPerEpisodes.Update(ndBrainFloat(stepNumber) / ndBrainFloat(m_parameters.m_batchTrajectoryCount));
	//
	//ndAtomic<ndInt32> iterator(0);
	//m_advantage.SetCount(m_trajectoryAccumulator.GetCount());
	//
	//m_workingBuffer.SetCount(m_baseValueWorkingBufferSize * GetThreadCount());
	//auto CalculateAdvantage = ndMakeObject::ndFunction([this, &iterator](ndInt32 threadIndex, ndInt32)
	//{
	//	// using Monte Carlos 
	//	ndBrainFixSizeVector<1> stateValue;
	//	ndBrainMemVector workingBuffer(&m_workingBuffer[threadIndex * m_baseValueWorkingBufferSize], m_baseValueWorkingBufferSize);
	//
	//	ndInt32 const count = m_trajectoryAccumulator.GetCount();
	//	for (ndInt32 i = iterator++; i < count; i = iterator++)
	//	{
	//		const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(i), m_parameters.m_numberOfObservations);
	//		m_critic->MakePrediction(observation, stateValue, workingBuffer);
	//		ndBrainFloat expectedReward = m_trajectoryAccumulator.GetExpectedReward(i);
	//		m_advantage[i] = expectedReward - stateValue[0];
	//	}
	//});
	//ndBrainThreadPool::ParallelExecute(CalculateAdvantage);
	//
	//if (m_advantage.GetCount() < m_parameters.m_miniBatchSize)
	//{
	//	ndInt32 start = 0;
	//	for (ndInt32 i = ndInt32(m_advantage.GetCount()); i < m_parameters.m_miniBatchSize; ++i)
	//	{
	//		m_trajectoryAccumulator.SetCount(i + 1);
	//		m_trajectoryAccumulator.CopyFrom(i, m_trajectoryAccumulator, start);
	//		m_advantage.PushBack(m_advantage[start]);
	//		start++;
	//	}
	//}
}	

//#pragma optimize( "", off )
//void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizePolicy()
//{
//	ndAtomic<ndInt32> iterator(0);
//	auto ClearGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
//	{
//		for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
//		{
//			ndBrainTrainer* const trainer = m_policyTrainers[i];
//			trainer->ClearGradients();
//		}
//	});
//	ndBrainThreadPool::ParallelExecute(ClearGradients);
//
//	ndAssert(m_trajectoryAccumulator.GetCount() % m_parameters.m_miniBatchSize == 0);
//
//	const ndInt32 steps = m_trajectoryAccumulator.GetCount();
//	ndBrainFloat gradientScale = ndBrainFloat(m_parameters.m_miniBatchSize) / ndBrainFloat(steps);
//	for (ndInt32 base = 0; base < steps; base += m_parameters.m_miniBatchSize)
//	{
//		auto CalculateGradients = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
//		{
//			class MaxLikelihoodLoss : public ndBrainLoss
//			{
//				public:
//				MaxLikelihoodLoss(ndBrainTrainer& trainer, ndBrainAgentContinuePolicyGradient_TrainerMaster* const owner, ndInt32 index)
//					:ndBrainLoss()
//					,m_trainer(trainer)
//					,m_owner(owner)
//					,m_index(index)
//				{
//				}
//	
//				void GetLoss(const ndBrainVector& probabilityDistribution, ndBrainVector& loss)
//				{
//					// as I understand it, this is just a special case of maximum likelihood optimization.
//					// given a multivariate Gaussian process with zero cross covariance to the actions.
//					// calculate the log of prob of a multivariate Gaussian
//	
//					const ndInt32 numberOfActions = m_owner->m_policy.GetOutputSize();
//					const ndBrainMemVector sampledProbability(m_owner->m_trajectoryAccumulator.GetActions(m_index), numberOfActions);
//					if (m_owner->m_parameters.m_usePerActionSigmas)
//					{
//						const ndInt32 size = ndInt32(m_owner->m_policy.GetOutputSize()) / 2;
//						for (ndInt32 i = size - 1; i >= 0; --i)
//						{
//							ndBrainFloat sigma = probabilityDistribution[size + i];
//							ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
//							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
//							ndBrainFloat meanGrad = z * invSigma * invSigma;
//							ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;
//
//							loss[i] = meanGrad;
//							loss[size + i] = sigmaGrad;
//						}
//					}
//					else
//					{
//						ndFloat32 sigma = m_owner->m_parameters.m_actionFixSigma;
//						ndFloat32 sigma2 = sigma * sigma;
//						ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / sigma2;
//						const ndInt32 size = ndInt32(m_owner->m_policy.GetOutputSize());
//						for (ndInt32 i = size - 1; i >= 0; --i)
//						{
//							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
//							ndBrainFloat meanGrad = z * invSigma2;
//							loss[i] = meanGrad;
//						}
//					}
//					const ndBrainFloat advantage = m_owner->m_advantage[m_index];
//	
//					//negate the gradient for gradient ascend?
//					ndBrainFloat ascend = ndBrainFloat(-1.0f) * advantage;
//					loss.Scale(ascend);
//				}
//	
//				ndBrainTrainer& m_trainer;
//				ndBrainAgentContinuePolicyGradient_TrainerMaster* m_owner;
//				ndInt32 m_index;
//			};
//	
//			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
//			{
//				ndBrainTrainer& trainer = *m_policyAuxiliaryTrainers[i];
//				ndInt32 index = base + i;
//				MaxLikelihoodLoss loss(trainer, this, index);
//				const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
//				trainer.BackPropagate(observation, loss);
//			}
//		});
//	
//		auto AddGradients = ndMakeObject::ndFunction([this, &iterator, gradientScale](ndInt32, ndInt32)
//		{
//			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
//			{
//				ndBrainTrainer* const trainer = m_policyTrainers[i];
//				ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
//				auxiliaryTrainer->ScaleWeights(gradientScale);
//				trainer->AddGradients(auxiliaryTrainer);
//			}
//		});
//	
//		iterator = 0;
//		ndBrainThreadPool::ParallelExecute(CalculateGradients);
//		iterator = 0;
//		ndBrainThreadPool::ParallelExecute(AddGradients);
//	}
//	m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
//}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizePolicy()
{
	ndAssert(0);
#if 0
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
	ndBrainFloat gradientScale = ndBrainFloat(m_parameters.m_miniBatchSize) / ndBrainFloat(steps);

	m_referenceProbability.SetCount(steps);
	m_policyActions.SetCount(m_trajectoryAccumulator.GetCount() * m_policy.GetOutputSize());

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
					// calculate the prob and save the base action for future KL iterations
					const ndInt32 numberOfActions = m_owner->m_policy.GetOutputSize();
					ndMemCpy(&m_owner->m_policyActions[m_index * numberOfActions], &probabilityDistribution[0], numberOfActions);
					m_owner->m_referenceProbability[m_index] = m_owner->CalculatePolicyProbability(m_index, probabilityDistribution);

					// as I understand it, this is just a special case of maximum likelihood optimization.
					// given a multivariate Gaussian process with zero cross covariance to the actions.

					const ndBrainMemVector sampledProbability(m_owner->m_trajectoryAccumulator.GetActions(m_index), numberOfActions);
					if (m_owner->m_parameters.m_usePerActionSigmas)
					{
						const ndInt32 size = numberOfActions / 2;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat sigma = probabilityDistribution[size + i];
							ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma * invSigma;
							ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

							loss[i] = meanGrad;
							loss[size + i] = sigmaGrad;
						}
					}
					else
					{
						ndFloat32 fixSigma = m_owner->m_parameters.m_actionFixSigma;
						ndFloat32 sigma2 = fixSigma * fixSigma;
						ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / sigma2;

						const ndInt32 size = numberOfActions;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma2;
							loss[i] = meanGrad;
						}
					}

					const ndBrainFloat advantage = m_owner->m_advantage[m_index];

					if (m_owner->m_parameters.m_entropyTemperature > ndBrainFloat(1.0e-6f))
					{
						if (m_owner->m_parameters.m_usePerActionSigmas)
						{
							// calculate and add the Gradient of entropy (grad of log probability)
							const ndInt32 size = numberOfActions / 2;
							ndBrainFloat entropyRegularizerCoef = m_owner->m_parameters.m_entropyTemperature;
							for (ndInt32 i = size - 1; i >= 0; --i)
							{
								ndBrainFloat sigma = probabilityDistribution[size + i];
								ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;;
								ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
								ndBrainFloat meanGrad = z * invSigma * invSigma;
								ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

								loss[i] -= entropyRegularizerCoef * meanGrad;
								loss[size + i] -= entropyRegularizerCoef * sigmaGrad;
							}
						}
					}

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

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator, gradientScale](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_policyTrainers[i];
				ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
				auxiliaryTrainer->ScaleWeights(gradientScale);
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}
	m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
#endif
}

//#pragma optimize( "", off )
void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizedSurrogatePolicy()
{
ndAssert(0);
#if 0
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

	const ndInt32 steps = m_trajectoryAccumulator.GetCount() & -m_parameters.m_miniBatchSize;
	ndBrainFloat gradientScale = ndBrainFloat(m_parameters.m_miniBatchSize) / ndBrainFloat(steps);
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
					// calculate clipped surrogate advantage loss.
					const ndBrainMemVector newProbabilityDistribution(&m_owner->m_policyDivergeActions[m_index * numberOfActions], numberOfActions);
					ndBrainFloat prob = m_owner->CalculatePolicyProbability(m_index, newProbabilityDistribution);
					const ndBrainFloat referenceAdvantage = m_owner->m_advantage[m_index];
					ndBrainFloat r = prob / m_owner->m_referenceProbability[m_index];
					if (referenceAdvantage >= 0.0f)
					{
						r = ndMin(r, ndBrainFloat(1.0f) + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
					}
					else if (referenceAdvantage < 0.0f)
					{
						r = ndMax(r, ndBrainFloat(1.0f) - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
					}

					// calculate grad of probability	
					const ndBrainMemVector sampledProbability(m_owner->m_trajectoryAccumulator.GetActions(m_index), numberOfActions);

					if (m_owner->m_parameters.m_usePerActionSigmas)
					{
						const ndInt32 size = numberOfActions / 2;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat sigma = probabilityDistribution[size + i];
							ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma * invSigma;
							ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

							loss[i] = meanGrad;
							loss[size + i] = sigmaGrad;
						}
					}
					else
					{
						ndFloat32 fixSigma = m_owner->m_parameters.m_actionFixSigma;
						ndFloat32 sigma2 = fixSigma * fixSigma;
						ndBrainFloat invSigma2 = ndBrainFloat(1.0f) / sigma2;
						const ndInt32 size = numberOfActions;
						for (ndInt32 i = size - 1; i >= 0; --i)
						{
							ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
							ndBrainFloat meanGrad = z * invSigma2;
							loss[i] = meanGrad;
						}
					}

					if (m_owner->m_parameters.m_entropyTemperature > ndBrainFloat(1.0e-6f))
					{
						// calculate and add the Gradient of entropy (grad of log probability)
						if (m_owner->m_parameters.m_usePerActionSigmas)
						{
							ndBrainFloat entropyRegularizerCoef = m_owner->m_parameters.m_entropyTemperature;
							const ndInt32 size = numberOfActions / 2;
							for (ndInt32 i = size - 1; i >= 0; --i)
							{
								ndBrainFloat sigma = probabilityDistribution[size + i];
								ndBrainFloat invSigma = ndBrainFloat(1.0f) / sigma;;
								ndBrainFloat z = sampledProbability[i] - probabilityDistribution[i];
								ndBrainFloat meanGrad = z * invSigma * invSigma;
								ndBrainFloat sigmaGrad = z * z * invSigma * invSigma * invSigma - sigma;

								loss[i] -= entropyRegularizerCoef * meanGrad;
								loss[size + i] -= entropyRegularizerCoef * sigmaGrad;
							}
						}
					}

					//negate the gradient for gradient ascend?
					const ndBrainFloat advantage = r * referenceAdvantage;
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

		auto AddGradients = ndMakeObject::ndFunction([this, &iterator, gradientScale](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
			{
				ndBrainTrainer* const trainer = m_policyTrainers[i];
				ndBrainTrainer* const auxiliaryTrainer = m_policyAuxiliaryTrainers[i];
				auxiliaryTrainer->ScaleWeights(gradientScale);
				trainer->AddGradients(auxiliaryTrainer);
			}
		});

		iterator = 0;
		ndBrainThreadPool::ParallelExecute(CalculateGradients);
		iterator = 0;
		ndBrainThreadPool::ParallelExecute(AddGradients);
	}
	m_policyOptimizer->Update(this, m_policyTrainers, m_parameters.m_policyLearnRate);
#endif
}

//#pragma optimize( "", off )
//void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeCritic()
//{
//	m_randomPermutation.SetCount(0);
//	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount()) - 1; i >= 0; --i)
//	{
//		m_randomPermutation.PushBack(i);
//	}
//
//	ndAtomic<ndInt32> iterator(0);
//	for (ndInt32 iter = m_parameters.m_criticVelueIterations - 1; iter >= 0; --iter)
//	{
//		m_randomPermutation.RandomShuffle(m_randomPermutation.GetCount());
//		for (ndInt32 base = 0; base < m_randomPermutation.GetCount(); base += m_parameters.m_miniBatchSize)
//		{
//			auto BackPropagateBatch = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
//			{
//				ndBrainFixSizeVector<1> stateValue;
//				ndBrainFixSizeVector<1> stateQValue;
//				ndBrainLossLeastSquaredError loss(1);
//
//				// calculate GAE(l, 1) // very, very noisy
//				// calculate GAE(l, 0) // too smooth, and does not work either
//				// but this shit does not seems any better than ramdom.
//				ndBrainFloat gamma = m_parameters.m_discountRewardFactor * m_parameters.m_generalizedAdvangeDiscount;
//				for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
//				{
//					const ndInt32 index = m_randomPermutation[base + i];
//					ndBrainTrainer& trainer = *m_criticTrainers[i];
//
//					stateValue[0] = m_trajectoryAccumulator.GetReward(index);
//					if (!m_trajectoryAccumulator.GetTerminalState(index))
//					{
//						const ndBrainMemVector nextObservation(m_trajectoryAccumulator.GetNextObservations(index), m_parameters.m_numberOfObservations);
//						m_critic.MakePrediction(nextObservation, stateQValue);
//						stateValue[0] += gamma * stateQValue[0];
//					}
//
//					loss.SetTruth(stateValue);
//					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
//					trainer.BackPropagate(observation, loss);
//				}
//			});
//
//			auto BackPropagateMontecarlo = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
//			{
//				ndBrainFixSizeVector<1> stateValue;
//				ndBrainLossLeastSquaredError loss(1);
//				for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
//				{
//					const ndInt32 index = m_randomPermutation[base + i];
//					ndBrainTrainer& trainer = *m_criticTrainers[i];
//					stateValue[0] = m_trajectoryAccumulator.GetExpectedReward(index);
//					loss.SetTruth(stateValue);
//					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
//					trainer.BackPropagate(observation, loss);
//				}
//			});
//
//			iterator = 0;
//			//ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
//			ndBrainThreadPool::ParallelExecute(BackPropagateMontecarlo);
//			m_criticOptimizer->Update(this, m_criticTrainers, m_parameters.m_criticLearnRate);
//		}
//	}
//}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::OptimizeCritic()
{
	ndAssert(0);
#if 0
	m_randomPermutation.SetCount(0);
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount()) - 1; i >= 0; --i)
	{
		m_randomPermutation.PushBack(i);
	}

	ndAtomic<ndInt32> iterator(0);
	for (ndInt32 iter = m_parameters.m_criticVelueIterations - 1; iter >= 0; --iter)
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
				ndBrainFloat gamma = m_parameters.m_discountRewardFactor * m_parameters.m_generalizedAdvangeDiscount;

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

					if (m_parameters.m_entropyTemperature > ndBrainFloat(1.0e-6f))
					{
						ndBrainFixSizeVector<256> entropyActions;
						entropyActions.SetCount(m_policy.GetOutputSize());

						const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
						m_policy.MakePrediction(observation, entropyActions);
						ndBrainFloat prob = CalculatePolicyProbability(index, entropyActions);
						ndBrainFloat logProb = ndBrainFloat(ndLog(prob));
						stateValue[0] -= m_parameters.m_entropyTemperature * logProb;
					}

					loss.SetTruth(stateValue);
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});

			auto BackPropagateMontecarlo = ndMakeObject::ndFunction([this, &iterator, base](ndInt32, ndInt32)
			{
				ndBrainLossLeastSquaredError loss(1);
				ndBrainFixSizeVector<1> stateValue;
				for (ndInt32 i = iterator++; i < m_parameters.m_miniBatchSize; i = iterator++)
				{
					const ndInt32 index = m_randomPermutation[base + i];
					ndBrainTrainer& trainer = *m_criticTrainers[i];
					stateValue[0] = m_trajectoryAccumulator.GetExpectedReward(index);

					if (m_parameters.m_entropyTemperature > ndBrainFloat(1.0e-6f))
					{
						ndBrainFixSizeVector<256> entropyActions;
						entropyActions.SetCount(m_policy.GetOutputSize());

						const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
						m_policy.MakePrediction(observation, entropyActions);
						ndBrainFloat prob = CalculatePolicyProbability(index, entropyActions);
						ndBrainFloat logProb = ndBrainFloat(ndLog(prob));
						stateValue[0] -= m_parameters.m_entropyTemperature * logProb;
					}

					loss.SetTruth(stateValue);
					const ndBrainMemVector observation(m_trajectoryAccumulator.GetObservations(index), m_parameters.m_numberOfObservations);
					trainer.BackPropagate(observation, loss);
				}
			});

			iterator = 0;
			//ndBrainThreadPool::ParallelExecute(BackPropagateBatch);
			ndBrainThreadPool::ParallelExecute(BackPropagateMontecarlo);
			m_criticOptimizer->Update(this, m_criticTrainers, m_parameters.m_criticLearnRate);
		}
	}
#endif
}

//#pragma optimize( "", off )
//void ndBrainAgentContinuePolicyGradient_TrainerMaster::Optimize()
//{
//	CalculateAdvange();
//	OptimizePolicy();
//	OptimizeCritic();
//}

void ndBrainAgentContinuePolicyGradient_TrainerMaster::Optimize()
{
	CalculateAdvange();
	OptimizePolicy();
	ndBrainFloat divergence = CalculateKLdivergence();
	for (ndInt32 i = ND_CONTINUE_PROXIMA_POLICY_ITERATIONS; (i >= 0) && (divergence < ND_CONTINUE_PROXIMA_POLICY_KL_DIVERGENCE); --i)
	{
		OptimizedSurrogatePolicy();
		divergence = CalculateKLdivergence();
	}
	OptimizeCritic();
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