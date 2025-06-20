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

#ifndef _ND_BRAIN_AGENT_DESCRETE_POLICY_GRADIENT_TRAINER_H__
#define _ND_BRAIN_AGENT_DESCRETE_POLICY_GRADIENT_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainThreadPool.h"
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationSigmoidLinear.h"

// this is an implementation of the vanilla policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/vpg.html
// This is an implementation of the proxima policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/ppo.html
// is is an impropment that allows multiple passes on the same data collection as 
// a long as the two disrtribution are close.

//#define ND_DISCRETE_PROXIMA_POLICY_GRADIENT

class ndBrainOptimizerAdamLegacy;
class ndBrainAgentDiscretePolicyGradient_TrainerMaster;

class ndBrainAgentDiscretePolicyGradient_Trainer : public ndBrainAgent
{
	public:
	class ndTrajectoryTransition : protected ndBrainVector
	{
		public:
		ndTrajectoryTransition(ndInt32 obsevationsSize, ndInt32 actionsSize);

		ndInt32 GetCount() const;
		void SetCount(ndInt32 count);

		ndBrainFloat GetAction(ndInt32 entry) const;
		void SetAction(ndInt32 entry, ndBrainFloat actionIndex);

		ndBrainFloat GetReward(ndInt32 entry) const;
		void SetReward(ndInt32 entry, ndBrainFloat reward);

		bool GetTerminalState(ndInt32 entry) const;
		void SetTerminalState(ndInt32 entry, bool isTernimal);

		ndBrainFloat GetAdvantage(ndInt32 entry) const;
		void SetAdvantage(ndInt32 entry, ndBrainFloat advantage);

		void Clear(ndInt32 entry);

		ndBrainFloat* GetObservations(ndInt32 entry);
		const ndBrainFloat* GetObservations(ndInt32 entry) const;

		ndBrainFloat* GetProbabilityDistribution(ndInt32 entry);
		const ndBrainFloat* GetProbabilityDistribution(ndInt32 entry) const;

		ndInt32 m_actionbsSize;
		ndInt32 m_obsevationsSize;
	};

	class ndRandomGenerator
	{
		public:
		ndRandomGenerator()
			:m_gen()
			,m_rd()
			,m_d(ndFloat32(0.0f), ndFloat32(1.0f))
		{
		}

		std::mt19937 m_gen;
		std::random_device m_rd;
		std::uniform_real_distribution<ndFloat32> m_d;
	};

	ndBrainAgentDiscretePolicyGradient_Trainer(const ndSharedPtr<ndBrainAgentDiscretePolicyGradient_TrainerMaster>& master);
	~ndBrainAgentDiscretePolicyGradient_Trainer();

	ndBrain* GetActor();
	ndBrainFloat SampleActions(const ndBrainVector& probabilities) const;

	void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	void Save(ndBrainSave* const) { ndAssert(0); }
	bool IsTrainer() const { ndAssert(0); return true; }
	ndInt32 GetEpisodeFrames() const;
	void InitWeights(ndBrainFloat, ndBrainFloat) { ndAssert(0); }

	virtual void Step();
	virtual void SaveTrajectory();
	virtual bool IsTerminal() const;

	ndBrainVector m_workingBuffer;
	ndTrajectoryTransition m_trajectory;
	ndSharedPtr<ndBrainAgentDiscretePolicyGradient_TrainerMaster> m_master;
	ndRandomGenerator* m_randomGenerator;

	friend class ndBrainAgentDiscretePolicyGradient_TrainerMaster;
};

class ndBrainAgentDiscretePolicyGradient_TrainerMaster : public ndBrainThreadPool
{
	public:
	class HyperParameters
	{
		public:
		HyperParameters();

		ndBrainFloat m_policyLearnRate;
		ndBrainFloat m_criticLearnRate;
		ndBrainFloat m_regularizer;
		ndBrainFloat m_discountRewardFactor;

		ndInt32 m_threadsCount;
		ndInt32 m_numberOfActions;
		ndInt32 m_numberOfObservations;

		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_miniBatchSize;
		ndInt32 m_hiddenLayersNumberOfNeurons;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_batchTrajectoryCount;
		ndInt32 m_extraTrajectorySteps;
		ndUnsigned32 m_randomSeed;
	};

	class MemoryStateValues : protected ndBrainVector
	{
		public:
		MemoryStateValues(ndInt32 obsevationsSize);
		ndBrainFloat GetReward(ndInt32 index) const;
		const ndBrainFloat* GetObservations(ndInt32 index) const;
		void SaveTransition(ndInt32 index, ndBrainFloat reward, const ndBrainFloat* const observations);

		ndInt32 m_obsevationsSize;
	};

	ndBrainAgentDiscretePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters);
	virtual ~ndBrainAgentDiscretePolicyGradient_TrainerMaster();

	ndBrain* GetValueNetwork();
	ndBrain* GetPolicyNetwork();

	const ndString& GetName() const;
	void SetName(const ndString& name);

	ndUnsigned32 GetFramesCount() const;
	ndUnsigned32 GetEposideCount() const;

	bool IsSampling() const;
	ndFloat32 GetAverageScore() const;
	ndFloat32 GetAverageFrames() const;

	void OptimizeStep();

	private:
	void Optimize();
	void OptimizePolicy();
	void OptimizeCritic();
	void UpdateBaseLineValue();
	void Normalize(ndBrain& actor);
	ndBrainAgentDiscretePolicyGradient_Trainer::ndRandomGenerator* GetRandomGenerator();

#ifdef ND_DISCRETE_PROXIMA_POLICY_GRADIENT
	void OptimizePolicyPPOstep();
	ndBrainFloat CalculateKLdivergence();
#endif

	ndBrain m_policy;
	ndBrain m_critic;
	
	ndArray<ndBrainTrainer*> m_criticTrainers;
	ndArray<ndBrainTrainer*> m_policyTrainers;
	ndArray<ndBrainTrainer*> m_policyWeightedTrainer;
	ndArray<ndBrainTrainer*> m_policyAuxiliaryTrainers;

	//ndBrainOptimizerAdamLegacy* m_criticOptimizer;
	//ndBrainOptimizerAdamLegacy* m_policyOptimizer;

	ndArray<ndInt32> m_randomPermutation;
	ndBrainAgentDiscretePolicyGradient_Trainer::ndRandomGenerator* m_randomGenerator;
	ndBrainAgentDiscretePolicyGradient_Trainer::ndTrajectoryTransition m_trajectoryAccumulator;

	ndBrainFloat m_gamma;
	//ndBrainFloat m_policyLearnRate;
	//ndBrainFloat m_criticLearnRate;
	ndInt32 m_numberOfActions;
	ndInt32 m_numberOfObservations;
	ndInt32 m_framesAlive;
	ndUnsigned32 m_frameCount;
	ndUnsigned32 m_eposideCount;
	//ndInt32 m_miniBatchSize;
	ndInt32 m_maxTrajectorySteps;
	ndInt32 m_extraTrajectorySteps;
	ndInt32 m_batchTrajectoryIndex;
	ndInt32 m_batchTrajectoryCount;
	ndInt32 m_batchTrajectorySteps;
	ndInt32 m_baseValueWorkingBufferSize;
	ndUnsigned32 m_randomSeed;
	ndBrainVector m_workingBuffer;
	ndMovingAverage<8> m_averageExpectedRewards;
	ndMovingAverage<8> m_averageFramesPerEpisodes;
	ndString m_name;
	ndList<ndBrainAgentDiscretePolicyGradient_Trainer*> m_agents;
	friend class ndBrainAgentDiscretePolicyGradient_Trainer;
};

#endif 