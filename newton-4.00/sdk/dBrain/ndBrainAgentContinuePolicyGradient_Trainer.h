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

#ifndef _ND_AGENT_CONTINUE_POLICY_GRADIENT_TRAINER_H__
#define _ND_AGENT_CONTINUE_POLICY_GRADIENT_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainOptimizer.h"
#include "ndBrainThreadPool.h"
#include "ndBrainLossLeastSquaredError.h"

// this is an implementation of the vanilla policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/vpg.html
// https://spinningup.openai.com/en/latest/algorithms/ppo.html

class ndBrainAgentContinuePolicyGradient_TrainerMaster;

class ndBrainAgentContinuePolicyGradient_Agent : public ndBrainAgent
{
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
		std::normal_distribution<ndReal> m_d;
	};

	public:
	class ndTrajectoryTransition
	{
		public:
		ndTrajectoryTransition();
		void Init(ndInt32 actionsSize, ndInt32 obsevationsSize);

		ndInt32 GetCount() const;
		void SetCount(ndInt32 count);

		void Clear(ndInt32 entry);
		void CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry);

		ndBrainFloat GetReward(ndInt32 entry) const;
		void SetReward(ndInt32 entry, ndBrainFloat reward);

		ndBrainFloat GetExpectedReward(ndInt32 entry) const;
		void SetExpectedReward(ndInt32 entry, ndBrainFloat reward);

		bool GetTerminalState(ndInt32 entry) const;
		void SetTerminalState(ndInt32 entry, bool isTernimal);

		ndBrainFloat* GetActions(ndInt32 entry);
		const ndBrainFloat* GetActions(ndInt32 entry) const;

		ndBrainFloat* GetObservations(ndInt32 entry);
		const ndBrainFloat* GetObservations(ndInt32 entry) const;

		ndBrainFloat* GetNextObservations(ndInt32 entry);
		const ndBrainFloat* GetNextObservations(ndInt32 entry) const;

		ndBrainVector m_reward;
		ndBrainVector m_terminal;
		ndBrainVector m_actions;
		ndBrainVector m_observations;
		ndBrainVector m_expectedReward;
		ndBrainVector m_nextObservations;

		ndInt64 m_actionsSize;
		ndInt64 m_obsevationsSize;
	};

	ndBrainAgentContinuePolicyGradient_Agent(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master);
	~ndBrainAgentContinuePolicyGradient_Agent();

	ndBrain* GetActor();
	void SampleActions(ndBrainVector& actions) const;

	void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	void Save(ndBrainSave* const) { ndAssert(0); }
	bool IsTrainer() const { ndAssert(0); return true; }
	void InitWeights(ndBrainFloat, ndBrainFloat) { ndAssert(0); }

	virtual void Step();
	virtual void SaveTrajectory();
	virtual bool IsTerminal() const;
	ndInt32 GetEpisodeFrames() const;

	ndBrainVector m_workingBuffer;
	ndTrajectoryTransition m_trajectory;
	ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster> m_owner;
	ndRandomGenerator* m_randomGenerator;
	bool m_isDead;

	friend class ndBrainAgentContinuePolicyGradient_TrainerMaster;
};

class ndBrainAgentContinuePolicyGradient_TrainerMaster: public ndClassAlloc 
{
	public:
	class HyperParameters
	{
		public:
		HyperParameters();

		ndBrainFloat m_policyLearnRate;
		ndBrainFloat m_criticLearnRate;
		ndBrainFloat m_policyRegularizer;
		ndBrainFloat m_criticRegularizer;
		ndBrainFloat m_discountRewardFactor;
		ndBrainFloat m_generalizedAdvangeDiscount;

		ndBrainFloat m_actionFixSigma;
		ndBrainFloat m_entropyTemperature;

		ndInt32 m_threadsCount;
		ndInt32 m_numberOfActions;
		ndInt32 m_numberOfObservations;

		ndInt32 m_miniBatchSize;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_batchTrajectoryCount;

		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_criticVelueIterations;
		ndInt32 m_hiddenLayersNumberOfNeurons;
		ndUnsigned32 m_randomSeed;

		bool m_usePerActionSigmas;
		ndRegularizerType m_policyRegularizerType;
		ndRegularizerType m_criticRegularizerType;
	};

	class MemoryStateValues: protected ndBrainVector
	{
		public:
		MemoryStateValues(ndInt32 obsevationsSize);
		ndBrainFloat GetReward(ndInt32 index) const;
		const ndBrainFloat* GetObservations(ndInt32 index) const;
		void SaveTransition(ndInt32 index, ndBrainFloat reward, const ndBrainFloat* const observations);

		ndInt32 m_obsevationsSize;
	};

	ndBrainAgentContinuePolicyGradient_TrainerMaster(const HyperParameters& hyperParameters);
	virtual ~ndBrainAgentContinuePolicyGradient_TrainerMaster();

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

	protected:
	void BuildPolicyClass();
	void BuildCriticClass();
	virtual void Optimize();
	virtual void OptimizePolicy();
	virtual void OptimizeCritic();
	virtual void CalculateAdvange();
	void OptimizedSurrogatePolicy();
	ndBrainFloat CalculateKLdivergence();
	ndBrainAgentContinuePolicyGradient_Agent::ndRandomGenerator* GetRandomGenerator();
	ndBrainFloat CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution) const;

	public:
	ndSharedPtr<ndBrain> m_policy;
	ndSharedPtr<ndBrain> m_critic;
	HyperParameters m_parameters;
	ndArray<ndBrainTrainer*> m_criticTrainers;
	ndArray<ndBrainTrainer*> m_policyTrainers;
	ndArray<ndBrainTrainer*> m_policyAuxiliaryTrainers;

	//ndSharedPtr<ndBrainOptimizerAdamLegacy> m_criticOptimizer;
	//ndSharedPtr<ndBrainOptimizerAdamLegacy> m_policyOptimizer;

	ndBrainVector m_advantage;
	ndArray<ndInt32> m_randomPermutation;
	ndList<ndBrainAgentContinuePolicyGradient_Agent::ndRandomGenerator> m_randomGenerator;
	ndBrainAgentContinuePolicyGradient_Agent::ndTrajectoryTransition m_trajectoryAccumulator;
	
	ndInt32 m_framesAlive;
	ndUnsigned32 m_frameCount;
	ndUnsigned32 m_eposideCount;
	ndInt32 m_extraTrajectorySteps;
	ndInt32 m_batchTrajectoryIndex;
	ndInt32 m_baseValueWorkingBufferSize;
	ndUnsigned32 m_randomSeed;
	ndBrainVector m_workingBuffer;
	ndMovingAverage<8> m_averageExpectedRewards;
	ndMovingAverage<8> m_averageFramesPerEpisodes;
	ndString m_name;

	ndBrainVector m_policyActions;
	ndBrainVector m_policyDivergeActions;
	ndBrainVector m_referenceProbability;
	ndList<ndBrainAgentContinuePolicyGradient_Agent*> m_agents;
	friend class ndBrainAgentContinuePolicyGradient_Agent;
};

#endif 