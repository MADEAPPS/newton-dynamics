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

#ifndef _ND_BRAIN_AGENT_DDPG_TRAINER_H__
#define _ND_BRAIN_AGENT_DDPG_TRAINER_H__

#include "ndBrainStdafx.h"

#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainThreadPool.h"
//include "ndBrainLayer.h"

//include "ndBrainTrainer.h"
//include "ndBrainReplayBuffer.h"
//include "ndBrainLayerLinear.h"
//include "ndBrainLayerActivationTanh.h"
//include "ndBrainLossLeastSquaredError.h"

// this is an implementation of the vanilla deep deterministic 
// policy gradient for continues control re enforcement learning.  
// ddpg algorithm as described in: https://arxiv.org/pdf/1509.02971.pdf
// https://spinningup.openai.com/en/latest/algorithms/ddpg.html

class ndBrainOptimizerAdam;
class ndBrainAgentDDPG_Trainer;

class ndBrainAgentDDPG_Agent: public ndBrainAgent
{
	//class ndRandomGenerator
	//{
	//	public:
	//	ndRandomGenerator()
	//		:m_gen()
	//		,m_rd()
	//		,m_d(ndFloat32(0.0f), ndFloat32(1.0f))
	//	{
	//	}
	//
	//	std::mt19937 m_gen;
	//	std::random_device m_rd;
	//	std::normal_distribution<ndFloat32> m_d;
	//};

	public:
	class ndTrajectoryTransition : protected ndBrainVector
	{
		public:
		enum ndMemberIndex
		{
			m_reward,
			m_isterminalState,
			m_transitionSize
		};
	
		ndTrajectoryTransition(ndInt32 actionsSize, ndInt32 obsevationsSize);
	
		ndInt32 GetCount() const;
		void SetCount(ndInt32 count);
	
		void Clear(ndInt32 entry);
		void CopyFrom(ndInt32 entry, ndTrajectoryTransition& src, ndInt32 srcEntry);
	
		ndBrainFloat GetReward(ndInt32 entry) const;
		void SetReward(ndInt32 entry, ndBrainFloat reward);

		bool GetTerminalState(ndInt32 entry) const;
		void SetTerminalState(ndInt32 entry, bool isTernimal);

		ndBrainFloat* GetActions(ndInt32 entry);
		const ndBrainFloat* GetActions(ndInt32 entry) const;

		ndBrainFloat* GetObservations(ndInt32 entry);
		const ndBrainFloat* GetObservations(ndInt32 entry) const;

		ndBrainFloat* GetNextObservations(ndInt32 entry);
		const ndBrainFloat* GetNextObservations(ndInt32 entry) const;

		ndInt32 m_actionsSize;
		ndInt32 m_obsevationsSize;
	};

	public:
	ndBrainAgentDDPG_Agent(const ndSharedPtr<ndBrainAgentDDPG_Trainer>& master);
	~ndBrainAgentDDPG_Agent();

	virtual void Step();
	virtual void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	virtual void Save(ndBrainSave* const) { ndAssert(0); }
	virtual bool IsTrainer() const { ndAssert(0); return true; }
	virtual ndInt32 GetEpisodeFrames() const;

	ndReal PerturbeAction(ndReal action) const;

	ndSharedPtr<ndBrainAgentDDPG_Trainer> m_owner;
	ndTrajectoryTransition m_trajectory;
	ndBrainVector m_workingBuffer;
	ndInt32 m_trajectoryBaseCount;
	friend class ndBrainAgentDDPG_Trainer;
};

class ndBrainAgentDDPG_Trainer : public ndBrainThreadPool
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
		ndBrainFloat m_policyMovingAverageFactor;
		ndBrainFloat m_criticMovingAverageFactor;

		ndInt32 m_miniBatchSize;
		ndInt32 m_numberOfActions;
		ndInt32 m_numberOfObservations;

		ndInt32 m_actorHiddenLayers;
		ndInt32 m_hiddenLayersNumberOfNeurons;

		ndInt32 m_replayBufferSize;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_replayBufferStartOptimizeSize;
		
		ndInt32 m_threadsCount;
		ndInt32 m_criticUpdatesCount;
		ndInt32 m_policyUpdatesCount;

		ndBrainOptimizer::ndRegularizerType m_policyRegularizerType;
		ndBrainOptimizer::ndRegularizerType m_criticRegularizerType;
	};

	ndBrainAgentDDPG_Trainer(const HyperParameters& parameters);
	~ndBrainAgentDDPG_Trainer();

	const ndString& GetName() const;
	void SetName(const ndString& name);

	ndBrain* GetPolicyNetwork();

	void OptimizeStep();

	bool IsSampling() const;
	ndUnsigned32 GetFramesCount() const;
	ndUnsigned32 GetEposideCount() const;

	ndFloat32 GetAverageScore() const;
	ndFloat32 GetAverageFrames() const;


	private:
	void Optimize();
	void SaveTrajectory();
	void BuildPolicyClass();
	void BuildCriticClass();
	void LearnPolicyFunction();
	void CalculateExpectedRewards();
	void LearnQvalueFunction(ndInt32 criticIndex);

	public:
	HyperParameters m_parameters;
	ndBrain m_policy;
	ndBrain m_critic[2];
	ndBrain m_referencePolicy;
	ndBrain m_referenceCritic[2];
	ndString m_name;
	ndBrainVector m_expectedRewards;
	ndArray<ndInt32> m_expectedRewardsIndexBuffer;
	ndBrainAgentDDPG_Agent::ndTrajectoryTransition m_replayBuffer;
	ndBrainAgentDDPG_Agent* m_agent;
	ndSharedPtr<ndBrainOptimizerAdam> m_policyOptimizer;
	ndSharedPtr<ndBrainOptimizerAdam> m_criticOptimizer[2];
	ndArray<ndBrainTrainer*> m_policyTrainers;
	ndArray<ndBrainTrainer*> m_criticTrainers[2];
	ndArray<ndInt32> m_shuffleBuffer;
	ndMovingAverage<8> m_averageScore;
	ndMovingAverage<32> m_averageFramesPerEpisodes;
	ndUnsigned32 m_frameCount;
	ndUnsigned32 m_framesAlive;
	ndUnsigned32 m_eposideCount;
	ndInt32 m_replayBufferIndex;
	bool m_startOptimization;
	ndUnsigned32 m_shuffleIndexBuffer;

	friend class ndBrainAgentDDPG_Agent;
};

#endif 
