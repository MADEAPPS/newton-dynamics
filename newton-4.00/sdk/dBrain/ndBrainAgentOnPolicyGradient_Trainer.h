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

#ifndef _ND_BRAIN_AGENT_ON_POLICY_TRAINER_H__
#define _ND_BRAIN_AGENT_ON_POLICY_TRAINER_H__

#include "ndBrainStdafx.h"

#include "ndBrain.h"
#include "ndBrainAgent.h"

// This is an implementation the Proximal Policy optimization as decrived in.
// https://arxiv.org/abs/1707.06347
// The algorithm is stochastic, and use the generalized advantage statimation.
// it optinally supports entropy regularization to scale the loss 
// of both actor and critic networks. 
// pseudo code implementation is found here
// https://spinningup.openai.com/en/latest/algorithms/ppo.html

#define ND_ON_POLICY_MOVING_AVERAGE_SCORE	8

class ndBrainFloatBuffer;
class ndBrainIntegerBuffer;
class ndBrainUniformBuffer;
class ndBrainAgentOnPolicyGradient_Trainer;

class ndBrainAgentOnPolicyGradient_Agent: public ndBrainAgent
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
	class ndTrajectory
	{
		public:
		ndTrajectory();
		ndTrajectory(ndInt32 actionsSize, ndInt32 obsevationsSize);

		void Init(ndInt32 actionsSize, ndInt32 obsevationsSize);
	
		ndInt32 GetCount() const;
		void SetCount(ndInt32 count);

		void Clear(ndInt32 entry);
		void CopyFrom(ndInt32 entry, ndTrajectory& src, ndInt32 srcEntry);
	
		ndBrainFloat GetReward(ndInt32 entry) const;
		void SetReward(ndInt32 entry, ndBrainFloat reward);

		bool GetTerminalState(ndInt32 entry) const;
		void SetTerminalState(ndInt32 entry, bool isTermimal);

		ndBrainFloat* GetActions(ndInt32 entry);
		const ndBrainFloat* GetActions(ndInt32 entry) const;

		ndBrainFloat* GetObservations(ndInt32 entry);
		const ndBrainFloat* GetObservations(ndInt32 entry) const;

		ndBrainFloat* GetNextObservations(ndInt32 entry);
		const ndBrainFloat* GetNextObservations(ndInt32 entry) const;

		// for GPU 
		ndInt32 GetStride() const;
		ndInt32 GetRewardOffset() const;
		ndInt32 GetActionOffset() const;
		ndInt32 GetTerminalOffset() const;
		ndInt32 GetObsevationOffset() const;
		ndInt32 GetNextObsevationOffset() const;
		void GetFlatArray(ndInt32 index, ndBrainVector& output) const;

		ndBrainVector m_reward;
		ndBrainVector m_terminal;
		ndBrainVector m_actions;
		ndBrainVector m_observations;
		ndBrainVector m_nextObservations;
		ndInt32 m_actionsSize;
		ndInt32 m_obsevationsSize;
	};

	public:
	ndBrainAgentOnPolicyGradient_Agent(ndBrainAgentOnPolicyGradient_Trainer* const master);

	virtual void Step();
	virtual void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	virtual void Save(ndBrainSave* const) { ndAssert(0); }
	virtual bool IsTrainer() const { ndAssert(0); return true; }
	virtual ndInt32 GetEpisodeFrames() const;
	virtual void SampleActions(ndBrainVector& action);
	
	ndTrajectory m_trajectory;
	ndRandomGenerator m_randomGenerator;
	ndBrainAgentOnPolicyGradient_Trainer* m_owner;

	bool m_isDead;
	friend class ndBrainAgentOnPolicyGradient_Trainer;
};

class ndBrainAgentOnPolicyGradient_Trainer : public ndClassAlloc
{
	public:
	class HyperParameters
	{
		public:
		HyperParameters();

		ndBrainFloat m_learnRate;
		ndBrainFloat m_minSigmaSquared;
		ndBrainFloat m_maxSigmaSquared;
		ndBrainFloat m_policyRegularizer;
		ndBrainFloat m_criticRegularizer;
		ndBrainFloat m_discountRewardFactor;
		ndBrainFloat m_entropyRegularizerCoef;

		ndUnsigned32 m_randomSeed;
		ndInt32 m_miniBatchSize;
		ndInt32 m_numberOfActions;
		ndInt32 m_numberOfObservations;

		ndInt32 m_criticValueIterations;
		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_hiddenLayersNumberOfNeurons;

		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_batchTrajectoryCount;

		bool m_useGpuBackend;
		
		ndRegularizerType m_policyRegularizerType;
		ndRegularizerType m_criticRegularizerType;
	};

	ndBrainAgentOnPolicyGradient_Trainer(const HyperParameters& parameters);

	const ndString& GetName() const;
	void SetName(const ndString& name);

	ndBrain* GetPolicyNetwork();

	void OptimizeStep();

	bool IsValid() const;
	bool IsSampling() const;
	ndUnsigned32 GetFramesCount() const;
	ndUnsigned32 GetEposideCount() const;

	ndFloat32 GetAverageScore() const;
	ndFloat32 GetAverageFrames() const;

	void AddAgent(ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>& agent);

	private:
	void Optimize();
	void UpdateScore();
	void OptimizeCritic();
	void OptimizePolicy();
	void BuildPolicyClass();
	void BuildCriticClass();
	void CalculateAdvantage();
	void TrajectoryToGpuBuffers();
	void SaveTrajectory(ndBrainAgentOnPolicyGradient_Agent* const agent);

	public:
	ndString m_name;
	HyperParameters m_parameters;
	ndSharedPtr<ndBrainContext> m_context;
	ndSharedPtr<ndBrainTrainer> m_policyTrainer;
	ndSharedPtr<ndBrainTrainer> m_criticTrainer;

	std::mt19937 m_randomGenerator;
	std::uniform_real_distribution<ndFloat32> m_uniformDistribution;
	ndList<ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>> m_agents;

	ndSharedPtr<ndBrainFloatBuffer> m_meanBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_zMeanBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_sigmaBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_invSigmaBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_invSigma2Buffer;
	
	ndSharedPtr<ndBrainFloatBuffer> m_meanGradiendBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_sigmaGradiendBuffer;

	ndSharedPtr<ndBrainFloatBuffer> m_criticStateValue;

	ndSharedPtr<ndBrainFloatBuffer> m_trainingBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_advantageBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_likelihoodBuffer;
	ndSharedPtr<ndBrainIntegerBuffer> m_randomShuffleBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_policyGradientAccumulator;

	ndSharedPtr<ndBrainFloatBuffer> m_advantageMinibatchBuffer;
	ndSharedPtr<ndBrainIntegerBuffer> m_randomShuffleMinibatchBuffer;

	ndBrainVector m_lastPolicy;
	ndBrainVector m_scratchBuffer;
	ndArray<ndInt32> m_shuffleBuffer;
	ndArray<ndInt32> m_tmpShuffleBuffer;
	ndBrainAgentOnPolicyGradient_Agent::ndTrajectory m_trajectoryAccumulator;
	ndMovingAverage<ND_ON_POLICY_MOVING_AVERAGE_SCORE> m_averageExpectedRewards;
	ndMovingAverage<ND_ON_POLICY_MOVING_AVERAGE_SCORE> m_averageFramesPerEpisodes;

	ndBrainFloat m_learnRate;
	ndUnsigned32 m_frameCount;
	ndUnsigned32 m_horizonSteps;
	ndUnsigned32 m_eposideCount;
	ndUnsigned32 m_trajectiesCount;
	ndUnsigned32 m_numberOfGpuTransitions;

	friend class ndBrainAgentOnPolicyGradient_Agent;
};

#endif 
