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

#ifndef _ND_BRAIN_AGENT_OFF_POLICY_TRAINER_H__
#define _ND_BRAIN_AGENT_OFF_POLICY_TRAINER_H__

#include "ndBrainStdafx.h"

#include "ndBrain.h"
#include "ndBrainAgent.h"

// this is an implementation of the vanilla deep deterministic 
// policy gradient for continues control re enforcement learning.  
// ddpg algorithm as described in: https://arxiv.org/pdf/1509.02971.pdf
// pseudo code sample:
// https://spinningup.openai.com/en/latest/algorithms/ddpg.html
// https://spinningup.openai.com/en/latest/algorithms/td3.html
// https://spinningup.openai.com/en/latest/algorithms/sac.html

#define ND_OFF_POLICY_ACTIVATION_NAME		"ndBrainAgentOffPolicyGradient"
#define ND_OFF_POLICY_MOVING_AVERAGE_SCORE	16

class ndBrainFloatBuffer;
class ndBrainIntegerBuffer;
class ndBrainUniformBuffer;
class ndBrainAgentOffPolicyGradient_Trainer;

class ndBrainAgentOffPolicyGradient_Agent: public ndBrainAgent
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
	ndBrainAgentOffPolicyGradient_Agent(const ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master);
	~ndBrainAgentOffPolicyGradient_Agent();

	virtual void Step();
	virtual void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	virtual void Save(ndBrainSave* const) { ndAssert(0); }
	virtual bool IsTrainer() const { ndAssert(0); return true; }
	virtual ndInt32 GetEpisodeFrames() const;

	void SampleActions(ndBrainVector& action);

	ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer> m_owner;
	ndTrajectory m_trajectory;
	ndRandomGenerator m_randomeGenerator;
	ndUnsigned32 m_trajectoryBaseIndex;
	friend class ndBrainAgentOffPolicyGradient_Trainer;
};

class ndBrainAgentOffPolicyGradient_Trainer : public ndClassAlloc
{
	public:
	class ndActivation;
	class HyperParameters
	{
		public:
		HyperParameters();

		ndBrainFloat m_policyLearnRate;
		ndBrainFloat m_criticLearnRate;
		ndBrainFloat m_policyRegularizer;
		ndBrainFloat m_criticRegularizer;
		ndBrainFloat mm_polyakBlendFactor;
		ndBrainFloat m_discountRewardFactor;

		ndBrainFloat m_actionFixSigma;
		ndBrainFloat m_entropyRegularizerCoef;

		ndUnsigned32 m_randomSeed;
		ndInt32 m_miniBatchSize;
		ndInt32 m_numberOfActions;
		ndInt32 m_numberOfObservations;

		ndInt32 m_numberOfHiddenLayers;
		ndInt32 m_hiddenLayersNumberOfNeurons;

		ndInt32 m_numberOfUpdates;
		ndInt32 m_replayBufferSize;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_replayBufferStartOptimizeSize;

		bool m_useGpuBackend;
		bool m_useSofActorCritic;
		bool m_usePerActionSigmas;
		
		ndRegularizerType m_policyRegularizerType;
		ndRegularizerType m_criticRegularizerType;
	};

	ndBrainAgentOffPolicyGradient_Trainer(const HyperParameters& parameters);
	virtual ~ndBrainAgentOffPolicyGradient_Trainer();

	const ndString& GetName() const;
	void SetName(const ndString& name);

	ndBrain* GetPolicyNetwork();

	void OptimizeStep();

	bool IsSampling() const;
	ndUnsigned32 GetFramesCount() const;
	ndUnsigned32 GetEposideCount() const;

	ndFloat32 GetAverageScore() const;
	ndFloat32 GetAverageFrames() const;

	static ndBrainLayer* LoadActivation(const ndBrainLoad* const loadSave);

	protected:
	void Optimize();
	void CalculateScore();
	void SaveTrajectory();
	void SaveTrajectoryLoadBuffer();
	
	void TrainSacPolicy();
	void TrainTd3Policy();
	void BuildPolicyClass();
	void BuildCriticClass();
	void SaveTrajectoryTerminal();
	void SaveTrajectoryNoTerminal();
	void CalculateSacExpectedRewards();
	void CalculateTd3ExpectedRewards();
	void TrainCritics(ndInt32 criticIndex);
	ndBrainFloat CalculatePolicyProbability(ndInt32 index) const;
	ndBrainFloat CalculatePolicyProbability(ndInt32 index, const ndBrainVector& distribution) const;

	public:
	ndString m_name;
	HyperParameters m_parameters;
	ndSharedPtr<ndBrainContext> m_context;
	ndSharedPtr<ndBrainTrainer> m_policyTrainer;
	ndSharedPtr<ndBrainTrainer> m_criticTrainer[2];
	ndSharedPtr<ndBrainTrainerInference> m_referencePolicyTrainer;
	ndSharedPtr<ndBrainTrainerInference> m_referenceCriticTrainer[2];

	ndBrainAgentOffPolicyGradient_Agent* m_agent;
	std::mt19937 m_randomGenerator;
	std::uniform_real_distribution<ndFloat32> m_uniformDistribution;

	ndSharedPtr<ndBrainFloatBuffer> m_minibatchMean;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchSigma;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchEntropy;

	ndSharedPtr<ndBrainFloatBuffer> m_uniformRandom0;
	ndSharedPtr<ndBrainFloatBuffer> m_replayBufferFlat;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchRewards;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchNoTerminal;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchOfTransitions;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchExpectedRewards;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchCriticInputTest;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchUniformRandomDistribution;

	ndSharedPtr<ndBrainIntegerBuffer> m_randomShuffleBuffer;
	ndSharedPtr<ndBrainIntegerBuffer> m_minibatchIndexBuffer;

	ndBrainVector m_scratchBuffer;

	ndArray<ndInt32> m_shuffleBuffer;
	ndArray<ndInt32> m_miniBatchIndices;
	ndMovingAverage<ND_OFF_POLICY_MOVING_AVERAGE_SCORE> m_averageExpectedRewards;
	ndMovingAverage<ND_OFF_POLICY_MOVING_AVERAGE_SCORE> m_averageFramesPerEpisodes;

	ndUnsigned32 m_frameCount;
	ndUnsigned32 m_eposideCount;
	ndUnsigned32 m_policyDelayMod;
	ndUnsigned32 m_replayBufferIndex;
	ndUnsigned32 m_shuffleBatchIndex;
	bool m_replayIsFilled;
	bool m_startOptimization;

	friend class ndBrainAgentOffPolicyGradient_Agent;
};

#endif 
