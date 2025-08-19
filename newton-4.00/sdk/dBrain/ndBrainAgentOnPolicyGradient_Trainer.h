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

// this is an implementation of the different versions of the  
// on policy gradient for continue control re enforcement learning.  
// pseudo code samples of variance of the same algorithm can be found at:
// https://spinningup.openai.com/en/latest/algorithms/vpg.html
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
	ndBrainAgentOnPolicyGradient_Agent(const ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer>& master);

	virtual void Step();
	virtual void InitWeights() { ndAssert(0); }
	virtual void OptimizeStep() { ndAssert(0); }
	virtual void Save(ndBrainSave* const) { ndAssert(0); }
	virtual bool IsTrainer() const { ndAssert(0); return true; }
	virtual ndInt32 GetEpisodeFrames() const;

	void SampleActions(ndBrainVector& action);

	ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer> m_owner;
	ndTrajectory m_trajectory;
	ndRandomGenerator m_randomeGenerator;
	ndUnsigned32 m_trajectoryBaseIndex;
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
		ndBrainFloat m_policyRegularizer;
		ndBrainFloat m_criticRegularizer;
		ndBrainFloat m_polyakBlendFactor;
		ndBrainFloat m_discountRewardFactor;

		ndBrainFloat m_entropyTemperature;
		ndBrainFloat m_entropyMaxTemperature;
		ndBrainFloat m_entropyMinTemperature;
		ndInt32	m_entropyFrameStar;
		ndInt32	m_entropyFrameEnd;

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

	void SaveState(const char* baseName);
	void RecoverState(const char* baseName);

	private:
	void Optimize();
	void CalculateScore();
	void SaveTrajectory();
	void CacheTrajectoryTransitions();
	
	void TrainPolicy();
	void BuildPolicyClass();
	void BuildCriticClass();
	void SaveTrajectoryTerminal();
	void SaveTrajectoryNoTerminal();
	void CalculateExpectedRewards();
	void TrainCritics(ndInt32 criticIndex);

	public:
	ndString m_name;
	HyperParameters m_parameters;
	ndSharedPtr<ndBrainContext> m_context;
	ndSharedPtr<ndBrainTrainer> m_policyTrainer;
	ndSharedPtr<ndBrainTrainer> m_criticTrainer[2];
	ndSharedPtr<ndBrainTrainerInference> m_referenceCriticTrainer[2];

	//ndBrainAgentOnPolicyGradient_Agent* m_agent;
	ndList<ndBrainAgentOnPolicyGradient_Agent*> m_agents;
	std::mt19937 m_randomGenerator;
	std::uniform_real_distribution<ndFloat32> m_uniformDistribution;

	ndSharedPtr<ndBrainFloatBuffer> m_uniformRandom;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchMean;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchSigma;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchEntropy;
	ndSharedPtr<ndBrainFloatBuffer> m_replayBufferFlat;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchNoTerminal;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchOfTransitions;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchExpectedRewards;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchCriticInputTest;
	ndSharedPtr<ndBrainFloatBuffer> m_minibatchUniformRandomDistribution;

	ndSharedPtr<ndBrainIntegerBuffer> m_randomShuffleBuffer;
	ndSharedPtr<ndBrainIntegerBuffer> m_minibatchIndexBuffer;

	ndBrainVector m_lastPolicy;
	ndBrainVector m_scratchBuffer;

	ndArray<ndInt32> m_shuffleBuffer;
	ndArray<ndInt32> m_miniBatchIndices;
	ndMovingAverage<ND_ON_POLICY_MOVING_AVERAGE_SCORE> m_averageExpectedRewards;
	ndMovingAverage<ND_ON_POLICY_MOVING_AVERAGE_SCORE> m_averageFramesPerEpisodes;

	ndUnsigned32 m_frameCount;
	ndUnsigned32 m_horizonSteps;
	ndUnsigned32 m_eposideCount;
	ndUnsigned32 m_replayBufferIndex;
	ndUnsigned32 m_shuffleBatchIndex;
	
	bool m_replayIsFilled;
	bool m_startOptimization;

	friend class ndBrainAgentOnPolicyGradient_Agent;
};

#endif 
