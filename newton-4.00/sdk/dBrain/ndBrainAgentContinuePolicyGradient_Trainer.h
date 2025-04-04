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
#include "ndBrainThreadPool.h"
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationSigmoidLinear.h"

// this is an implementation of the vanilla policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/vpg.html

class ndBrainOptimizerAdam;
class ndBrainAgentContinuePolicyGradient_TrainerMaster;


class ndPolicyGradientActivation : public ndBrainLayerActivation
{
	public:
	ndPolicyGradientActivation(ndInt32 neurons);
	ndPolicyGradientActivation(const ndPolicyGradientActivation& src);
	ndBrainLayer* Clone() const;

	virtual const char* GetLabelId() const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);
	virtual void Save(const ndBrainSave* const loadSave) const override;

	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;
};


class ndBrainAgentContinuePolicyGradient_Trainer : public ndBrainAgent
{
	class ndTrajectoryStep : protected ndBrainVector
	{
		public:
		ndTrajectoryStep(ndInt32 actionsSize, ndInt32 obsevationsSize);

		ndInt32 GetCount() const;
		void SetCount(ndInt32 count);

		ndBrainFloat GetReward(ndInt32 entry) const;
		void SetReward(ndInt32 entry, ndBrainFloat reward);

		bool GetTerminalState(ndInt32 entry) const;
		void SetTerminalState(ndInt32 entry, bool isTernimal);

		ndBrainFloat GetAdvantage(ndInt32 entry) const;
		void SetAdvantage(ndInt32 entry, ndBrainFloat advantage);

		void Clear(ndInt32 entry);
		ndBrainFloat* GetActions(ndInt32 entry);
		const ndBrainFloat* GetActions(ndInt32 entry) const;

		ndBrainFloat* GetObservations(ndInt32 entry);
		const ndBrainFloat* GetObservations(ndInt32 entry) const;

		ndBrainFloat* GetProbabilityDistribution(ndInt32 entry);
		const ndBrainFloat* GetProbabilityDistribution(ndInt32 entry) const;

		ndInt64 m_actionsSize;
		ndInt64 m_obsevationsSize;
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
		std::normal_distribution<ndFloat32> m_d;
	};

	public:
	ndBrainAgentContinuePolicyGradient_Trainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master);
	~ndBrainAgentContinuePolicyGradient_Trainer();

	ndBrain* GetActor();
	void SelectAction(ndBrainVector& actions) const;

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
	ndTrajectoryStep m_trajectory;
	ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster> m_master;
	ndRandomGenerator* m_randomGenerator;

	friend class ndBrainAgentContinuePolicyGradient_TrainerMaster;
};

class ndBrainAgentContinuePolicyGradient_TrainerMaster : public ndBrainThreadPool
{
	public:
	class HyperParameters
	{
		public:
		HyperParameters();

		ndBrainFloat m_policyLearnRate;
		ndBrainFloat m_criticLearnRate;
		ndBrainFloat m_regularizer;
		ndBrainFloat m_discountFactor;

		ndInt32 m_threadsCount;
		ndInt32 m_numberOfActions;
		ndInt32 m_numberOfObservations;

		ndInt32 m_numberOfLayers;
		ndInt32 m_bashBufferSize;
		ndInt32 m_neuronPerLayers;
		ndInt32 m_maxTrajectorySteps;
		ndInt32 m_bashTrajectoryCount;
		ndInt32 m_extraTrajectorySteps;
		ndUnsigned32 m_randomSeed;
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
	virtual void Optimize();
	void OptimizePolicy();
	void OptimizeCritic();
	void NormalizePolicy();
	void NormalizeCritic();
	void UpdateBaseLineValue();
	ndBrainAgentContinuePolicyGradient_Trainer::ndRandomGenerator* GetRandomGenerator();
	ndBrain m_policy;
	ndBrain m_critic;

	ndArray<ndBrainTrainer*> m_criticTrainers;
	ndArray<ndBrainTrainer*> m_policyTrainers;
	ndArray<ndBrainTrainer*> m_policyWeightedTrainer;
	ndArray<ndBrainTrainer*> m_policyAuxiliaryTrainers;

	ndBrainOptimizerAdam* m_criticOptimizer;
	ndBrainOptimizerAdam* m_policyOptimizer;

	ndArray<ndInt32> m_randomPermutation;
	ndBrainAgentContinuePolicyGradient_Trainer::ndRandomGenerator* m_randomGenerator;
	ndBrainAgentContinuePolicyGradient_Trainer::ndTrajectoryStep m_trajectoryAccumulator;
	
	ndBrainFloat m_gamma;
	ndBrainFloat m_policyLearnRate;
	ndBrainFloat m_criticLearnRate;
	ndBrainFloat m_policyLearnRateStop;
	ndBrainFloat m_criticLearnRateStop;
	ndBrainFloat m_policyLearnRateAnnealing;
	ndBrainFloat m_criticLearnRateAnnealing;

	ndInt32 m_numberOfActions;
	ndInt32 m_numberOfObservations;
	ndInt32 m_framesAlive;
	ndUnsigned32 m_frameCount;
	ndUnsigned32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_maxTrajectorySteps;
	ndInt32 m_extraTrajectorySteps;
	ndInt32 m_bashTrajectoryIndex;
	ndInt32 m_bashTrajectoryCount;
	ndInt32 m_bashTrajectorySteps;
	ndInt32 m_baseValueWorkingBufferSize;
	ndUnsigned32 m_randomSeed;
	ndBrainVector m_workingBuffer;
	ndMovingAverage<8> m_averageScore;
	ndMovingAverage<8> m_averageFramesPerEpisodes;
	ndString m_name;
	ndList<ndBrainAgentContinuePolicyGradient_Trainer*> m_agents;
	friend class ndBrainAgentContinuePolicyGradient_Trainer;
};

#endif 