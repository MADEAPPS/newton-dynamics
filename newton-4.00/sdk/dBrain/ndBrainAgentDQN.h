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

#ifndef _ND_BRAIN_AGENT_DQN_H__
#define _ND_BRAIN_AGENT_DQN_H__

#include "ndBrainStdafx.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDQN: public ndBrainAgent
{
	class ndOptimizer: public ndBrainTrainer
	{
		public:
		ndOptimizer(ndBrain* const brain)
			:ndBrainTrainer(brain)
			,m_inputBatch()
			,m_agent(nullptr)
		{
			m_inputBatch.SetCount(statesDim);
		}

		ndOptimizer(const ndOptimizer& src)
			:ndBrainTrainer(src)
			,m_inputBatch()
			,m_agent(nullptr)
		{
			m_inputBatch.SetCount(statesDim);
		}

		virtual void GetGroundTruth(ndInt32 index, ndBrainVector& groundTruth, const ndBrainVector& output) const
		{
			ndAssert(0);
			//m_agent->GetGroundTruth(index, groundTruth, output);
		}

		virtual void Optimize(ndValidation& validator, const ndBrainMatrix&, ndReal learnRate, ndInt32 steps)
		{
			ndFloatExceptions exception;
			//ndAssert(inputBatch.GetCount() == inputBatch.GetCount());

			//ndArray<ndInt32> randomizeVector;
			//ndBrain bestNetwork(*m_instance.GetBrain());
			//randomizeVector.SetCount(inputBatch.GetCount());
			//
			//ndBrainVector truth;
			m_truth.SetCount(m_output.GetCount());
			//for (ndInt32 i = 0; i < inputBatch.GetCount(); ++i)
			//{
			//	randomizeVector[i] = i;
			//}
	
			ndArray<ndInt32>& shuffleBuffer = m_agent->m_shuffleBuffer;
			const ndBrainReplayBuffer<ndInt32, statesDim, 1>& replayBuffer = m_agent->m_replayBuffer;

			shuffleBuffer.RandomShuffle(shuffleBuffer.GetCount());
			//const ndInt32 miniBatchSize = ndMin(m_miniBatchSize, inputBatch.GetCount());
			//const ndInt32 batchCount = (inputBatch.GetCount() + miniBatchSize - 1) / miniBatchSize;
			//
			////m_bestCost = validator.Validate(inputBatch, groundTruth);
			//m_bestCost = validator.Validate(inputBatch);
			//for (ndInt32 i = 0; (i < steps) && (m_bestCost > 0.0f); ++i)
			//{
			//	for (ndInt32 j = 0; j < m_agent->m_bashBufferSize; ++j)
			//	{
					ClearGradientsAcc();
					//const ndInt32 start = j * miniBatchSize;
					//const ndInt32 count = ((start + miniBatchSize) < inputBatch.GetCount()) ? miniBatchSize : inputBatch.GetCount() - start;
					//for (ndInt32 k = 0; k < count; ++k)
					for (ndInt32 i = 0; i < m_agent->m_bashBufferSize; ++i)
					{
						ndInt32 index = shuffleBuffer[i];
						//ndBrainVector& input = inputBatch[i];
						const ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& memory = replayBuffer[index];
						for (ndInt32 j = 0; j < statesDim; ++j)
						{ 
							m_inputBatch[j] = memory.m_state[j];
						}
						MakePrediction(m_inputBatch);
						//GetGroundTruth(index, truth, m_output);
						//BackPropagate(truth);
					}
					UpdateWeights(m_agent->m_learnRate, m_agent->m_bashBufferSize);
			//	}
			//	ApplyWeightTranspose();
			//	randomizeVector.RandomShuffle(randomizeVector.GetCount());
			//
			//	ndReal batchError = validator.Validate(inputBatch);
			//	if (batchError <= m_bestCost)
			//	{
			//		m_bestCost = batchError;
			//		bestNetwork.CopyFrom(*m_instance.GetBrain());
			//	}
			//}
			//m_instance.GetBrain()->CopyFrom(bestNetwork);
		}

		ndBrainVector m_inputBatch;
		ndBrainAgentDQN<statesDim, actionDim>* m_agent;
	};

	public: 
	ndBrainAgentDQN(const ndSharedPtr<ndBrain>& qValuePredictor);
	virtual ~ndBrainAgentDQN();

	void SetBufferSize(ndInt32 size);
	ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& GetTransition();

	virtual void LearnStep();

	private:
	void BackPropagate();
	ndInt32 GetAction() const;

	protected:
	//ndBrainMatrix m_inputBatch;
	ndSharedPtr<ndBrain> m_onlineNetwork;
	ndBrain m_targetNetwork;
	ndOptimizer m_trainer;
	ndBrainInstance m_targetInstance;
	ndArray<ndInt32> m_shuffleBuffer;
	ndBrainReplayBuffer<ndInt32, statesDim, 1> m_replayBuffer;
	ndBrainReplayTransitionMemory<ndInt32, statesDim, 1> m_currentTransition;
	
	ndReal m_gamma;
	ndReal m_learnRate;
	ndReal m_epsilonGreedy;
	ndReal m_epsilonGreedyStep;
	ndReal m_epsilonGreedyFloor;
	ndInt32 m_frameCount;
	ndInt32 m_eposideCount;
	ndInt32 m_bashBufferSize;
	ndInt32 m_epsilonGreedyFreq;
	ndInt32 m_targetUpdatePeriod;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN<statesDim, actionDim>::ndBrainAgentDQN(const ndSharedPtr<ndBrain>& qValuePredictor)
	:ndBrainAgent()
	,m_onlineNetwork(qValuePredictor)
	,m_targetNetwork(*(*m_onlineNetwork))
	,m_trainer(*m_onlineNetwork)
	,m_targetInstance(&m_targetNetwork)
	,m_shuffleBuffer()
	,m_replayBuffer()
	,m_gamma(ndReal(0.99f))
	,m_learnRate(ndReal(5.0e-4f))
	,m_epsilonGreedy(ndReal(1.0f))
	,m_epsilonGreedyStep(ndReal(5.0e-4f))
	,m_epsilonGreedyFloor(ndReal(2.0e-3f))
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_bashBufferSize(32)
	,m_epsilonGreedyFreq(64)
	,m_targetUpdatePeriod(1000)
{
	m_trainer.m_agent = this;
	m_onlineNetwork->InitGaussianWeights(0.0f, 0.125f);
	//SetRegularizer(GetRegularizer() * 10.0f);

	SetBufferSize(1024 * 128);
	m_targetNetwork.CopyFrom(*(*m_onlineNetwork));
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN<statesDim, actionDim>::~ndBrainAgentDQN()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::SetBufferSize(ndInt32 size)
{
	m_shuffleBuffer.SetCount(size);
	m_replayBuffer.SetCount(size);

	m_shuffleBuffer.SetCount(0);
	m_replayBuffer.SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& ndBrainAgentDQN<statesDim, actionDim>::GetTransition()
{
	return m_currentTransition;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN<statesDim, actionDim>::GetAction() const
{
	ndInt32 action = 0;
	ndFloat32 explore = ndRand();
	if (explore <= m_epsilonGreedy)
	{
		action = ndInt32(ndRandInt() % actionDim);
	}
	else
	{
		ndAssert(0);
		//action = m_agent->GetMaxValueAction();
		//for (ndInt32 i = 0; i < m_stateCount; ++i)
		//{
		//	m_input[i] = m_currentTransition.m_state[i];
		//}
		//ndBrainInstance& instance = m_trainer.GetInstance();
		//instance.MakePrediction(m_input, m_output);
		//
		//ndReal maxReward = m_output[0];
		//for (ndInt32 i = 1; i < m_actionsCount; ++i)
		//{
		//	if (m_output[i] > maxReward)
		//	{
		//		action = i;
		//		maxReward = m_output[i];
		//	}
		//}
	}

	return action;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::BackPropagate()
{
	class ndTestValidator : public ndBrainTrainer::ndValidation
	{
		public:
		ndTestValidator(ndBrainTrainer& trainer)
			:ndBrainTrainer::ndValidation(trainer)
			//,m_minError(1.0e10f)
			//,m_step(0)
			//,m_step0(0)
		{
		}
	
		//ndReal Validate(const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth)
		ndReal Validate(const ndBrainMatrix& inputBatch)
		{
			//ndReal error = ndBrainTrainer::ndValidation::Validate(inputBatch, groundTruth);
			//if (error < m_minError)
			//{
			//	m_minError = error;
			//	ndExpandTraceMessage("%f; %d; %d\n", m_minError, m_step, m_step - m_step0);
			//	m_step0 = m_step;
			//}
			//m_step++;
			////ndExpandTraceMessage("%f\n", error);
			//return error;
			return ndReal (1.0f);
		}
		//ndReal m_minError;
		//ndInt32 m_step;
		//ndInt32 m_step0;
	};

	ndTestValidator validator(m_trainer);
	
	//m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	//for (ndInt32 i = 0; i < m_bashBufferSize; ++i)
	//{
	//	//ndInt32 index = m_shuffleBuffer[i];
	//	//const ndBrainReplayTransitionMemory<ndInt32, m_stateCount, 1>& transition = m_replayBuffer[index];
	//	//for (ndInt32 j = 0; j < m_stateCount; ++j)
	//	//{
	//	//	inputBatch[i][j] = transition.m_state[j];
	//	//}
	//}

	ndBrainMatrix inputBatch;
	m_trainer.Optimize(validator, inputBatch, m_learnRate, 1);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::LearnStep()
{
	if (!m_frameCount)
	{
		ResetModel();
	}

	GetObservation(&m_currentTransition.m_nextState[0]);
	m_currentTransition.m_reward = GetReward();
	m_currentTransition.m_terminalState = IsTerminal();

	m_replayBuffer.AddTransition(m_currentTransition);

	if (m_frameCount < m_shuffleBuffer.GetCapacity())
	{
		m_shuffleBuffer.PushBack(m_frameCount);
	}

	m_currentTransition.m_state = m_currentTransition.m_nextState;
	m_currentTransition.m_action[0] = GetAction();

	if (m_currentTransition.m_terminalState)
	{
		m_eposideCount++;
		ResetModel();
	}

	if (m_frameCount % m_epsilonGreedyFreq == (m_epsilonGreedyFreq - 1))
	{
		m_epsilonGreedy = ndMax(m_epsilonGreedy - m_epsilonGreedyStep, m_epsilonGreedyFloor);
	}

	if (m_frameCount > (m_bashBufferSize * 8))
	{
		BackPropagate();
	}

	if ((m_frameCount % m_targetUpdatePeriod) == (m_targetUpdatePeriod - 1))
	{
		// update on line network
		m_targetNetwork.CopyFrom(*(*m_onlineNetwork));
	}

	m_frameCount++;
}

#endif 

