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
#include "ndBrain.h"
#include "ndBrainAgentDQN.h"

//ndBrainAgentDQN::ndBrainAgentDQN(ndBrain* const agent, ndInt32 replayBufferSize, ndInt32 replayBatchSize)
ndBrainAgentDQN::ndBrainAgentDQN(ndBrain* const agent, ndInt32, ndInt32)
	:ndBrainAgent(agent)
	,m_targetNetwork(new ndBrain(*agent))
{
	ndAssert(0);
	//ndInt32 stateSize = m_network.GetBrain()->GetInputSize();
	//ndInt32 actionSize = m_network.GetBrain()->GetOutputSize();
	//m_replayBuffer.SetCount(replayBufferSize, replayBatchSize, stateSize, actionSize);
}

ndBrainAgentDQN::~ndBrainAgentDQN()
{
	delete m_targetNetwork.GetBrain();
}

//void ndBrainAgentDQN::PredictAccion(ndBrainReiforcementTransition& transition)
//{
//	ndBrainAgent::PredictAccion(transition);
//}

void ndBrainAgentDQN::LearnStep()
{
	//ndBrainReiforcementTransition& transition = m_replayBuffer.GetTransitionEntry();
	//GetTransition(transition);
	//
	//if (m_replayBuffer.m_replayBufferIndex < m_replayBuffer.m_learnBatchSize)
	//{
	//	return;
	//}
	//
	//m_replayBuffer.MakeRandomBatch();
	////ndAssert(0);
}