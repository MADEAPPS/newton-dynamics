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

#ifndef _ND_BRAIN_REPLAY_BUFFER_H__
#define _ND_BRAIN_REPLAY_BUFFER_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainInstance.h"

class ndBrainReiforcementTransition
{
	public:
	ndBrainReiforcementTransition();
	void CopyFrom(const ndBrainReiforcementTransition& src);

	ndBrainVector m_state;
	ndBrainVector m_action;
	ndBrainVector m_nextState;
	ndReal m_reward;
	bool m_terminalState;
};

class ndBrainReplayBuffer : public ndArray<ndBrainReiforcementTransition>
{
	public:
	ndBrainReplayBuffer();
	~ndBrainReplayBuffer();

	void SetCount(ndInt32 replayBufferSize, ndInt32 replayBatchSize, ndInt32 stateSize, ndInt32 actionSize);

	ndBrainReiforcementTransition& GetTransitionEntry();

	void MakeRandomBatch();

	ndArray<ndUnsigned32> m_randomShaffle;
	ndBrainMatrix m_inputBatch;
	ndBrainMatrix m_outputBatch;
	ndBrainMatrix m_nextInputBatch;
	ndBrainMatrix m_groundTruthBatch;
	ndBrainVector n_rewardBatch;
	ndBrainVector n_terminalBatch;
	ndInt32 m_learnBatchSize;
	ndInt32 m_replayBufferIndex;
};

#endif 

