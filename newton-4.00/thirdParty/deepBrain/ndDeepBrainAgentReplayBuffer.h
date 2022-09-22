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

#ifndef _ND_DEEP_BRAIN_AGENT_REAPLAY_BUFFER_H__
#define _ND_DEEP_BRAIN_AGENT_REAPLAY_BUFFER_H__

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainVector.h"
#include "ndDeepBrainMatrix.h"
#include "ndDeepBrainInstance.h"

class ndDeepBrainTransition
{
	public:
	ndDeepBrainTransition();
	void CopyFrom(const ndDeepBrainTransition& src);

	ndDeepBrainVector m_state;
	ndDeepBrainVector m_action;
	ndDeepBrainVector m_nextState;
	ndReal m_reward;
	bool m_terminalState;
};

class ndDeepBrainReplayBuffer : public ndArray<ndDeepBrainTransition>
{
	public:
	ndDeepBrainReplayBuffer();
	~ndDeepBrainReplayBuffer();

	void SetCount(ndInt32 count, ndInt32 stateSize, ndInt32 actionSize);

	//ndInt32 GetStateSize() const;
	//ndInt32 GetActionSize() const;

	ndDeepBrainTransition& GetTransitionEntry();

	void MakeRandomBatch();

	ndArray<ndUnsigned32> m_randomShaffle;
	ndDeepBrainMatrix m_inputBatch;
	ndDeepBrainMatrix m_outputBatch;
	ndDeepBrainMatrix m_nextInputBatch;
	ndDeepBrainMatrix m_groundTruthBatch;
	ndDeepBrainVector n_rewardBatch;
	ndDeepBrainVector n_terminalBatch;
	ndInt32 m_learnBashSize;
	ndInt32 m_replayBufferIndex;
};

#endif 

