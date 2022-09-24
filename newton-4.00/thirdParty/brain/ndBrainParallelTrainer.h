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

#ifndef _ND_BRAIN_PARALLEL_TRAINER_H__
#define _ND_BRAIN_PARALLEL_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrainTrainer.h"


class ndBrainParallelTrainer: public ndBrainTrainer, public ndThreadPool
{
	public: 
	class ndBrainTrainerChannel;
	ndBrainParallelTrainer(ndBrain* const brain, ndInt32 threads = 1);
	~ndBrainParallelTrainer();

	virtual void Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps);

	private:
	void Optimize();
	ndReal Validate(const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndBrainVector& output);
	virtual void ThreadFunction();

	private:
	void AverageWeights();
	ndFixSizeArray<ndBrainTrainerChannel*, D_MAX_THREADS_COUNT> m_threadData;

	const ndBrainMatrix* m_inputBatch;
	const ndBrainMatrix* m_groundTruth;
	ndValidation* m_validator;
	ndReal m_learnRate;
	ndInt32 m_steps;
};


#endif 

