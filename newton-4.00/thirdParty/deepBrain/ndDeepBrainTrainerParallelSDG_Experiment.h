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

#ifndef _ND_DEEP_BRAIN_TRAINER_PARALLEL_SGD_EXPERIMENT_H__
#define _ND_DEEP_BRAIN_TRAINER_PARALLEL_SGD_EXPERIMENT_H__

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainTrainerSDG.h"

class ndDeepBrainTrainerParallelSDG_Experiment
	:public ndDeepBrainTrainerSDG
	,public ndThreadPool
{
	public: 
	ndDeepBrainTrainerParallelSDG_Experiment(ndDeepBrain* const brain, ndReal regularizer = 0.0f, ndInt32 threads = 1);
	~ndDeepBrainTrainerParallelSDG_Experiment();

	virtual void Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps);

	private:
	virtual void ThreadFunction();
	void MakePrediction(const ndDeepBrainVector& input);

	const ndDeepBrainMatrix* m_inputBatch;
	const ndDeepBrainMatrix* m_groundTruth;
	ndReal m_learnRate;
	ndInt32 m_steps;
};


#endif 

