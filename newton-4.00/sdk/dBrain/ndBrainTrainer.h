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

#ifndef _ND_BRAIN_TRAINER_H__
#define _ND_BRAIN_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"

class ndBrain;
class ndBrainLoss;

class ndBrainTrainerCpuCommand: public ndContainersFreeListAlloc<ndBrainTrainerCpuCommand>
{
	public:
	ndBrainTrainerCpuCommand(size_t m_id)
		:ndContainersFreeListAlloc<ndBrainTrainerCpuCommand>()
		,m_id(m_id)
	{
	}

	virtual ~ndBrainTrainerCpuCommand()
	{
	}

	virtual void Execute(ndInt32 miniBatchIndex) = 0;

	size_t m_id;
};

class ndBrainTrainer: public ndClassAlloc
{
	public: 
	ndBrainTrainer(const ndSharedPtr<ndBrain>& brain);
	ndBrainTrainer(const ndBrainTrainer& src);
	virtual ~ndBrainTrainer();

	ndSharedPtr<ndBrain>& GetBrain();
	virtual void GetOutput(ndBrainVector&) const {}

	virtual void MakePrediction(const ndBrainVector& input) = 0;

	// legacy method;
	virtual void BackPropagate(const ndBrainVector& input, ndBrainLoss& loss) = 0;

	// new method
	virtual void BackPropagate(const ndBrainVector& outputGradients) = 0;

	// new method
	virtual void ApplyLearnRate(ndBrainFloat learnRate) = 0;

	protected:
	ndSharedPtr<ndBrain> m_brain;
};

#endif 

