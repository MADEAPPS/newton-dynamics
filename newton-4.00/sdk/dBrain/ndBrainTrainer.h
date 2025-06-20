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
class ndBrainBuffer;
class ndBrainContext;

class ndBrainTrainer: public ndClassAlloc
{
	public: 
	ndBrainTrainer(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context);
	ndBrainTrainer(const ndBrainTrainer& src);
	virtual ~ndBrainTrainer();

	ndSharedPtr<ndBrain>& GetBrain();
	ndSharedPtr<ndBrainContext> GetContext();

	virtual ndBrainBuffer* GetInputBuffer() = 0;
	virtual void SaveInput(ndBrainVector& ouput) const = 0;
	virtual void LoadInput(const ndBrainVector& input) = 0;

	virtual void GetOutput(ndBrainVector&) const {}
	virtual void GetWorkingBuffer(ndBrainVector&) const {}
	virtual void GetParameterBuffer(ndBrainVector&) const {}
	virtual void GetGradientBuffer(ndBrainVector&) const {}
	virtual void SoftCopyParameters(const ndBrainTrainer&, ndBrainFloat) {}

	// legacy method;
	virtual void BackPropagate(const ndBrainVector& input, ndBrainLoss& loss) = 0;

	// new method
	virtual void SyncQueue() = 0;

	// new method
	virtual void MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output) = 0;

	// new method
	virtual void MakePrediction() = 0;
	void MakePrediction(const ndBrainVector& input);

	// new method
	virtual void BackPropagate(const ndBrainVector& outputGradients) = 0;

	// new method
	virtual void ApplyLearnRate() = 0;

	// new method
	void UpdateParameters();
	virtual void UpdateParameters(const ndBrainVector& weightAndBias) = 0;

	protected:
	ndSharedPtr<ndBrain> m_brain;
	ndSharedPtr<ndBrainContext> m_context;
};

#endif 

