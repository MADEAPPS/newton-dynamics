/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _ND_DEEP_BRAIN_INSTANCE_H__
#define _ND_DEEP_BRAIN_INSTANCE_H__

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainVector.h"

class ndDeepBrain;
class ndDeepBrainLayer;
class ndDeepBrainTrainingOperator;

class ndDeepBrainInstance: public ndClassAlloc
{
	public: 
	ndDeepBrainInstance(ndDeepBrain* const brain);
	~ndDeepBrainInstance();

	ndDeepBrainVector& GetInputs();
	ndDeepBrainVector& GetOutputs();
	ndArray<ndDeepBrainLayer*>& GetLayers();
	const ndArray<ndDeepBrainLayer*>& GetLayers() const;

	void MakePrediction(const ndDeepBrainVector& input);

	protected:
	void SetInput(const ndDeepBrainVector& input);
	void MakeTrainingPrediction(const ndDeepBrainVector& input, ndDeepBrainTrainingOperator& trainingOperator);
	void BackPropagate(ndDeepBrainTrainingOperator& trainingOperator);

	ndDeepBrainVector m_inputs;
	ndDeepBrainVector m_outputs;
	ndDeepBrain* m_brain;

	friend class ndDeepBrainGradientDescendTrainingOperator;
};


#endif 

