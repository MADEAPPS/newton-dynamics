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

class ndBrainTrainer: public ndClassAlloc
{
	public: 
	class ndLayerData;

	ndBrainTrainer(ndBrain* const brain);
	ndBrainTrainer(const ndBrainTrainer& src);
	virtual ~ndBrainTrainer();

	ndBrain* GetBrain() const;
	void BackPropagate(const ndBrainVector& input, ndBrainLoss& loss);
	void AcculumateGradients(const ndBrainTrainer& src, ndInt32 index);

	ndBrainLayer* GetWeightsLayer(ndInt32 index) const;
	ndBrainLayer* GetGradientLayer(ndInt32 index) const;

	void ClearGradients();
	void ScaleWeights(const ndBrainFloat s);
	void AddGradients(const ndBrainTrainer* const src);

	ndBrainVector& GetWorkingBuffer();

	private:
	ndArray<ndLayerData*> m_data;
	ndBrainVector m_workingBuffer;
	ndFixSizeArray<ndInt32, 256> m_prefixScan;
	ndBrain* m_brain;
	ndInt32 m_workingBufferSize;
	ndInt32 m_maxLayerBufferSize;
};

#endif 

