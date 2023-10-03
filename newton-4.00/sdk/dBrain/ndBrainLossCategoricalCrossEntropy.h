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

#ifndef _ND_BRAIN_LOSS_CATEGORICAL_CROSS_ENTROPY_H__
#define _ND_BRAIN_LOSS_CATEGORICAL_CROSS_ENTROPY_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainLoss.h"

// note: Categorical entropy loss is designed you work with the SoftMax activation layer
// the rules for using it are
// 1- can only be use as when the last layer of the neural net is SoftMax layer
// 2- the function does not calculate the derivative since this is done by the SoftMax layer 
// which make use that the combine the truth value can only be 1 or 0, 
// and this fact cancel out many term from the derivative equation. 
// in that regard, the loss is just the truth value.
class ndBrainLossCategoricalCrossEntropy: public ndBrainLoss
{
	public:
	ndBrainLossCategoricalCrossEntropy(ndInt32 size);
	void SetTruth(const ndBrainVector& truth);
	virtual void GetLoss(const ndBrainVector& output, ndBrainVector& loss);

	virtual bool IsCategorical() const;

	ndBrainVector m_truth;
};

#endif 

