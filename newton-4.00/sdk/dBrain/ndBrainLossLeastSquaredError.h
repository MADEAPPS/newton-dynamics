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

#ifndef _ND_BRAIN_LOSS_LEAST_SQUARED_ERROR_H__
#define _ND_BRAIN_LOSS_LEAST_SQUARED_ERROR_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainLoss.h"

class ndBrainLossLeastSquaredError: public ndBrainLoss
{
	public:
	ndBrainLossLeastSquaredError(ndInt32 size);
	void SetTruth(const ndBrainVector& truth);
	virtual void GetLoss(const ndBrainVector& output, ndBrainVector& loss);

	virtual bool HasGpuSupport() const;
	ndBrainVector m_truth;
};

class ndBrainLossHuber : public ndBrainLossLeastSquaredError
{
	public:
	ndBrainLossHuber(ndInt32 size, ndBrainFloat lambda = ndBrainFloat(1.0f));
	virtual void GetLoss(const ndBrainVector& output, ndBrainVector& loss);

	ndBrainFloat m_lambda;
};

#endif 

