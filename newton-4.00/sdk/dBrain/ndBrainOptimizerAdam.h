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

#ifndef _ND_BRAIN_OPTIMIZER_ADAM_H__
#define _ND_BRAIN_OPTIMIZER_ADAM_H__

#include "ndBrainStdafx.h"
#include "ndBrainOptimizer.h"

class ndBrainOptimizerAdam : public ndBrainOptimizer
{
	public: 
	class ndAdamData;
	ndBrainOptimizerAdam(ndBrainTrainer* const trainer);
	virtual ~ndBrainOptimizerAdam();

	virtual void Update(ndBrainFloat learnRate, ndInt32 bashSize);

	ndArray<ndAdamData*> m_data;
	ndBrainFloat m_beta;
	ndBrainFloat m_alpha;
	ndBrainFloat m_epsilon;
	ndBrainFloat m_betaAcc;
	ndBrainFloat m_alphaAcc;
};

#endif 
