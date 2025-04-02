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

#ifndef _ND_AGENT_CONTINUE_PROXIMA_POLICY_GRADIENT_TRAINER_H__
#define _ND_AGENT_CONTINUE_PROXIMA_POLICY_GRADIENT_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrainAgentContinuePolicyGradient_Trainer.h"
//#include "ndBrain.h"
//#include "ndBrainAgent.h"
//#include "ndBrainThreadPool.h"
//#include "ndBrainLayerActivationRelu.h"
//#include "ndBrainLayerActivationTanh.h"
//#include "ndBrainLossLeastSquaredError.h"
//#include "ndBrainLayerActivationSigmoidLinear.h"

// This is an implementation of the proxima policy Gradient as described in:
// https://spinningup.openai.com/en/latest/algorithms/ppo.html
// it is an impropment that allows multiple passes on the same data collection as 
// a long as the two disrtribution are close.

// I have a huge misunderstanding of this algorithm.
// no matter what I do I can't get it to work at all, in fact it is the worst trainer so far. 


class ndBrainAgentContinueProximaPolicyGradient_TrainerMaster : public ndBrainAgentContinuePolicyGradient_TrainerMaster
{
	public:
	ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(const HyperParameters& hyperParameters);
	virtual ~ndBrainAgentContinueProximaPolicyGradient_TrainerMaster();

	virtual void Optimize();
	private:
	void OptimizePolicyPPOstep();
	ndBrainFloat CalculateKLdivergence();
	
	ndBrain m_oldPolicy;
	ndBrain m_tempPolicy;
};

#endif 