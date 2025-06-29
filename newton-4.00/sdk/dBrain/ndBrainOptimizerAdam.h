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


class ndBrainFloatBuffer;

class ndBrainOptimizerAdam : public ndBrainOptimizer
{
	public: 
	class ndCommandShareInfo
	{
		public:
		ndCommandShareInfo()
			:m_beta(ndBrainFloat(0.999f))
			,m_alpha(ndBrainFloat(0.9f))
			,m_epsilon(ndBrainFloat(1.0e-6f))
			,m_betaAcc(m_beta)
			,m_alphaAcc(m_alpha)
			,m_learnRate(ndBrainFloat(1.0e-4f))
			,m_invBeta(ndBrainFloat(1.0f)/ (ndBrainFloat(1.0f) - m_beta))
			,m_invAlpha(ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_alpha))
			,m_decayRegularizer(ndBrainFloat(1.0e-4f))
			//,m_parametersSize(0)
		{
		}

		ndBrainFloat m_beta;
		ndBrainFloat m_alpha;
		ndBrainFloat m_epsilon;
		ndBrainFloat m_betaAcc;
		ndBrainFloat m_alphaAcc;
		ndBrainFloat m_learnRate;
		ndBrainFloat m_invBeta;
		ndBrainFloat m_invAlpha;
		ndBrainFloat m_decayRegularizer;
	};

	ndBrainOptimizerAdam(const ndSharedPtr<ndBrainContext>& context);

	// lageavy
	virtual void Update(ndBrainVector& parameters, const ndBrainVector& gradients) override;

	// new system
	virtual void Init(ndInt32 parametersBufferSizeInFloats, ndBrainFloat m_learnRate) override;

	private:
	ndSharedPtr<ndBrainFloatBuffer> m_vdw;
	ndSharedPtr<ndBrainFloatBuffer> m_vdw2;
	ndCommandShareInfo m_parameters;

	friend class ndBrainTrainer;
};

#endif 

