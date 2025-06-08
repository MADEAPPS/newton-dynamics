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

#ifndef _ND_BRAIN_OPTIMIZER_ADAM_GPU_H__
#define _ND_BRAIN_OPTIMIZER_ADAM_GPU_H__

#include "ndBrainStdafx.h"
#include "ndBrainOptimizer.h"

class ndBrainThreadPool;

class ndBrainOptimizerAdamGpu : public ndBrainOptimizer
{
	public: 
	ndBrainOptimizerAdamGpu(const ndSharedPtr<ndBrainContext>& context);

	virtual void Init(ndInt32 parametersBufferSizeInFloats) override;
	virtual void Update(ndBrainVector& parameters, const ndBrainVector& gradients, ndBrainFloat learnRate) override;

	private:
	//ndBrainVector m_vdw;
	//ndBrainVector m_vdw2;
	//ndBrainVector m_temp;
	//ndBrainVector m_vdwCorrected;
	//ndBrainVector m_vdw2Corrected;
	ndSharedPtr<ndBrainGpuBuffer> m_vdw;
	ndSharedPtr<ndBrainGpuBuffer> m_vdw2;
	ndSharedPtr<ndBrainGpuBuffer> m_temp;
	ndSharedPtr<ndBrainGpuBuffer> m_vdwCorrected;
	ndSharedPtr<ndBrainGpuBuffer> m_vdw2Corrected;

	ndBrainFloat m_beta;
	ndBrainFloat m_alpha;
	ndBrainFloat m_epsilon;
	ndBrainFloat m_betaAcc;
	ndBrainFloat m_alphaAcc;
	ndBrainThreadPool* m_threadPool;
	ndInt32 m_miniBatchSize;

	friend class ndBrainTrainerCpu;
};

#endif 

