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

#ifndef _ND_BRAIN_TRAINER_CPU_H__
#define _ND_BRAIN_TRAINER_CPU_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainTrainer.h"
#include "ndBrainOptimizerAdamCpu.h"
#include "ndBrainTrainerCpuInference.h"

class ndBrainTrainerCpu: public ndBrainTrainerCpuInference
{
	public: 
	ndBrainTrainerCpu(
		const ndSharedPtr<ndBrain>& brain, 
		const ndSharedPtr<ndBrainOptimizerAdamCpu>& optimizer, 
		ndBrainThreadPool* const threadPool, ndInt32 minibatchSize);
	ndBrainTrainerCpu(const ndBrainTrainerCpu& src);

	virtual void UpdateParameters() override;
	virtual void ApplyLearnRate(ndBrainFloat learnRate) override;
	virtual void BackPropagate(const ndBrainVector& outputGradients) override;

	protected:
	void AddLayersGradientCommands();
	void AddCopyOutputGradientCommand();

	ndBrainVector m_inputOuputGradientsBuffer;
	ndBrainVector m_weightAndBiasGradientsBuffer;
	ndBrainVector m_miniBatchInputGradientBuffer;
	ndBrainVector m_miniBatchOutputGradientBuffer;
	ndSharedPtr<ndBrainOptimizerAdamCpu> m_optimizer;
	ndList<ndSharedPtr<ndBrainTrainerCpuCommand>> m_backPropagateCommands;

	friend class ndBrainLayerLinear;
	friend class ndBrainLayerActivationRelu;
	friend class ndBrainLayerActivationTanh;
	friend class ndBrainLayerActivationSoftmax;
	friend class ndBrainLayerLinearWithDropOut;
	friend class ndBrainLayerActivationCategoricalSoftmax;
};

#endif 

