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

#ifndef _ND_BRAIN_TRAINER_GPU_H__
#define _ND_BRAIN_TRAINER_GPU_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainTrainer.h"
#include "ndBrainTrainerGpuInference.h"

class ndBrainOptimizerAdamGpu;

class ndBrainTrainerGpu: public ndBrainTrainerGpuInference
{
	public: 
	ndBrainTrainerGpu(const ndTrainerDescriptor& descriptor);
	ndBrainTrainerGpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndBrainFloat learnRate, ndInt32 minibatchSize);
	ndBrainTrainerGpu(const ndBrainTrainerGpu& src);
	virtual ~ndBrainTrainerGpu();

	virtual void ApplyLearnRate() override;
	virtual void GetGradientBuffer(ndBrainVector&) const override;
	virtual void BackPropagate(const ndBrainVector& outputGradients) override;

	protected:
	void AddLayersGradientCommands();
	void AddCopyInputGradientCommand();
	void AddCopyOutputGradientCommand();
	void Initialize(const ndTrainerDescriptor& descriptor);
	void AddOptimizerGradientCommand(ndBrainFloat learnRate);

	ndSharedPtr<ndBrainOptimizerAdamGpu> m_optimizer;
	ndSharedPtr<ndBrainGpuCommand> m_adamOtimizerUpdate;
	ndSharedPtr<ndBrainGpuCommand> m_adamMomentumUpdate;

	ndSharedPtr<ndBrainGpuFloatBuffer> m_inputOutputGradientsBuffer;
	ndSharedPtr<ndBrainGpuFloatBuffer> m_weightAndBiasGradientsBuffer;
	ndSharedPtr<ndBrainGpuFloatBuffer> m_miniBatchInputGradientBuffer;
	ndSharedPtr<ndBrainGpuFloatBuffer> m_miniBatchOutputGradientBuffer;

	ndList<ndSharedPtr<ndBrainGpuCommand>> m_backPropagateCommands;
	ndList<ndSharedPtr<ndBrainGpuCommand>> m_accumulateGradientsCommands;
};

#endif 

