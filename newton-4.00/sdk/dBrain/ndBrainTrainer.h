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
#include "ndBrainTrainerInference.h"

class ndBrainOptimizerAdam;

class ndBrainTrainer: public ndBrainTrainerInference
{
	public: 
	ndBrainTrainer(const ndTrainerDescriptor& descriptor);
	ndBrainTrainer(const ndBrainTrainer& src);
	virtual ~ndBrainTrainer();

	ndBrainFloatBuffer* GetOuputGradientBuffer();

	ndBrainFloatBuffer* GetHiddenLayerGradientBuffer();
	ndBrainFloatBuffer* GetWeightAndBiasGradientBuffer();
	ndBrainFloatBuffer* GetPartialSumBiasGradientBuffer();

	
	//virtual void GetGradientBuffer(ndBrainVector&) const override;
	//virtual void BackPropagate(const ndBrainVector& outputGradients) override;

	void BackPropagate();
	void ApplyLearnRate();
	
	protected:
	void Initialize();
	void AddLayersGradientCommands();
	void AddCopyInputGradientCommand();
	void AddOptimizerGradientCommand();
	void AddCopyOutputGradientCommand();

	ndSharedPtr<ndBrainOptimizerAdam> m_optimizer;
	ndSharedPtr<ndBrainBufferCommand> m_adamOtimizerUpdate;
	ndSharedPtr<ndBrainBufferCommand> m_adamMomentumUpdate;

	ndSharedPtr<ndBrainFloatBuffer> m_inputOutputGradientsBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_weightAndBiasGradientsBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_miniBatchInputGradientBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_miniBatchOutputGradientBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_biasPartialSumGradientsCacheBuffer;

	ndList<ndSharedPtr<ndBrainBufferCommand>> m_backPropagateCommands;
	ndList<ndSharedPtr<ndBrainBufferCommand>> m_optimizerBufferCommands;
};

#endif 

