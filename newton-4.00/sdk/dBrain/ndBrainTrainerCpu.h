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

class ndBrainLayerBackPropagateCpuCommand: public ndBrainTrainerCpuCommand
{
	public:
	ndBrainLayerBackPropagateCpuCommand(ndBrainLayer* const layer)
		:ndBrainTrainerCpuCommand(layer->GetCpuCommandSharedInfo(), size_t(layer))
		,m_layer(layer)
	{
	}

	virtual void Execute(ndInt32 miniBatchIndex)
	{
		m_layer->BackPropagate(this, miniBatchIndex);
	}
	const ndBrainLayer* m_layer;
};

class ndBrainTrainerCpu: public ndBrainTrainerCpuInference
{
	public: 
	ndBrainTrainerCpu(const ndTrainerDescriptor& descriptor);
	ndBrainTrainerCpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndBrainFloat learnRate, ndInt32 minibatchSize);
	ndBrainTrainerCpu(const ndBrainTrainerCpu& src);

	virtual void ApplyLearnRate() override;
	virtual void SaveInput(ndBrainVector& input) const override;

	virtual void BackPropagate(const ndBrainVector& outputGradients) override;

	protected:
	class ndCopyOutputGradientCommand : public ndBrainTrainerCpuCommand
	{
		public:
		ndCopyOutputGradientCommand(ndBrainTrainerCpu* const owner)
			:ndBrainTrainerCpuCommand(ndBrainLayer::ndCommandShareInfo(nullptr), m_outpuId)
		{
			m_owner = owner;
		}

		virtual void Execute(ndInt32 miniBatchIndex)
		{
			ndBrainTrainerCpu* const owner = (ndBrainTrainerCpu*)m_owner;
			const ndBrainMemVector src(&owner->m_miniBatchOutputGradientBuffer[miniBatchIndex * m_info.m_outputSize], m_info.m_outputSize);
			ndInt32 destOffset = miniBatchIndex * m_info.m_inputOutputSize;
			ndBrainMemVector dst(&owner->m_inputOuputGradientsBuffer[destOffset + m_info.m_inputOutputStartOffset], m_info.m_outputSize);
			dst.Set(src);
		}
	};

	class ndCopyInputGradientCommand : public ndBrainTrainerCpuCommand
	{
		public:
		ndCopyInputGradientCommand(ndBrainTrainerCpu* const owner)
			:ndBrainTrainerCpuCommand(ndBrainLayer::ndCommandShareInfo(nullptr), m_inputId)
		{
			m_owner = owner;
		}
	
		virtual void Execute(ndInt32 miniBatchIndex)
		{
			ndBrainTrainerCpu* const owner = (ndBrainTrainerCpu*)m_owner;
			ndInt32 srcOffset = miniBatchIndex * m_info.m_inputOutputSize;
			const ndBrainMemVector src(&owner->m_inputOuputGradientsBuffer[srcOffset + m_info.m_inputOutputStartOffset], m_info.m_inputSize);
			ndBrainMemVector dst(&owner->m_miniBatchInputGradientBuffer[miniBatchIndex * m_info.m_inputSize], m_info.m_inputSize);
			dst.Set(src);
		}
	};

	void AddLayersGradientCommands();
	void AddCopyInputGradientCommand();
	void AddCopyOutputGradientCommand();
	void Initialize(const ndTrainerDescriptor& descriptor);

	ndSharedPtr<ndBrainOptimizerAdamCpu> m_optimizer;
	ndBrainVector m_inputOuputGradientsBuffer;
	ndBrainVector m_weightAndBiasGradientsBuffer;
	ndBrainVector m_miniBatchInputGradientBuffer;
	ndBrainVector m_miniBatchOutputGradientBuffer;
	ndList<ndSharedPtr<ndBrainTrainerCpuCommand>> m_backPropagateCommands;

	friend class ndBrainLayerLinear;
	friend class ndBrainLayerActivationRelu;
	friend class ndBrainLayerActivationTanh;
	friend class ndBrainLayerActivationSoftmax;
	friend class ndBrainLayerLinearWithDropOut;
	friend class ndBrainLayerActivationCategoricalSoftmax;
};

#endif 

