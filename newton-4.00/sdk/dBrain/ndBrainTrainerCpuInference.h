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

#ifndef _ND_BRAIN_TRAINER_CPU_INFERENCE_H__
#define _ND_BRAIN_TRAINER_CPU_INFERENCE_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainTrainer.h"

class ndBrain;
class ndBrainLoss;
class ndBrainLayer;
class ndBrainThreadPool;

class ndBrainTrainerCpuInference: public ndBrainTrainer
{
	public: 
	ndBrainTrainerCpuInference(
		const ndSharedPtr<ndBrain>& brain, 
		const ndSharedPtr<ndBrainContext>& context,
		ndInt32 minibatchSize);
	ndBrainTrainerCpuInference(const ndBrainTrainerCpuInference& src);

	virtual void GetInput(ndBrainVector& input) const override;
	virtual void GetOutput(ndBrainVector& ouput) const override;
	virtual void SoftCopyParameters(const ndBrainTrainer& src, ndBrainFloat blendFactor) override;

	// legacy method;
	virtual void BackPropagate(const ndBrainVector&, ndBrainLoss&) override { ndAssert(0);}

	// new method
	virtual void MakePrediction(const ndBrainVector& input) override;
	// new method

	// new method
	virtual void MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output) override;

	// new method
	virtual void BackPropagate(const ndBrainVector& outputGradients) override;

	// new method
	virtual void ApplyLearnRate(ndBrainFloat learnRate) override;

	// new method
	virtual void UpdateParameters() override;

	protected:
	enum ndInputOutputCommandId
	{
		m_inputId = 7,
		m_outpuId = 8,
	};

	class ndCopyInputCommand : public ndBrainTrainerCpuCommand
	{
		public:
		ndCopyInputCommand(ndBrainTrainerCpuInference* const owner)
			:ndBrainTrainerCpuCommand(m_inputId)
			,m_owner(owner)
			,m_inputSize(0)
			,m_outputSize(0)
			,m_parametersSize(0)
			,m_inputOutputSize(0)
			,m_inputOutputStartOffset(0)
		{
		}

		virtual void Execute(ndInt32 miniBatchIndex)
		{
			const ndBrainMemVector src(&m_owner->m_miniBatchInputBuffer[miniBatchIndex * m_inputSize], m_inputSize);
			ndBrainMemVector dst(&m_owner->m_inputOutputBuffer[miniBatchIndex * m_inputOutputSize], m_inputSize);
			dst.Set(src);
		}

		ndBrainTrainerCpuInference* m_owner;
		ndInt32 m_inputSize;
		ndInt32 m_outputSize;
		ndInt32 m_parametersSize;
		ndInt32 m_inputOutputSize;
		ndInt32 m_inputOutputStartOffset;
	};

	class ndCopyOutputCommand : public ndBrainTrainerCpuCommand
	{
		public:
		ndCopyOutputCommand(ndBrainTrainerCpuInference* const owner)
			:ndBrainTrainerCpuCommand(m_outpuId)
			,m_owner(owner)
			,m_inputSize(0)
			,m_outputSize(0)
			,m_parametersSize(0)
			,m_inputOutputSize(0)
			,m_inputOutputStartOffset(0)
		{
		}

		virtual void Execute(ndInt32 miniBatchIndex)
		{
			const ndBrainMemVector src(&m_owner->m_inputOutputBuffer[miniBatchIndex * m_inputOutputSize + m_inputOutputStartOffset], m_outputSize);
			ndBrainMemVector dst(&m_owner->m_miniBatchOutputBuffer[miniBatchIndex * m_outputSize], m_outputSize);
			dst.Set(src);
		}

		ndBrainTrainerCpuInference* m_owner;
		ndInt32 m_inputSize;
		ndInt32 m_outputSize;
		ndInt32 m_parametersSize;
		ndInt32 m_inputOutputSize;
		ndInt32 m_inputOutputStartOffset;
	};

	ndBrainTrainerCpuCommand* FindCommand(size_t id) const;

	void AddCopyOutputCommand();
	void InitInputOutputBuffer();
	void InitWeightAndBiasBuffer();
	void AddCopyInputCommand(const ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const firstCommand);
	void AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndBrainLayerFeedFowardCpuCommand*, 256>& layersCommands);

	ndBrainVector m_inputOutputBuffer;
	ndBrainVector m_weightAndBiasBuffer;
	ndBrainVector m_miniBatchInputBuffer;
	ndBrainVector m_miniBatchOutputBuffer;
	ndList<ndSharedPtr<ndBrainTrainerCpuCommand>> m_feedFowardCommands;
	ndBrainThreadPool* m_threadPool;
	ndInt32 m_miniBatchSize;

	friend class ndBrainLayerLinear;
	friend class ndBrainLayerActivationRelu;
	friend class ndBrainLayerActivationTanh;
	friend class ndBrainLayerActivationSoftmax;
	friend class ndBrainLayerLinearWithDropOut;
	friend class ndBrainLayerActivationCategoricalSoftmax;
};

#endif 

