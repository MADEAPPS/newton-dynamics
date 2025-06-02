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

#ifndef _ND_BRAIN_TRAINER_GPU_INFERENCE_H__
#define _ND_BRAIN_TRAINER_GPU_INFERENCE_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainTrainer.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuCommand.h"

class ndBrain;
class ndBrainLoss;
class ndBrainLayer;

class ndBrainTrainnerGpuInference: public ndBrainTrainer
{
	public: 
	class ndUniformBufferObject
	{
		public:
		ndUniformBufferObject()
			:m_inputSize(0)
			,m_outputSize(0)
			,m_parametersStartOffset(0)
			,m_inputOutputSize(0)
			,m_inputOutputStartOffset(0)
		{
		}

		ndUnsigned32 m_inputSize;
		ndUnsigned32 m_outputSize;
		ndUnsigned32 m_parametersStartOffset;
		ndUnsigned32 m_inputOutputSize;
		ndUnsigned32 m_inputOutputStartOffset;
	};

	class ndGpuCommand : public ndBrainGpuCommand
	{
		public:
		ndGpuCommand(ndBrainGpuContext* const context,
			ndVulkanShader m_shader,
			ndInt32 numberOfinputs,
			const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
			ndBrainGpuBuffer* const buffer1,
			ndBrainGpuBuffer* const buffer2, size_t id);

		ndSharedPtr<ndBrainGpuBuffer> m_uniformBuffer;
	};

	ndBrainTrainnerGpuInference(
		const ndSharedPtr<ndBrain>& brain, 
		const ndSharedPtr<ndBrainContext>& context, 
		ndInt32 minibatchSize);
	ndBrainTrainnerGpuInference(const ndBrainTrainnerGpuInference& src);
	virtual ~ndBrainTrainnerGpuInference();

	virtual void GetInput(ndBrainVector& ouput) const;
	virtual void GetOutput(ndBrainVector& ouput) const override;
	virtual void GetWorkingBuffer(ndBrainVector& ouput) const;
	virtual void GetParameterBuffer(ndBrainVector& ouput) const;

	// legacy
	virtual void BackPropagate(const ndBrainVector&, ndBrainLoss&) override { ndAssert(0); }

	// new methods
	virtual void UpdateParameters() override;
	virtual void ApplyLearnRate(ndBrainFloat learnRate) override;
	virtual void MakePrediction(const ndBrainVector& input) override;
	virtual void BackPropagate(const ndBrainVector& outputGradients) override;
	virtual void MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output) override;

	protected:
	enum ndInputOutputCommandId
	{
		m_inputId = 7,
		m_outpuId = 8,
	};

	void SubmitCommands();
	void AddCopyOutputCommand();
	void InitInputOutputBuffer();
	void InitWeightAndBiasBuffer();
	ndBrainGpuCommand* FindCommand(size_t id) const;
	void AddCopyInputCommand(const ndLayerUniformDataGpu& uniformData);
	void UnloadBuffer(ndBrainVector& ouput, const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer) const;
	void AddLayersCommands(ndFixSizeArray<ndLayerUniformDataGpu, 256>& layersUniformsData);

	ndBrainGpuContext* m_context;
	ndSharedPtr<ndBrainContext> m_contextRef;
	ndSharedPtr<ndBrainGpuBuffer> m_inputOutputBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_weightAndBiasBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_miniBatchOutputBuffer;
	ndList<ndSharedPtr<ndBrainGpuCommand>> m_feedFowardCommands;
	ndInt32 m_miniBatchSize;
};

#endif 

