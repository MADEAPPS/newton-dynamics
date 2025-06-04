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
class ndBrainTrainerGpuInference;

class ndBrainTrainerGpuCommand : public ndBrainGpuCommand
{
	public:
	ndBrainTrainerGpuCommand(ndBrainTrainerGpuInference* const owner,
		const ndBrainLayer::ndCommandShareInfo& info, size_t m_id,
		ndBrainGpuContext* const context,
		const ndSharedPtr<ndBrainGpuShader>& shader,
		ndInt32 numberOfinputs,
		const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
		ndBrainGpuBuffer* const buffer1,
		ndBrainGpuBuffer* const buffer2);

	ndSharedPtr<ndBrainGpuBuffer> m_uniformBuffer;
	ndBrainLayer::ndCommandShareInfo m_info;
	ndBrainTrainerGpuInference* m_owner;
	size_t m_id;
};

class ndBrainTrainerGpuInference: public ndBrainTrainer
{
	public: 
	ndBrainTrainerGpuInference(
		const ndSharedPtr<ndBrain>& brain, 
		const ndSharedPtr<ndBrainContext>& context, 
		ndInt32 minibatchSize);
	ndBrainTrainerGpuInference(const ndBrainTrainerGpuInference& src);
	virtual ~ndBrainTrainerGpuInference();

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
	ndBrainTrainerGpuCommand* FindCommand(size_t id) const;
	void AddCopyInputCommand(const ndBrainLayer::ndCommandShareInfo& uniformData);
	void UnloadBuffer(ndBrainVector& ouput, const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer) const;
	void AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndCommandShareInfo, 256>& layersUniformsData);

	ndBrainGpuContext* m_context;
	ndSharedPtr<ndBrainContext> m_contextRef;
	ndSharedPtr<ndBrainGpuBuffer> m_inputOutputBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_weightAndBiasBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_miniBatchOutputBuffer;
	//ndList<ndSharedPtr<ndBrainTrainerGpuCommand>> m_feedFowardCommands;
	ndList<ndSharedPtr<ndBrainGpuCommand>> m_feedFowardCommands;
	ndInt32 m_miniBatchSize;
};

#endif 

