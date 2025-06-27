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
	ndBrainTrainerGpuCommand(
		ndBrainTrainerGpuInference* const owner,
		const ndBrainLayer::ndCommandShareInfo& info, 
		size_t id,
		ndBrainGpuContext* const context,
		const ndSharedPtr<ndBrainGpuShader>& shader,
		ndInt32 numberOfinputs,
		const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer,
		ndBrainGpuFloatBuffer* const inputOutputData,
		ndBrainGpuFloatBuffer* const parameters,
		ndBrainGpuFloatBuffer* const inputOutputGradients = nullptr,
		ndBrainGpuFloatBuffer* const weightsAndBiasGradients = nullptr);

	ndSharedPtr<ndBrainGpuUniformBuffer> m_uniformBuffer;
	ndBrainTrainerGpuInference* m_owner;
	size_t m_id;
};

class ndBrainTrainerGpuInference: public ndBrainTrainer
{
	public: 
	ndBrainTrainerGpuInference(const ndTrainerDescriptor& descriptor);
	ndBrainTrainerGpuInference(
		const ndSharedPtr<ndBrain>& brain, 
		const ndSharedPtr<ndBrainContext>& context, 
		ndInt32 minibatchSize);
	ndBrainTrainerGpuInference(const ndBrainTrainerGpuInference& src);
	virtual ~ndBrainTrainerGpuInference();

	virtual ndBrainBuffer* GetInputBuffer() override;
	virtual const ndBrainBuffer* GetOutputBuffer() override;

	virtual void LoadInput(const ndBrainVector& input) override;
	virtual void SaveInput(ndBrainVector& output) const override;

	virtual void GetOutput(ndBrainVector& output) const override;
	virtual void GetWorkingBuffer(ndBrainVector& output) const override;
	virtual void GetParameterBuffer(ndBrainVector& output) const override;

	// legacy
	virtual void BackPropagate(const ndBrainVector&, ndBrainLoss&) override { ndAssert(0); }

	// new methods
	virtual void SyncQueue() override;
	virtual void ApplyLearnRate() override;
	virtual void MakePrediction() override;
	virtual void BackPropagate(const ndBrainVector& outputGradients) override;
	virtual void UpdateParameters(const ndBrainVector& weightAndBias) override;
	virtual void MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output) override;

	protected:
	enum ndInputOutputCommandId
	{
		m_inputId = 7,
		m_outpuId,
	};

	void AddCopyOutputCommand();
	void InitInputOutputBuffer();
	void InitWeightAndBiasBuffer();
	ndInt32 RoundoffOffset(ndInt32 value) const;
	void Initialize(const ndTrainerDescriptor& descriptor);
	ndBrainTrainerGpuCommand* FindCommand(size_t id) const;
	void AddCopyInputCommand(const ndBrainLayer::ndCommandShareInfo& uniformData);
	void AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndCommandShareInfo, 256>& layersUniformsData);

	ndSharedPtr<ndBrainGpuFloatBuffer> m_inputOutputBuffer;
	ndSharedPtr<ndBrainGpuFloatBuffer> m_weightAndBiasBuffer;
	ndSharedPtr<ndBrainGpuFloatBuffer> m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainGpuFloatBuffer> m_miniBatchOutputBuffer;

	ndSharedPtr<ndBrainGpuFloatBuffer> m_singlePredictionInputBuffer;
	ndSharedPtr<ndBrainGpuUniformBuffer> m_singlePredictionInputBufferParameters;

	ndList<ndSharedPtr<ndBrainGpuCommand>> m_feedForwardCommands;
	ndInt32 m_miniBatchSize;
};

#endif 

