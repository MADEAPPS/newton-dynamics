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

#ifndef _ND_BRAIN_TRAINER_INFERENCE_H__
#define _ND_BRAIN_TRAINER_INFERENCE_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainOptimizer.h"
#include "ndBrainContext.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainBufferCommand.h"

class ndBrain;
class ndBrainLoss;
class ndBrainLayer;
class ndBrainFloatBuffer;
class ndBrainUniformBuffer;
class ndBrainTrainerInference;

class ndTrainerDescriptor
{
	public:
	ndTrainerDescriptor();
	ndTrainerDescriptor(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndInt32 minibatchSize, ndBrainFloat learnRate);

	ndSharedPtr<ndBrain> m_brain;
	ndSharedPtr<ndBrainContext> m_context;
	ndBrainFloat m_learnRate;
	ndBrainFloat m_regularizer;
	ndInt32 m_minibatchSize;
	ndRegularizerType m_regularizerType;
};

class ndBrainTrainerInference : public ndClassAlloc
{
	public: 
	ndBrainTrainerInference(const ndTrainerDescriptor& descriptor);
	ndBrainTrainerInference(const ndBrainTrainerInference& src);
	virtual ~ndBrainTrainerInference();

	ndSharedPtr<ndBrain>& GetBrain();
	ndSharedPtr<ndBrainContext> GetContext();

	ndBrainFloatBuffer* GetInputBuffer();
	ndBrainFloatBuffer* GetOuputBuffer();

	ndBrainFloatBuffer* GetHiddenLayerBuffer();
	ndBrainFloatBuffer* GetWeightAndBiasBuffer();

	//virtual const ndBrainBuffer* GetOutputBuffer() override;
	//
	//virtual void LoadInput(const ndBrainVector& input) override;
	//virtual void SaveInput(ndBrainVector& output) const override;
	//
	//virtual void GetOutput(ndBrainVector& output) const override;
	//virtual void GetParameterBuffer(ndBrainVector& output) const override;
	
	
	// new methods
	void MakePrediction();
	//virtual void SyncQueue() override;
	//virtual void ApplyLearnRate() override;
	//
	//virtual void BackPropagate(const ndBrainVector& outputGradients) override;
	//virtual void UpdateParameters(const ndBrainVector& weightAndBias) override;
	//virtual void MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output) override;

	protected:
	enum ndInputOutputCommandId
	{
		m_inputId = 7,
		m_outpuId,
		m_adamOptimizer,
	};

	void AddCopyOutputCommand();
	void InitInputOutputBuffer();
	void InitWeightAndBiasBuffer();
	ndInt32 RoundoffOffset(ndInt32 value) const;
	ndBrainBufferCommand* FindCommand(size_t id) const;
	void AddCopyInputCommand(const ndCommandSharedInfo& uniformData);
	void AddLayersCommands(ndFixSizeArray<ndCommandSharedInfo, 256>& layersUniformsData);

	ndTrainerDescriptor m_descriptor;
	ndSharedPtr<ndBrainFloatBuffer> m_inputOutputBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_weightAndBiasBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainFloatBuffer> m_miniBatchOutputBuffer;

	//ndSharedPtr<ndBrainFloatBuffer> m_singlePredictionInputBuffer;
	//ndSharedPtr<ndBrainFloatBuffer> m_singlePredictionOutputBuffer;
	//ndSharedPtr<ndBrainUniformBuffer> m_singlePredictionInputBufferParameters;
	//ndSharedPtr<ndBrainUniformBuffer> m_singlePredictionOutputBufferParameters;

	ndList<ndSharedPtr<ndBrainBufferCommand>> m_feedForwardCommands;
};

#endif 

