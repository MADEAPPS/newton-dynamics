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

class ndBrain;
class ndBrainLoss;
class ndBrainLayer;

class ndBrainTrainnerGpuInference: public ndBrainTrainer
{
	public: 
	class ndGpuCommand;
	class ndUniformBufferObject;

	ndBrainTrainnerGpuInference(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainGpuContext>& context, ndInt32 minibatchSize);
	ndBrainTrainnerGpuInference(const ndBrainTrainnerGpuInference& src);
	virtual ~ndBrainTrainnerGpuInference();

	virtual void GetInput(ndBrainVector& ouput) const;
	virtual void GetOutput(ndBrainVector& ouput) const;
	virtual void GetWorkingBuffer(ndBrainVector& ouput) const;
	virtual void GetParameterBuffer(ndBrainVector& ouput) const;

	// legacy
	virtual void BackPropagate(const ndBrainVector& input, ndBrainLoss& loss) override { ndAssert(0); }

	virtual void MakePrediction(const ndBrainVector& input) override;
	virtual void BackPropagate(const ndBrainVector& outputGradients) override;

	protected:
	void SubmitCommands();
	void AddCopyOutputCommand();
	void InitInputOutputBuffer();
	void InitWeightAndBiasBuffer();
	void AddCopyInputCommand(const ndBrainLayer::ndLayerUniformDataGpu& uniformData);
	void UnloadBuffer(ndBrainVector& ouput, const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer) const;
	void AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndLayerUniformDataGpu, 256>& layersUniformsData);

	ndSharedPtr<ndBrainGpuContext> m_context;
	ndList<ndSharedPtr<ndBrainGpuBuffer>> m_uniforms;
	ndSharedPtr<ndBrainGpuBuffer> m_inputOutputBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_weightAndBiasBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainGpuBuffer> m_miniBatchOutputBuffer;
	ndList<ndSharedPtr<ndBrainGpuCommand>> m_commandBuffers;
	ndInt32 m_miniBatchSize;
};

#endif 

