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

class ndBrain;
class ndBrainLoss;
class ndBrainLayer;
class ndBrainThreadPool;

class ndBrainTrainerCpuInference: public ndBrainTrainer
{
	public: 
	ndBrainTrainerCpuInference(const ndSharedPtr<ndBrain>& brain, ndBrainThreadPool* const threadPool, ndInt32 minibatchSize);
	ndBrainTrainerCpuInference(const ndBrainTrainerCpuInference& src);
	virtual ~ndBrainTrainerCpuInference();

	virtual void MakePrediction(const ndBrainVector& input) override;

	// legacy method;
	virtual void BackPropagate(const ndBrainVector&, ndBrainLoss&) override { ndAssert(0);}

	// new method
	virtual void BackPropagate(const ndBrainVector& outputGradients) override;

	//virtual void MakePrediction(const ndBrainVector& input) override;
	//virtual void BackPropagate(const ndBrainVector& input, ndBrainLoss& loss) override;
	//
	//virtual void BackPropagate(const ndBrainVector& input, const ndBrainVector& groundTruth) override;
	//
	//void AcculumateGradients(const ndBrainTrainerCpuInference& src, ndInt32 index);
	//void CalculateInputGradient(const ndBrainVector& input, ndBrainVector& inputGradientsOut, ndBrainLoss& loss);
	//ndBrainLayer* GetWeightsLayer(ndInt32 index) const;
	//ndBrainLayer* GetGradientLayer(ndInt32 index) const;
	//
	//void ClearGradients();
	//void ScaleWeights(const ndBrainFloat s);
	//void AddGradients(const ndBrainTrainerCpuInference* const src);
	//void CopyGradients(const ndBrainTrainerCpuInference* const src);
	//
	//ndBrainVector& GetWorkingBuffer();

	protected:
	void InitInputOutputBuffer();
	void InitWeightAndBiasBuffer();
	void AddCopyInputCommand(const ndBrainLayer::ndLayerUniformDataCpu* const uniformData);

	ndBrainVector m_inputOutputBuffer;
	ndBrainVector m_weightAndBiasBuffer;
	ndBrainVector m_miniBatchInputBuffer;
	ndBrainVector m_miniBatchOutputBuffer;
	//ndList<ndSharedPtr<ndUniformParameters>> m_uniforms;
	//ndList<ndSharedPtr<ndBrainGpuCommand>> m_commandBuffers;

	ndBrainThreadPool* m_threadPool;

	ndInt32 m_miniBatchSize;
};

#endif 

