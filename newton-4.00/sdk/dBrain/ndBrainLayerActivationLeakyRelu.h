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

#ifndef _ND_BRAIN_LAYER_LEAKY_RELU_ACTIVATION_H__
#define _ND_BRAIN_LAYER_LEAKY_RELU_ACTIVATION_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayerActivation.h"

#define ND_LEAKY_LRU_NEG_GRADIENT					ndBrainFloat(0.01f)
#define ND_BRAIN_LAYER_ACTIVATION_LEAKY_RELU_NAME	"ndBrainLayerActivationLeakyRelu"

class ndBrainLayerActivationLeakyRelu : public ndBrainLayerActivation
{
	public:
	ndBrainLayerActivationLeakyRelu(ndInt32 neurons);
	ndBrainLayerActivationLeakyRelu(const ndBrainLayerActivationLeakyRelu& src);
	ndBrainLayer* Clone() const;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	const char* GetLabelId() const;
	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual bool HasGpuSupport() const override;
	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const info, ndInt32 miniBatchIndex) const override;
	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const info, ndInt32 miniBatchIndex) const override;

	virtual ndCommandArray CreateGpuFeedForwardCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const override;

	virtual ndCommandArray CreateGpuBackPropagateCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias,
		ndBrainFloatBuffer* const inputOutputGradients,
		ndBrainFloatBuffer* const weightsAndBiasGradients) const override;
};

#endif 

