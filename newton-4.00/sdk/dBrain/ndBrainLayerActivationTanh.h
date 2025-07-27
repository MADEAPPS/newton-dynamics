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

#ifndef _ND_BRAIN_LAYER_TANH_ACTIVATION_H__
#define _ND_BRAIN_LAYER_TANH_ACTIVATION_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayerActivation.h"

typedef ndVector ndBrainVector4;

#define ND_BRAIN_LAYER_ACTIVATION_TANGH_NAME	"ndBrainLayerActivationTanh"

class ndBrainLayerActivationTanh : public ndBrainLayerActivation
{
	public:
	ndBrainLayerActivationTanh(ndInt32 neurons);
	ndBrainLayerActivationTanh(const ndBrainLayerActivationTanh& src);
	ndBrainLayer* Clone() const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	const char* GetLabelId() const override;
	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;

	virtual bool HasGpuSupport() const override;
	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const override;
	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const override;

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

