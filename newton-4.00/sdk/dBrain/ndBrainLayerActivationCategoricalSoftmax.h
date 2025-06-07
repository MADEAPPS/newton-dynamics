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

#ifndef _ND_BRAIN_LAYER_CATEGORICAL_SOFTMAX_ACTIVATION_H__
#define _ND_BRAIN_LAYER_CATEGORICAL_SOFTMAX_ACTIVATION_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayerActivationSoftmax.h"


// note: Categorical SoftMax activation layer is designed you work with a 
// Categorical entropy loss. These rules for using it are:
// 1- can only be used as the last layer of the deep neural net is SoftMax layer.
// 2- the loss function returns a one hat encoding as the ground truth.  
// 
// This activation makes use of the knowledge that only one of the elements 
// of a one hat encoding truth is always 1 and the rest are always 0, 
// which greatly simplify the calculation of the derivative. 
class ndBrainLayerActivationCategoricalSoftmax : public ndBrainLayerActivationSoftmax
{
	public:
	ndBrainLayerActivationCategoricalSoftmax(ndInt32 neurons);
	ndBrainLayerActivationCategoricalSoftmax(const ndBrainLayerActivationCategoricalSoftmax& src);
	ndBrainLayer* Clone() const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	const char* GetLabelId() const override;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;

	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const info, ndInt32 miniBatchIndex) const override;

	virtual ndBrainTrainerGpuCommand* CreateGpuBackPropagateCommand(ndBrainTrainerGpuInference* const owner,
		const ndBrainLayer::ndCommandShareInfo& info,
		ndBrainGpuContext* const context, ndInt32 miniBatchSize,
		const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
		ndBrainGpuBuffer* const inputOutputData,
		ndBrainGpuBuffer* const parameters, 
		ndBrainGpuBuffer* const inputOutputGradients) const override;
};

#endif 

