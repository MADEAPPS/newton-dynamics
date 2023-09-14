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

#include "ndBrainStdafx.h"
#include "ndBrainTrainer.h"
#include "ndBrainOptimizerSgd.h"

ndBrainOptimizerSgd::ndBrainOptimizerSgd(ndBrainTrainer* const trainer)
	:ndBrainOptimizer(trainer)
{
}

ndBrainOptimizerSgd::~ndBrainOptimizerSgd()
{
}

void ndBrainOptimizerSgd::Update(ndReal learnRate, ndInt32 bashSize)
{
	ndAssert(0);

	//learnRate *= ndReal(-1.0f);
	//ndReal regularizer = -GetRegularizer();
	//const ndArray<ndBrainLayer*>& layers = *m_brain;
	//for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	//{
	//	ndAssert(0);
	//	//ndBrainLayer* const layer = layers[i];
	//	//const ndInt32 inputSize = layer->GetInputSize();
	//	//const ndInt32 outputSize = layer->GetOutputSize();
	//	//
	//	//ndBrainVector& bias = layer->GetBias();
	//	//ndBrainMemVector biasGradients(&m_biasGradientsAcc[m_inputOutputPrefixScan[i + 1]], outputSize);
	//	//biasGradients.Scale(learnRate);
	//	//biasGradients.ScaleAdd(bias, regularizer);
	//	//bias.Add(biasGradients);
	//	//bias.FlushToZero();
	//	//
	//	//const ndInt32 weightGradientStride = (inputSize + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	//	//const ndReal* weightGradientPtr = &m_weightGradients[m_weightGradientsPrefixScan[i]];
	//	//
	//	//ndBrainMatrix& weightMatrix = layer->m_weights;
	//	//for (ndInt32 j = 0; j < outputSize; ++j)
	//	//{
	//	//	ndBrainVector& weightVector = weightMatrix[j];
	//	//	ndBrainMemVector weightGradients(weightGradientPtr, inputSize);
	//	//	weightGradients.Scale(learnRate);
	//	//	weightGradients.ScaleAdd(weightVector, regularizer);
	//	//	weightVector.Add(weightGradients);
	//	//	weightVector.FlushToZero();
	//	//	weightGradientPtr += weightGradientStride;
	//	//}
	//}
}