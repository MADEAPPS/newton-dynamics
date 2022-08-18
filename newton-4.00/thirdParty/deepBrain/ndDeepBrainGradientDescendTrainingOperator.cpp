/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrain.h"
#include "ndDeepBrainLayer.h"
#include "ndDeepBrainGradientDescendTrainingOperator.h"

ndDeepBrainGradientDescendTrainingOperator::ndDeepBrainGradientDescendTrainingOperator(ndDeepBrain* const brain)
	:ndDeepBrainTrainingOperator(brain)
{
}

ndDeepBrainGradientDescendTrainingOperator::ndDeepBrainGradientDescendTrainingOperator(const ndDeepBrainGradientDescendTrainingOperator& src)
	:ndDeepBrainTrainingOperator(src)
{
}

ndDeepBrainGradientDescendTrainingOperator::~ndDeepBrainGradientDescendTrainingOperator()
{
}

void ndDeepBrainGradientDescendTrainingOperator::Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	ndInt32 miniBatchsize = 2000;
	ndAssert(inputBatch.GetCount() == groundTruth.GetCount());
	ndAssert(m_output.GetCount() == groundTruth[0].GetCount());

	ndInt32 index = 0;
	ndInt32 batchCount = (inputBatch.GetCount() + miniBatchsize - 1) / miniBatchsize;
	for (ndInt32 i = 0; i < steps; ++i)
	{
		const ndInt32 batchStart = index * miniBatchsize;
		const ndInt32 batchSize = index != (batchCount - 1) ? miniBatchsize : inputBatch.GetCount() - batchStart;
		index = (index + 1) % batchCount;

		m_averageError = 0.0f;
		//for (ndInt32 j = inputBatch.GetCount() - 1; j >= 0; --j)
		for (ndInt32 j = 0; j < batchSize; ++j)
		{
			const ndDeepBrainVector& input = inputBatch[batchStart + j];
			const ndDeepBrainVector& truth = groundTruth[batchStart + j];
			MakePrediction(input);
			BackPropagate(truth);
			UpdateWeights(learnRate);
			ndFloat32 error = CalculateMeanSquareError(truth);
			m_averageError += error;
		}
		ApplyWeightTranspose();
		//m_averageError = ndSqrt(m_averageError / inputBatch.GetCount());
		m_averageError = ndSqrt(m_averageError / batchSize);
		ndExpandTraceMessage("%f\n", m_averageError);
	}
}