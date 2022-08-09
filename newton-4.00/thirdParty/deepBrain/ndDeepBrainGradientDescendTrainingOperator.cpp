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

ndDeepBrainGradientDescendTrainingOperator::~ndDeepBrainGradientDescendTrainingOperator()
{
}

void ndDeepBrainGradientDescendTrainingOperator::Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps)
{
	dAssert(inputBatch.GetCount() == groundTruth.GetCount());

	PrefixScan();

	ndArray<ndDeepBrainLayer*>& layers = *m_instance.m_brain;
	m_cost.SetCount(layers[layers.GetCount() - 1]->GetOuputSize());
	for (ndInt32 i = 0; i < steps; ++i)
	{
		for (ndInt32 j = inputBatch.GetCount() - 1; j >= 0; --j)
		{
			const ndDeepBrainVector& input = inputBatch[j];
			const ndDeepBrainVector& truth = groundTruth[j];
			m_instance.MakeTrainingPrediction(input, *this);
			m_cost.Sub(truth, m_instance.GetOutputs());
			m_instance.BackPropagate(*this);
		}
	}
}