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
#include "ndBrain.h"
#include "ndBrainMatrix.h"
#include "ndBrainThreadPool.h"
#include "ndBrainOptimizerSgd.h"

ndBrainOptimizerSgd::ndBrainOptimizerSgd()
	:ndBrainOptimizer(ndSharedPtr<ndBrainContext>())
{
}

ndBrainOptimizerSgd::~ndBrainOptimizerSgd()
{
}

#if 0
//void ndBrainOptimizerSgd::Update(ndBrainThreadPool* const threadPool, ndArray<ndBrainTrainer*>& partialGradients, ndBrainFloat learnRate)
//void ndBrainOptimizerSgd::Update(ndBrainVector& parameters, const ndBrainVector& gradients, ndBrainFloat learnRate)
void ndBrainOptimizerSgd::Update(ndBrainVector&, const ndBrainVector&)
{
	ndAssert(0);
	ndBrainTrainerCpuLegacy* const trainer = (ndBrainTrainerCpuLegacy*)partialGradients[0];
	ndBrain& brain = **trainer->GetBrain();
	
	ndFixSizeArray<ndInt32, 256> paramLayer;
	for (ndInt32 i = 0; i < brain.GetCount(); ++i)
	{
		if (brain[i]->HasParameters())
		{
			paramLayer.PushBack(i);
		}
	}
	
	auto UpdateGradients = ndMakeObject::ndFunction([this, learnRate, &paramLayer, &partialGradients](ndInt32 threadIndex, ndInt32 threadCount)
	{
		//ndBrainTrainer* const trainer = partialGradients[0];
		ndBrainTrainerCpuLegacy* const trainer = (ndBrainTrainerCpuLegacy*)partialGradients[0];
		ndBrainFloat regularizer = -GetRegularizer();
		ndBrainFloat descendRate = -learnRate / ndBrainFloat(partialGradients.GetCount());
	
		const ndStartEnd startEnd(paramLayer.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndInt32 index = paramLayer[i];
			ndAssert((**trainer->GetBrain())[index]->HasParameters());
			
			ndBrainLayer& weights = *trainer->GetWeightsLayer(index);
			ndBrainLayer& gradients = *trainer->GetGradientLayer(index);
	
			for (ndInt32 j = 1; j < partialGradients.GetCount(); ++j)
			{
				ndBrainTrainerCpuLegacy* const src = (ndBrainTrainerCpuLegacy*)partialGradients[j];
				trainer->AcculumateGradients(*src, index);
			}
			
			gradients.Scale(descendRate);
			gradients.ScaleAdd(weights, regularizer);
			weights.Add(gradients);
			weights.FlushToZero();
		}
	});
	threadPool->ndBrainThreadPool::ParallelExecute(UpdateGradients);
}
#endif