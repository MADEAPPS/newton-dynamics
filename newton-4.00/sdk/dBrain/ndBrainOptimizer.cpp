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
#include "ndBrainTrainer.h"
#include "ndBrainOptimizer.h"
#include "ndBrainThreadPool.h"

ndBrainOptimizer::ndBrainOptimizer(const ndSharedPtr<ndBrainContext>& context)
	:ndClassAlloc()
	,m_context(context)
	,m_learnRate(ndBrainFloat(1.0e-4f))
	,m_weighDecayRegularizer(ndBrainFloat(0.0f))
	,m_regularizerType(m_ridge)
	,m_commands()
{
}

ndBrainOptimizer::~ndBrainOptimizer()
{
}

ndRegularizerType ndBrainOptimizer::GetRegularizerType() const
{
	return m_regularizerType;
}

void ndBrainOptimizer::SetRegularizerType(ndRegularizerType type)
{
	m_regularizerType = type;
}

ndBrainFloat ndBrainOptimizer::GetRegularizer() const
{
	return m_weighDecayRegularizer;
}

void ndBrainOptimizer::SetRegularizer(ndBrainFloat regularizer)
{
	m_weighDecayRegularizer = ndClamp(regularizer, ndBrainFloat(0.0f), ndBrainFloat(0.01f));
}

#if 0
void ndBrainOptimizer::Update(ndBrainThreadPool* const, ndArray<ndBrainTrainer*>&, ndBrainFloat)
{
	ndAssert(0);
}

//void ndBrainOptimizer::AccumulateGradients(ndBrainThreadPool* const threadPool, ndArray<ndBrainTrainer*>& partialGradients) const
void ndBrainOptimizer::AccumulateGradients(ndBrainThreadPool* const, ndArray<ndBrainTrainer*>&) const
{
	ndAssert(0);
	//ndBrainTrainerCpuLegacy* const trainer0 = (ndBrainTrainerCpuLegacy*)partialGradients[0];
	//const ndBrain& brain = **trainer0->GetBrain();
	//
	//ndFixSizeArray<ndInt32, 256> paramLayer;
	//for (ndInt32 i = 0; i < brain.GetCount(); ++i)
	//{
	//	if (brain[i]->HasParameters())
	//	{
	//		paramLayer.PushBack(i);
	//	}
	//}
	//
	//auto AddGradients = ndMakeObject::ndFunction([this, &paramLayer, &partialGradients](ndInt32 threadIndex, ndInt32 threadCount)
	//{
	//	ndBrainTrainerCpuLegacy* const dst = (ndBrainTrainerCpuLegacy*)partialGradients[0];
	//	const ndStartEnd startEnd(paramLayer.GetCount(), threadIndex, threadCount);
	//	for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
	//	{
	//		ndInt32 index = paramLayer[i];
	//		for (ndInt32 j = 1; j < partialGradients.GetCount(); ++j)
	//		{
	//			ndBrainTrainerCpuLegacy* const src = (ndBrainTrainerCpuLegacy*)partialGradients[j];
	//			dst->AcculumateGradients(*src, index);
	//		}
	//	}
	//});
	//threadPool->ndBrainThreadPool::ParallelExecute(AddGradients);
}
#endif
