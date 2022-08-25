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

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrain.h"
#include "ndDeepBrainLayer.h"
#include "ndDeepBrainTrainingOperator.h"

ndDeepBrainTrainingOperator::ndDeepBrainTrainingOperator(ndDeepBrain* const brain)
	:ndClassAlloc()
	,m_instance(brain)
	,m_miniBatchSize(100000)
{
}

ndDeepBrainTrainingOperator::ndDeepBrainTrainingOperator(const ndDeepBrainTrainingOperator& src)
	:ndClassAlloc()
	,m_instance(src.m_instance)
	,m_miniBatchSize(src.m_miniBatchSize)
{
}

ndDeepBrainTrainingOperator::~ndDeepBrainTrainingOperator()
{
}

void ndDeepBrainTrainingOperator::SetMiniBatchSize(ndInt32 miniBatchSize)
{
	m_miniBatchSize = miniBatchSize;
}

ndFloat32 ndDeepBrainTrainingOperator::CalculateMeanSquareError(const ndDeepBrainVector& groundTruth) const
{
	const ndArray<ndDeepBrainLayer*>& layers = (*m_instance.GetBrain());
	const ndInt32 layerIndex = layers.GetCount() - 1;

	ndDeepBrainLayer* const ouputLayer = layers[layerIndex];
	const ndInt32 outputCount = ouputLayer->GetOuputSize();
	const ndDeepBrainMemVector z(&m_instance.GetOutPut()[m_instance.GetPrefixScan()[layerIndex + 1]], outputCount);

	ndFloat32 error2 = 0;
	for (ndInt32 i = 0; i < outputCount; i++)
	{
		ndFloat32 dist = z[i] - groundTruth[i];
		error2 += dist * dist;
	}
	return error2 / outputCount;
}

