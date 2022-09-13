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
#include "ndDeepBrainTrainerBase.h"


ndDeepBrainTrainerBase::ndDeepBrainTrainerBase(ndDeepBrain* const brain)
	:ndClassAlloc()
	,m_instance(brain)
	,m_miniBatchSize(1)
{
}

ndDeepBrainTrainerBase::ndDeepBrainTrainerBase(const ndDeepBrainTrainerBase& src)
	:ndClassAlloc()
	,m_instance(src.m_instance)
	,m_miniBatchSize(src.m_miniBatchSize)
{
}

ndDeepBrainTrainerBase::~ndDeepBrainTrainerBase()
{
}

void ndDeepBrainTrainerBase::SetMiniBatchSize(ndInt32 miniBatchSize)
{
	m_miniBatchSize = miniBatchSize;
}

ndReal ndDeepBrainTrainerBase::ndValidation::Validate(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth)
{
	ndReal error2 = 0;
	ndDeepBrainInstance& instance = m_trainer.GetInstance();
	for (ndInt32 i = inputBatch.GetCount() - 1; i >= 0; --i)
	{
		const ndDeepBrainVector& input = inputBatch[i];
		const ndDeepBrainVector& truth = groundTruth[i];
		instance.MakePrediction(input, m_output);
		for (ndInt32 j = m_output.GetCount() - 1; j >= 0; --j)
		{
			ndFloat32 dist = m_output[j] - truth[j];
			error2 += dist * dist;
		}
	}
	ndReal error = ndSqrt (error2 / inputBatch.GetCount());
	return error;
}
