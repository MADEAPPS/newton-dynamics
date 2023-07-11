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
#include "ndBrainLayer.h"
#include "ndBrainTrainerBase.h"


ndBrainTrainerBase::ndBrainTrainerBase(ndBrain* const brain)
	:ndClassAlloc()
	,m_brain(brain)
	,m_miniBatchSize(1)
	,m_model(m_adam)
{
}

ndBrainTrainerBase::ndBrainTrainerBase(const ndBrainTrainerBase& src)
	:ndClassAlloc()
	,m_brain(src.m_brain)
	,m_miniBatchSize(src.m_miniBatchSize)
	,m_model(src.m_model)
{
}

ndBrainTrainerBase::~ndBrainTrainerBase()
{
}

void ndBrainTrainerBase::SetMiniBatchSize(ndInt32 miniBatchSize)
{
	m_miniBatchSize = miniBatchSize;
}

//ndReal ndBrainTrainerBase::Validate(const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndBrainVector& output)
ndReal ndBrainTrainerBase::Validate(const ndBrainMatrix&, const ndBrainMatrix&, ndBrainVector&)
{
	ndAssert(0);
	//ndReal error2 = 0;
	//for (ndInt32 i = inputBatch.GetCount() - 1; i >= 0; --i)
	//{
	//	const ndBrainVector& input = inputBatch[i];
	//	const ndBrainVector& truth = groundTruth[i];
	//	m_instance.MakePrediction(input, output);
	//
	//	//output.Sub(output, truth);
	//	output.Sub(truth);
	//	error2 += output.Dot(output);
	//}
	//ndReal error = ndReal(ndSqrt(error2 / ndReal(inputBatch.GetCount())));
	//return error;
	return 0;
}

//ndReal ndBrainTrainerBase::ndValidation::Validate(const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth)
ndReal ndBrainTrainerBase::ndValidation::Validate(const ndBrainMatrix&)
{
	ndAssert(0);
	return 0;
	//return m_trainer.Validate(inputBatch, groundTruth, m_output);
}

