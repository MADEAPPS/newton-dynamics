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
#include "ndDeepBrainInstance.h"

ndDeepBrainInstance::ndDeepBrainInstance(ndDeepBrain* const brain)
	:ndClassAlloc()
	,m_inputs()
	,m_outputs()
	,m_brain(brain)
{
}

ndDeepBrainInstance::~ndDeepBrainInstance()
{
}

ndDeepBrainVector& ndDeepBrainInstance::GetInputs()
{
	return m_inputs;
}

ndDeepBrainVector& ndDeepBrainInstance::GetOutputs()
{
	return m_outputs;
}

ndArray<ndDeepBrainLayer*>& ndDeepBrainInstance::GetLayers()
{
	return *m_brain;
}

const ndArray<ndDeepBrainLayer*>& ndDeepBrainInstance::GetLayers() const
{
	return *m_brain;
}

void ndDeepBrainInstance::SetInput(const ndDeepBrainVector& input)
{
	ndDeepBrainLayer* const inputLayer = (*m_brain)[0];
	ndInt32 size = inputLayer->GetInputSize();
	dAssert(size == input.GetCount());

	m_inputs.SetCount(size);
	m_inputs.CopyData(input);
}

void ndDeepBrainInstance::MakePrediction(const ndDeepBrainVector& input)
{
	dAssert(0);
	SetInput(input);
	ndArray<ndDeepBrainLayer*>& layers = (*m_brain);
	dAssert(layers.GetCount());

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		dAssert(m_inputs.GetCount() == layer->GetInputSize());

		ndInt32 neuronsCount = layer->GetOuputSize();
		m_outputs.SetCount(neuronsCount);
		layer->MakePrediction(m_inputs, m_outputs);
		m_inputs.Swap(m_outputs);
	}
	m_inputs.Swap(m_outputs);
}

void ndDeepBrainInstance::MakeTrainingPrediction(const ndDeepBrainVector& input, ndDeepBrainVector& output, const ndDeepBrainPrefixScan& prefixSum)
{
	SetInput(input);
	ndArray<ndDeepBrainLayer*>& layers = (*m_brain);
	dAssert(layers.GetCount());

	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		dAssert(m_inputs.GetCount() == layer->GetInputSize());
		
		ndInt32 outputCount = layer->GetOuputSize();
		m_outputs.SetCount(outputCount);
		layer->MakePrediction(m_inputs, m_outputs);
		ndInt32 start = prefixSum[i];
		for (ndInt32 j = outputCount - 1; j >= 0; --j)
		{
			output[start + j] = m_outputs[j];
		}
		m_inputs.Swap(m_outputs);
	}
	m_inputs.Swap(m_outputs);
}

void ndDeepBrainInstance::BackPropagate(const ndDeepBrainVector& leastSquareError, const ndDeepBrainVector& output, const ndDeepBrainPrefixScan& prefixSum)
{
	ndArray<ndDeepBrainLayer*>& layers = (*m_brain);
	dAssert(layers.GetCount());
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
	}
}