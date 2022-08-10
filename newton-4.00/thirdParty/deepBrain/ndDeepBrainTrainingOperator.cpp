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
#include "ndDeepBrainTrainingOperator.h"

ndDeepBrainTrainingOperator::ndDeepBrainTrainingOperator(ndDeepBrain* const brain)
	:ndClassAlloc()
	,m_instance(brain)
	,m_cost()
	,m_output()
	,m_gradient()
	,m_ouputPrefixScan()
	,m_gradientPrefixScan()
{
}

ndDeepBrainTrainingOperator::~ndDeepBrainTrainingOperator()
{
}

void ndDeepBrainTrainingOperator::InitGaussianWeights(ndReal mean, ndReal variance)
{
	ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		layers[i]->InitGaussianWeights(mean, variance);
	}
}

void ndDeepBrainTrainingOperator::PrefixScan()
{
	const ndArray<ndDeepBrainLayer*>& layers = m_instance.GetLayers();
	m_ouputPrefixScan.SetCount(layers.GetCount() + 1);
	m_ouputPrefixScan[0] = (layers[0]->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		ndInt32 size = (layer->GetOuputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_ouputPrefixScan[i + 1] = size;
	}
	
	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_ouputPrefixScan.GetCount(); ++i)
	{
		ndInt32 size = m_ouputPrefixScan[i];
		m_ouputPrefixScan[i] = sum;
		sum += size;
	}
	m_output.SetCount(sum);
	m_output.SetValue(0.0f);

	m_gradientPrefixScan.SetCount(layers.GetCount() + 1);
	m_gradientPrefixScan[layers.GetCount()] = (layers[layers.GetCount() - 1]->GetOuputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainLayer* const layer = layers[i];
		ndInt32 size = (layer->GetInputSize() + 1 + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		m_gradientPrefixScan[i] = size * layer->GetOuputSize();
	}

	sum = 0;
	for (ndInt32 i = 0; i < m_gradientPrefixScan.GetCount(); ++i)
	{
		ndInt32 size = m_gradientPrefixScan[i];
		m_gradientPrefixScan[i] = sum;
		sum += size;
	}
	m_gradient.SetCount(sum);
	m_gradient.SetValue(0.0f);
}

void ndDeepBrainTrainingOperator::BackwardPass()
{
	//for (ndInt32 i = m_layers.GetCount()-1; i >= 0; --i)
	//{
	//	m_layers[i]->FowardPass(m_inputs, m_outputs);
	//	m_inputs.Swap(m_outputs);
	//}
	//m_inputs.Swap(m_outputs);
}

