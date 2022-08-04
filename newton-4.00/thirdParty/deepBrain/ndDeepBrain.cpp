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
#include "ndDeepBrainNeuron.h"

ndDeepBrain::ndDeepBrain()
	:ndClassAlloc()
	,m_inputs()
	,m_outputs()
	,m_layers()
{
}

ndDeepBrain::~ndDeepBrain()
{
	for (ndInt32 i = 0; i < m_layers.GetCount(); ++i)
	{
		delete m_layers[i];
	}
}

ndDeepBrainVector& ndDeepBrain::GetInputs()
{
	return m_inputs;
}

ndDeepBrainVector& ndDeepBrain::GetOutputs()
{
	return m_outputs;
}

ndArray<ndDeepBrainLayer*>& ndDeepBrain::GetLayers()
{
	return m_layers;
}

void ndDeepBrain::AddLayer(ndDeepBrainLayer* const layer)
{
	dAssert(!m_layers.GetCount() || (m_layers[m_layers.GetCount() - 1]->GetNeurons().GetCount() == layer->GetInputSize()));
	m_inputs.SetCount(ndMax (layer->GetInputSize(), m_inputs.GetCount()));
	m_outputs.SetCount(m_inputs.GetCount());
	m_inputs.SetValue(0.0f);
	m_outputs.SetValue(0.0f);
	m_layers.PushBack(layer);
}

void ndDeepBrain::AddLayer(ndInt32 inputs, ndInt32 outputs, ndDeepBrainLayer::ActivationType type)
{
	AddLayer (new ndDeepBrainLayer(inputs, outputs, type));
}

void ndDeepBrain::FowardPass()
{
	for (ndInt32 i = 0; i < m_layers.GetCount(); ++i)
	{
		m_layers[i]->FowardPass(m_inputs, m_outputs);
		m_inputs.Swap(m_outputs);
	}
	m_inputs.Swap(m_outputs);
}
