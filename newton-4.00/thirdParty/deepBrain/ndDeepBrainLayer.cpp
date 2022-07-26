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
#include "ndDeepBrainLayer.h"

ndDeepBrainLayer::ndDeepBrainLayer(ndInt32 inputCount, ndInt32 outputCount, ActivationType type)
	:ndClassAlloc()
	,m_type(type)
	,m_ouputs()
	,m_neurons()
{
	m_ouputs.SetCount(outputCount);
	for (ndInt32 i = 0; i < outputCount; ++i)
	{
		ndDeepBrainNeuron* const neuron = new ndDeepBrainNeuron(inputCount);
		m_neurons.PushBack(neuron);
	}
}

ndDeepBrainLayer::~ndDeepBrainLayer()
{
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		dAssert(0);
		delete m_neurons[i];
	}
}

ndInt32 ndDeepBrainLayer::GetInputSize() const
{
	ndDeepBrainNeuron* const neuron = m_neurons[0];
	return neuron->GetWeights().GetCount();
}

ndArray<ndDeepBrainNeuron*>& ndDeepBrainLayer::GetNeurons()
{
	return m_neurons;
}