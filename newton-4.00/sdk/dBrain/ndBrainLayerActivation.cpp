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
#include "ndBrainLayerActivation.h"

ndBrainLayerActivation::ndBrainLayerActivation(ndInt32 neurons)
	:ndBrainLayer()
	,m_neurons(neurons)
{
}

ndBrainLayerActivation::ndBrainLayerActivation(const ndBrainLayerActivation& src)
	:ndBrainLayer(src)
	,m_neurons(src.m_neurons)
{
	ndAssert(0);
}

ndBrainLayerActivation::~ndBrainLayerActivation()
{
	ndAssert(0);
}

bool ndBrainLayerActivation::HasParameters() const
{
	return false;
}

const char* ndBrainLayerActivation::GetLabelId() const
{
	return "ndBrainLayerActivation";
}

ndBrainLayer* ndBrainLayerActivation::Clone() const
{
	ndAssert(0);
	return nullptr;
}

ndInt32 ndBrainLayerActivation::GetOutputSize() const
{
	return m_neurons;
}

ndInt32 ndBrainLayerActivation::GetInputSize() const
{
	return m_neurons;
}

void ndBrainLayerActivation::InitWeightsXavierMethod()
{
}

void ndBrainLayerActivation::InitWeights(ndReal, ndReal)
{
}

void ndBrainLayerActivation::ClearGradAcc(ndBrainVector&, ndBrainMatrix&) const
{
}

void ndBrainLayerActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == m_neurons);
	ndAssert(output.GetCount() == m_neurons);
	output.Set(input);
}

void ndBrainLayerActivation::ActivationDerivative(const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayerActivation::CalculateOutputLayersParamGradients(const ndBrainVector&, const ndBrainVector&, ndBrainVector&, ndBrainMatrix&) const
{
	ndAssert(0);
}

void ndBrainLayerActivation::InputDerivative(const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}
