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
#include "ndBrainSaveLoad.h"
#include "ndBrainLayerLinearWithDropOut.h"

ndBrainLayerLinearWithDropOut::ndBrainLayerLinearWithDropOut(ndInt32 inputs, ndInt32 outputs, ndBrainFloat dropOutFactor)
	:ndBrainLayerLinear(inputs, outputs)
	,m_dropout()
	,m_dropoutFactor(dropOutFactor)
	,m_dropoutScale(ndBrainFloat(1.0f))
	,m_droutOutEnable(true)
{
	ndAssert(dropOutFactor >= ndBrainFloat(0.5f));
	ndAssert(dropOutFactor <= ndBrainFloat(1.0f));
	m_dropout.SetCount(outputs);
	UpdateDropOut();
}

ndBrainLayerLinearWithDropOut::ndBrainLayerLinearWithDropOut(const ndBrainLayerLinearWithDropOut& src)
	:ndBrainLayerLinear(src)
	,m_dropout(src.m_dropout)
	,m_dropoutFactor(src.m_dropoutFactor)
	,m_dropoutScale(src.m_dropoutScale)
	,m_droutOutEnable(src.m_droutOutEnable)
{
}

ndBrainLayerLinearWithDropOut::~ndBrainLayerLinearWithDropOut()
{
}

const char* ndBrainLayerLinearWithDropOut::GetLabelId() const
{
	//return "ndBrainLayerLinearWithDropOut";
	return "ndBrainLayerLinear";
}

ndBrainLayer* ndBrainLayerLinearWithDropOut::Clone() const
{
	return new ndBrainLayerLinearWithDropOut(*this);
}

void ndBrainLayerLinearWithDropOut::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerLinear::Save(loadSave);
}

ndBrainLayer* ndBrainLayerLinearWithDropOut::Load(const ndBrainLoad* const)
{
	ndAssert(0);
	return nullptr;
}

void ndBrainLayerLinearWithDropOut::EnableDropOut(bool state)
{
	m_droutOutEnable = state;
}

void ndBrainLayerLinearWithDropOut::UpdateDropOut()
{
	ndInt32 activeCount = 0;
	for (ndInt32 i = ndInt32(m_dropout.GetCount()-1); i >= 0; --i)
	{
		ndInt32 active = (ndRand() <= m_dropoutFactor);
		m_dropout[i] = active ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		activeCount += active;
	}

	ndAssert(activeCount > 0);
	m_dropoutScale = ndBrainFloat (m_dropout.GetCount()) / ndBrainFloat (activeCount);
	m_dropout.Scale(m_dropoutScale);
}

void ndBrainLayerLinearWithDropOut::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(output.GetCount() == m_dropout.GetCount());
	ndBrainLayerLinear::MakePrediction(input, output);
	if (m_droutOutEnable)
	{
		output.Mul(m_dropout);
	}
}

//void ndBrainLayerLinearWithDropOut::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
void ndBrainLayerLinearWithDropOut::InputDerivative(const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
	//m_weights.TransposeMul(outputDerivative, inputDerivative);
}

void ndBrainLayerLinearWithDropOut::CalculateParamGradients(
	const ndBrainVector& input, const ndBrainVector& output,
	const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const
{
	if (m_droutOutEnable)
	{
		const ndBrainFloat* const outMemory = &outputDerivative[0];
		ndBrainMemVector outDerivative(outMemory, outputDerivative.GetCount());
		outDerivative.Mul(m_dropout);
	}
	ndBrainLayerLinear::CalculateParamGradients(input, output, outputDerivative, inputGradient, gradientOut);
}
