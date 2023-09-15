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
#include "ndBrainLayerLineal.h"

ndBrainLayerLineal::ndBrainLayerLineal(ndInt32 inputs, ndInt32 outputs)
	:ndBrainLayer()
	,m_bias()
	,m_weights(outputs, inputs)
{
	m_bias.SetCount(outputs);
}

ndBrainLayerLineal::ndBrainLayerLineal(const ndBrainLayerLineal& src)
	:ndBrainLayer(src)
{
	ndAssert(0);
}

ndBrainLayerLineal::~ndBrainLayerLineal()
{
	ndAssert(0);
}

const char* ndBrainLayerLineal::GetLabelId() const
{
	ndAssert(0);
	return "ndBrainLayerLineal";
}

ndBrainLayer* ndBrainLayerLineal::Clone() const
{
	ndAssert(0);
	return new ndBrainLayerLineal(*this);
}

ndInt32 ndBrainLayerLineal::GetOutputSize() const
{
	ndAssert(m_bias.GetCount() == m_weights.GetRows());
	return m_bias.GetCount();
}

ndInt32 ndBrainLayerLineal::GetInputSize() const
{
	return m_weights.GetColumns();
}

ndBrainVector* ndBrainLayerLineal::GetBias()
{
	return &m_bias;
}

ndBrainMatrix* ndBrainLayerLineal::GetWeights()
{
	return &m_weights;
}

bool ndBrainLayerLineal::HasParameters() const
{
	return true;
}

void ndBrainLayerLineal::InitWeightsXavierMethod()
{
	ndReal weighVariance = ndReal(ndSqrt(ndFloat32(6.0f) / ndFloat32(GetInputSize() + GetOutputSize())));
	InitWeights(weighVariance, ndReal(0.0f));
}

void ndBrainLayerLineal::InitGaussianBias(ndReal variance)
{
	m_bias.InitGaussianWeights(variance);
}

void ndBrainLayerLineal::InitGaussianWeights(ndReal variance)
{
	for (ndInt32 i = m_weights.GetCount() - 1; i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayerLineal::InitWeights(ndReal weighVariance, ndReal biasVariance)
{
	biasVariance = ndMin(biasVariance, ndReal(0.5f));
	weighVariance = ndMin(weighVariance, ndReal(0.5f));
	InitGaussianBias(biasVariance);
	InitGaussianWeights(weighVariance);
}

void ndBrainLayerLineal::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	m_weights.Mul(input, output);
	output.Add(m_bias);
}

void ndBrainLayerLineal::ClearGradAcc(ndBrainVector& gradBiasAcc, ndBrainMatrix& gradWeightAcc) const
{
	gradBiasAcc.Set(ndReal (0.0f));
	gradWeightAcc.Set(ndReal(0.0f));
}

void ndBrainLayerLineal::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	m_weights.TransposeMul(outputDerivative, inputDerivative);
}

void ndBrainLayerLineal::CalculateParamGradients(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative,
	ndBrainVector& inputGradient, ndBrainVector& biasGradient, ndBrainMatrix& weightGradient)
{
	ndAssert(biasGradient.GetCount() == outputDerivative.GetCount());
	biasGradient.Add(outputDerivative);
	for (ndInt32 i = outputDerivative.GetCount() - 1; i >= 0 ; --i)
	{
		ndReal value = outputDerivative[i];
		weightGradient[i].ScaleAdd(input, value);
	}
	InputDerivative(output, outputDerivative, inputGradient);
}
