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
	,m_bias(src.m_bias)
	,m_weights(src.m_weights)
{
}

ndBrainLayerLineal::~ndBrainLayerLineal()
{
}

const char* ndBrainLayerLineal::GetLabelId() const
{
	return "ndBrainLayerLineal";
}

ndBrainLayer* ndBrainLayerLineal::Clone() const
{
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

void ndBrainLayerLineal::CopyFrom(const ndBrainLayer& src)
{
	const ndBrainLayerLineal& linealSrc = (ndBrainLayerLineal&)src;
	m_bias.Set(linealSrc.m_bias);
	m_weights.Set(linealSrc.m_weights);
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

void ndBrainLayerLineal::Save(const ndBrainSave* const loadSave) const
{
	char buffer[1024];
	auto Save = [this, &buffer, &loadSave](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsprintf(buffer, fmt, v_args);
		va_end(v_args);
		loadSave->WriteData(buffer);
	};

	Save("\tinputs %d\n", m_weights.GetColumns());
	Save("\touputs %d\n", m_weights.GetCount());

	Save("\tbias ");
	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		Save("%g ", m_bias[i]);
	}
	Save("\n");

	Save("\tweights\n");
	for (ndInt32 i = 0; i < m_weights.GetCount(); ++i)
	{
		Save("\t\trow_%d ", i);
		const ndBrainVector& row = m_weights[i];
		for (ndInt32 j = 0; j < GetInputSize(); ++j)
		{
			Save("%g ", row[j]);
		}
		Save("\n");
	}
}

ndBrainLayerLineal* ndBrainLayerLineal::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	loadSave->ReadString(buffer);
	ndInt32 outputs = loadSave->ReadInt();
	ndBrainLayerLineal* const layer = new ndBrainLayerLineal(inputs, outputs);

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < outputs; ++i)
	{
		ndReal val = ndReal(loadSave->ReadFloat());
		layer->m_bias[i] = val;
	}

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < outputs; ++i)
	{
		loadSave->ReadString(buffer);
		for (ndInt32 j = 0; j < inputs; ++j)
		{
			ndReal val = ndReal(loadSave->ReadFloat());
			layer->m_weights[i][j] = val;
		}
	}

	loadSave->ReadString(buffer);
	return layer;
}
