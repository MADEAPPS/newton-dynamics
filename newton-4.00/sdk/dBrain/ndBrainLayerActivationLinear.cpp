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
//#include "ndBrainFloat4.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainContext.h"
#include "ndBrainLayerActivationLinear.h"

ndBrainLayerActivationLinear::ndBrainLayerActivationLinear(const ndBrainVector& slopes, const ndBrainVector& biases)
	:ndBrainLayerActivation(ndInt32 (slopes.GetCount()))
	,m_slopes(slopes)
	,m_biases(biases)
{
	ndAssert(slopes.GetCount() == biases.GetCount());
}

ndBrainLayerActivationLinear::ndBrainLayerActivationLinear(const ndBrainLayerActivationLinear& src)
	:ndBrainLayerActivation(src)
	,m_slopes(src.m_slopes)
	,m_biases(src.m_biases)
{
}

ndBrainLayer* ndBrainLayerActivationLinear::Clone() const
{
	return new ndBrainLayerActivationLinear(*this);
}

const char* ndBrainLayerActivationLinear::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_LINEAR_NAME;
}

void ndBrainLayerActivationLinear::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerActivation::Save(loadSave);

	char buffer[1024];
	auto Save = [&buffer, &loadSave](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsnprintf(buffer, sizeof(buffer), fmt, v_args);
		va_end(v_args);
		loadSave->WriteData(buffer);
	};

	Save("\tslopes ");
	for (ndInt32 i = 0; i < m_slopes.GetCount(); ++i)
	{
		Save("%g ", m_slopes[i]);
	}
	Save("\n");

	Save("\tbiases ");
	for (ndInt32 i = 0; i < m_biases.GetCount(); ++i)
	{
		Save("%g ", m_biases[i]);
	}
	Save("\n");
}

ndBrainLayer* ndBrainLayerActivationLinear::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();

	ndBrainVector activationBiases;
	ndBrainVector activationSlopes;
	activationBiases.SetCount(inputs);
	activationSlopes.SetCount(inputs);

	activationBiases.Set(ndFloat32(0.0f));
	activationSlopes.Set(ndFloat32(1.0f));
	ndBrainLayerActivationLinear* const layer = new ndBrainLayerActivationLinear(activationSlopes, activationBiases);

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_slopes[i] = val;
	}

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_biases[i] = val;
	}

	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationLinear::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	ndAssert(m_slopes.GetCount() == m_biases.GetCount());

	output.Set(input);
	output.Mul(m_slopes);
	output.Add(m_biases);
}

void ndBrainLayerActivationLinear::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(m_slopes.GetCount() == m_biases.GetCount());
	ndAssert(m_slopes.GetCount() == outputDerivative.GetCount());
	ndAssert(inputDerivative.GetCount() == outputDerivative.GetCount());

	inputDerivative.Set(m_slopes);
	inputDerivative.Mul(outputDerivative);
}

