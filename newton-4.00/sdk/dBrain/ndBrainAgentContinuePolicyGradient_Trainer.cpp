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
#include "ndBrainAgentContinuePolicyGradient_Trainer.h"

ndBrainLastLinearLayer::ndBrainLastLinearLayer(ndInt32 inputs, ndInt32 outputs)
	:ndBrainLayerLinear(inputs, outputs * 2)
{
}

ndBrainLastLinearLayer::ndBrainLastLinearLayer(const ndBrainLastLinearLayer& src)
	: ndBrainLayerLinear(src)
{
}

ndBrainLayer* ndBrainLastLinearLayer::Clone() const
{
	return new ndBrainLastLinearLayer(*this);
}

void ndBrainLastLinearLayer::Save(const ndBrainSave* const loadSave) const
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
	Save("\touputs %d\n", m_weights.GetCount() / 2);

	Save("\tbias ");
	for (ndInt32 i = 0; i < m_bias.GetCount() / 2; ++i)
	{
		Save("%g ", m_bias[i]);
	}
	Save("\n");

	Save("\tweights\n");
	for (ndInt32 i = 0; i < m_weights.GetCount() / 2; ++i)
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


ndBrainLastActivationLayer::ndBrainLastActivationLayer(ndInt32 neurons)
	:ndBrainLayerTanhActivation(neurons * 2)
	,m_sigma0(ndBrainFloat(0.01f))
	,m_sigmaScale(ndBrainFloat(0.5f))
	,m_sigmaScaleInv(ndBrainFloat(1.0f)/ m_sigmaScale)
{
}

ndBrainLastActivationLayer::ndBrainLastActivationLayer(const ndBrainLastActivationLayer& src)
	:ndBrainLayerTanhActivation(src)
	,m_sigma0(src.m_sigma0)
	,m_sigmaScale(src.m_sigmaScale)
	,m_sigmaScaleInv(src.m_sigmaScaleInv)
{
}

ndBrainLayer* ndBrainLastActivationLayer::Clone() const
{
	return new ndBrainLastActivationLayer(*this);
}

void ndBrainLastActivationLayer::Save(const ndBrainSave* const loadSave) const
{
	char buffer[1024];
	sprintf(buffer, "\tnewrons %d\n", m_neurons / 2);
	loadSave->WriteData(buffer);
}

void ndBrainLastActivationLayer::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndBrainLayerTanhActivation::MakePrediction(input, output);
	for (ndInt32 i = m_neurons / 2; i < m_neurons; ++i)
	{
		output[i] = ndBrainFloat(0.01f) + ndBrainFloat(0.5f) + output[i] * ndBrainFloat(0.5f);
	}
}

void ndBrainLastActivationLayer::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	//ndBrainFloat buffer[1024 * 2];
	//ndBrainMemVector tmpOut(buffer, inputDerivative.GetCount());
	//tmpOut.Set(output);
	//for (ndInt32 i = m_neurons / 2; i < m_neurons; ++i)
	//{
	//	tmpOut[i] = (output[i] - m_sigma0) * m_sigmaScaleInv;
	//}
	//ndBrainLayerTanhActivation::InputDerivative(tmpOut, outputDerivative, inputDerivative);
	//for (ndInt32 i = m_neurons / 2; i < m_neurons; ++i)
	//{
	//	inputDerivative[i] *= m_sigmaScale;
	//}

	for (ndInt32 i = 0; i < m_neurons / 2; ++i)
	{
		ndBrainFloat y = output[i];
		ndBrainFloat derivative = ndBrainFloat(1.0f) - y * y;
		inputDerivative[i] = derivative * outputDerivative[i];
	}
	for (ndInt32 i = m_neurons / 2; i < m_neurons; ++i)
	{
		ndBrainFloat y = (output[i] - m_sigma0) * m_sigmaScaleInv;
		ndBrainFloat derivative = ndBrainFloat(1.0f) - y * y;
		inputDerivative[i] = m_sigmaScale * derivative * outputDerivative[i];
	}
}