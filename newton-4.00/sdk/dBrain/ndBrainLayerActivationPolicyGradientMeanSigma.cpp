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
#include "ndBrainLayerActivationPolicyGradientMeanSigma.h"

ndBrainLayerActivationPolicyGradientMeanSigma::ndBrainLayerActivationPolicyGradientMeanSigma(ndInt32 neurons, bool constSigma2)
	:ndBrainLayerActivation(neurons)
	,m_constSigma2(constSigma2)
{
}

ndBrainLayerActivationPolicyGradientMeanSigma::ndBrainLayerActivationPolicyGradientMeanSigma(const ndBrainLayerActivationPolicyGradientMeanSigma& src)
	:ndBrainLayerActivation(src)
	,m_constSigma2(src.m_constSigma2)
{
}

ndBrainLayer* ndBrainLayerActivationPolicyGradientMeanSigma::Clone() const
{
	return new ndBrainLayerActivationPolicyGradientMeanSigma(*this);
}

ndBrainFloat ndBrainLayerActivationPolicyGradientMeanSigma::GetMinSigma() const
{
	return ND_CONTINUE_POLICY_MIN_SIGMA2;
}

void ndBrainLayerActivationPolicyGradientMeanSigma::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerActivation::Save(loadSave);

	char buffer[1024];
	snprintf(buffer, sizeof(buffer), "\tinput_width %d\n", m_constSigma2 ? 1 : 0);
	loadSave->WriteData(buffer);
}

ndBrainLayer* ndBrainLayerActivationPolicyGradientMeanSigma::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();

	loadSave->ReadString(buffer);
	bool useConstSigma = loadSave->ReadInt() ? true : false;

	ndBrainLayerActivationPolicyGradientMeanSigma* const layer = new ndBrainLayerActivationPolicyGradientMeanSigma(inputs, useConstSigma);
	loadSave->ReadString(buffer);
	return layer;
}

const char* ndBrainLayerActivationPolicyGradientMeanSigma::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_POLICY_MEAN_SIGMAN_NAME;
}

#pragma optimize( "", off )
void ndBrainLayerActivationPolicyGradientMeanSigma::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndInt32 base = m_neurons / 2;
	if (m_constSigma2)
	{
		for (ndInt32 i = base - 1; i >= 0; --i)
		{
			output[i] = input[i];
			output[i + base] = ND_CONTINUE_POLICY_CONST_SIGMA2;
		}
	}
	else
	{
		ndBrainFloat minSigma2 = GetMinSigma();
		for (ndInt32 i = base - 1; i >= 0; --i)
		{
			output[i] = input[i];
			ndBrainFloat sigma2 = input[i + base] + minSigma2;
			output[i + base] = ndMax(sigma2, minSigma2);
		}
	}
}

void ndBrainLayerActivationPolicyGradientMeanSigma::InputDerivative(const ndBrainVector& input, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	const ndInt32 base = m_neurons / 2;
	if (m_constSigma2)
	{
		for (ndInt32 i = base - 1; i >= 0; --i)
		{
			inputDerivative[i] = ndBrainFloat(1.0f);
			inputDerivative[i + base] = ndBrainFloat(0.0f);
		}
	}
	else
	{
		ndBrainFloat minSigma2 = GetMinSigma();
		for (ndInt32 i = base - 1; i >= 0; --i)
		{
			inputDerivative[i] = ndBrainFloat(1.0f);
			ndBrainFloat sigma2 = input[i + base] + minSigma2;
			inputDerivative[i + base] = (sigma2 >= minSigma2) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		}
	}
	inputDerivative.Mul(outputDerivative);
}
