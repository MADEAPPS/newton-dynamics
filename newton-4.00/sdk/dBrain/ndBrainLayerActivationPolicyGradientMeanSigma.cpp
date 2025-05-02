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

//#define ND_USE_NATURAL_SIGMA
#define ND_MIN_LINEAR_SIGMA	ndBrainFloat(0.01f)
#define ND_MAX_LINEAR_SIGMA	ndBrainFloat(1.0f)
#define ND_NATURAL_SIGMA	ndBrainFloat(0.135335f)

ndBrainLayerActivationPolicyGradientMeanSigma::ndBrainLayerActivationPolicyGradientMeanSigma(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationPolicyGradientMeanSigma::ndBrainLayerActivationPolicyGradientMeanSigma(const ndBrainLayerActivationPolicyGradientMeanSigma& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationPolicyGradientMeanSigma::Clone() const
{
	return new ndBrainLayerActivationPolicyGradientMeanSigma(*this);
}

ndBrainLayer* ndBrainLayerActivationPolicyGradientMeanSigma::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayer* const layer = new ndBrainLayerActivationPolicyGradientMeanSigma(inputs);

	loadSave->ReadString(buffer);
	return layer;
}

const char* ndBrainLayerActivationPolicyGradientMeanSigma::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_POLICY_MEAN_SIGMAN_NAME;
}

//#pragma optimize( "", off )
void ndBrainLayerActivationPolicyGradientMeanSigma::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	output.Set(input);
	const ndInt32 count = ndInt32(input.GetCount()) / 2;
	const ndInt32 start = ndInt32(input.GetCount()) / 2;
	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		#ifdef ND_USE_NATURAL_SIGMA
			output[start + i] = ND_NATURAL_SIGMA * ndExp(ndFloat32(2.0f) * input[start + i]);
		#else
			ndFloat32 b = (ND_MIN_LINEAR_SIGMA + ND_MAX_LINEAR_SIGMA) * ndFloat32(0.5f);
			ndFloat32 a = ndFloat32(1.0f) - b;
			output[start + i] = a * input[start + i] + b;
		#endif
	}
}

//#pragma optimize( "", off )
void ndBrainLayerActivationPolicyGradientMeanSigma::InputDerivative(const ndBrainVector&, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	inputDerivative.Set(ndBrainFloat(1.0f));
	const ndInt32 count = ndInt32(output.GetCount()) / 2;
	const ndInt32 start = ndInt32(output.GetCount()) / 2;
	for (ndInt32 i = count - 1; i >= 0; --i)
	{
		#ifdef ND_USE_NATURAL_SIGMA
			inputDerivative[start + i] = ndFloat32(2.0f) * output[start + i];
		#else
			ndFloat32 b = (ND_MIN_LINEAR_SIGMA + ND_MAX_LINEAR_SIGMA) * ndFloat32(0.5f);
			ndFloat32 a = ndFloat32(1.0f) - b;
			inputDerivative[start + i] = a;
		#endif
	}
	inputDerivative.Mul(outputDerivative);
}
