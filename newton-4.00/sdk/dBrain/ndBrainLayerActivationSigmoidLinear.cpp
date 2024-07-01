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
#include "ndBrainLayerActivationSigmoidLinear.h"

ndBrainLayerActivationSigmoidLinear::ndBrainLayerActivationSigmoidLinear(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationSigmoidLinear::ndBrainLayerActivationSigmoidLinear(const ndBrainLayerActivationSigmoidLinear& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationSigmoidLinear::Clone() const
{
	return new ndBrainLayerActivationSigmoidLinear(*this);
}

const char* ndBrainLayerActivationSigmoidLinear::GetLabelId() const
{
	return "ndBrainLayerActivationSigmoidLinear";
}

ndBrainLayer* ndBrainLayerActivationSigmoidLinear::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationSigmoidLinear* const layer = new ndBrainLayerActivationSigmoidLinear(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationSigmoidLinear::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		ndBrainFloat p = ndBrainFloat(ndExp(-value));
		ndBrainFloat out = ndFlushToZero(input[i] / (ndBrainFloat(1.0f) + p));

		ndAssert(ndCheckFloat(out));
		ndAssert(out <= ndBrainFloat(100.0f));
		ndAssert(out >= ndBrainFloat(-0.5f));
		output[i] = out;
	}
}

void ndBrainLayerActivationSigmoidLinear::InputDerivative(const ndBrainVector& input, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(input.GetCount() == outputDerivative.GetCount());
	ndAssert(input.GetCount() == inputDerivative.GetCount());

	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		ndBrainFloat p = ndBrainFloat(ndExp(-value));
		ndBrainFloat sigmoid = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) + p);
		ndBrainFloat sigmoidDer = sigmoid * (ndBrainFloat(1.0f) - sigmoid);

		ndBrainFloat derivative = input[i] * sigmoidDer + sigmoid;
		ndAssert(derivative < 1.5f);
		ndAssert(derivative > -0.5f);
		inputDerivative[i] = outputDerivative[i] * derivative;
	}
	inputDerivative.FlushToZero();
}
