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
#include "ndBrainLayerReluActivation.h"

ndBrainLayerReluActivation::ndBrainLayerReluActivation(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerReluActivation::ndBrainLayerReluActivation(const ndBrainLayerReluActivation& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerReluActivation::Clone() const
{
	ndAssert(0);
	return new ndBrainLayerReluActivation(*this);
}

const char* ndBrainLayerReluActivation::GetLabelId() const
{
	return "ndBrainLayerReluActivation";
}

void ndBrainLayerReluActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		output[i] = (output[i] > ndReal(0.0f)) ? output[i] : ndReal(0.0f);
		ndAssert(ndCheckFloat(output[i]));
		ndAssert(output[i] <= ndReal(1.0f));
		ndAssert(output[i] >= ndReal(-1.0f));
	}
}

void ndBrainLayerReluActivation::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	ndAssert(0);
	//inputDerivative.Set(ndReal(1.0f));
	//inputDerivative.MulSub(output, output);
	//inputDerivative.Mul(outputDerivative);
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		inputDerivative[i] = (output[i] > ndReal(0.0f)) ? ndReal(1.0f) : ndReal(0.0f);
		ndAssert(ndCheckFloat(output[i]));
		ndAssert(output[i] <= ndReal(1.0f));
		ndAssert(output[i] >= ndReal(-1.0f));
	}
}