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
		output[i] = (input[i] > ndBrainFloat(0.0f)) ? input[i] : ndBrainFloat(0.0f);
		ndAssert(ndCheckFloat(output[i]));
	}
}

void ndBrainLayerReluActivation::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		inputDerivative[i] = (output[i] > ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		ndAssert(ndCheckFloat(inputDerivative[i]));
	}
	inputDerivative.Mul(outputDerivative);
}

ndBrainLayer* ndBrainLayerReluActivation::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerReluActivation* const layer = new ndBrainLayerReluActivation(inputs);
	loadSave->ReadString(buffer);
	return layer;
}