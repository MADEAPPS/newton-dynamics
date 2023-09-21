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
#include "ndBrainLayerTanhActivation.h"

ndBrainLayerTanhActivation::ndBrainLayerTanhActivation(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerTanhActivation::ndBrainLayerTanhActivation(const ndBrainLayerTanhActivation& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerTanhActivation::Clone() const
{
	return new ndBrainLayerTanhActivation(*this);
}

const char* ndBrainLayerTanhActivation::GetLabelId() const
{
	return "ndBrainLayerTanhActivation";
}

void ndBrainLayerTanhActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
#if 0
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat out = ndBrainFloat(0.0f);
		ndBrainFloat value = input[i];
		ndBrainFloat out1 = ndTanh(value);
		if (value > ndBrainFloat(0.0f))
		{
			ndBrainFloat p = ndBrainFloat(ndExp(-ndBrainFloat(2.0f) * value));
			out = ndFlushToZero((ndBrainFloat(1.0f) - p) / (p + ndBrainFloat(1.0f)));
		}
		else
		{
			ndBrainFloat p = ndBrainFloat(ndExp(ndBrainFloat(2.0f) * value));
			out = ndFlushToZero((p - ndBrainFloat(1.0f)) / (p + ndBrainFloat(1.0f)));
		}

		output[i] = out;
		ndAssert(ndCheckFloat(output[i]));
		ndAssert(output[i] <= ndBrainFloat(1.0f));
		ndAssert(output[i] >= ndBrainFloat(-1.0f));
	}
#else
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		output[i] = ndTanh(input[i]);
	}
	output.FlushToZero();
#endif
}

void ndBrainLayerTanhActivation::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	inputDerivative.Set(ndBrainFloat(1.0f));
	inputDerivative.MulSub(output, output);
	inputDerivative.Mul(outputDerivative);
	inputDerivative.FlushToZero();
}

ndBrainLayerTanhActivation* ndBrainLayerTanhActivation::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerTanhActivation* const layer = new ndBrainLayerTanhActivation(inputs);
	loadSave->ReadString(buffer);
	return layer;
}