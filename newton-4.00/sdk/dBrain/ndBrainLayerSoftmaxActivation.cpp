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
#include "ndBrainLayerSoftmaxActivation.h"

ndBrainLayerSoftmaxActivation::ndBrainLayerSoftmaxActivation(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerSoftmaxActivation::ndBrainLayerSoftmaxActivation(const ndBrainLayerActivation& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerSoftmaxActivation::Clone() const
{
	ndAssert(0);
	return new ndBrainLayerSoftmaxActivation(*this);
}


const char* ndBrainLayerSoftmaxActivation::GetLabelId() const
{
	return "ndBrainLayerSoftmaxActivation";
}

//void ndBrainLayerSoftmaxActivation::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
void ndBrainLayerSoftmaxActivation::InputDerivative(const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
	//inputDerivative.Set(ndBrainFloat(1.0f));
	//inputDerivative.Sub(output);
	//inputDerivative.Mul(output);
	//inputDerivative.Mul(outputDerivative);
	//inputDerivative.FlushToZero();
}

void ndBrainLayerSoftmaxActivation::MakePrediction(const ndBrainVector&, ndBrainVector& output) const
{
	ndAssert(0);
	ndBrainFloat max = ndBrainFloat(1.0e-16f);
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		max = output[i];
	}

	ndBrainFloat acc = ndBrainFloat(0.0f);
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndAssert(ndCheckFloat(output[i]));
		ndBrainFloat prob = ndBrainFloat(ndExp(output[i] - max));
		output[i] = prob;
		acc += prob;
	}

	ndAssert(acc > ndBrainFloat (0.0f));
	output.Scale(ndBrainFloat(1.0f) / acc);
	output.FlushToZero();
}

ndBrainLayer* ndBrainLayerSoftmaxActivation::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerSoftmaxActivation* const layer = new ndBrainLayerSoftmaxActivation(inputs);
	loadSave->ReadString(buffer);
	return layer;
}