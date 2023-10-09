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
#include "ndBrainLayerCategoricalSoftmaxActivation.h"

ndBrainLayerCategoricalSoftmaxActivation::ndBrainLayerCategoricalSoftmaxActivation(ndInt32 neurons)
	:ndBrainLayerSoftmaxActivation(neurons)
{
}

ndBrainLayerCategoricalSoftmaxActivation::ndBrainLayerCategoricalSoftmaxActivation(const ndBrainLayerCategoricalSoftmaxActivation& src)
	:ndBrainLayerSoftmaxActivation(src)
{
}

ndBrainLayer* ndBrainLayerCategoricalSoftmaxActivation::Clone() const
{
	return new ndBrainLayerCategoricalSoftmaxActivation(*this);
}


const char* ndBrainLayerCategoricalSoftmaxActivation::GetLabelId() const
{
	return "ndBrainLayerCategoricalSoftmaxActivation";
}

void ndBrainLayerCategoricalSoftmaxActivation::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	#ifdef _DEBUG
	ndAssert(output.GetCount() == inputDerivative.GetCount());
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndInt32 index = 0;
	for (ndInt32 i = 0; i < outputDerivative.GetCount(); ++i)
	{
		ndAssert((outputDerivative[i] == ndBrainFloat(0.0f)) || (outputDerivative[i] == ndBrainFloat(1.0f)));
		index += (outputDerivative[i] == ndBrainFloat(1.0f)) ? 1 : 0;
	}
	ndAssert(index == 1);
	#endif

	// basically it acts as a loss function
	inputDerivative.Set(output);
	inputDerivative.Sub(outputDerivative);

	inputDerivative.FlushToZero();
}

ndBrainLayer* ndBrainLayerCategoricalSoftmaxActivation::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerCategoricalSoftmaxActivation* const layer = new ndBrainLayerCategoricalSoftmaxActivation(inputs);
	loadSave->ReadString(buffer);
	return layer;
}