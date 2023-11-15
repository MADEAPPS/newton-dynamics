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
#include "ndBrainLayerLeakyReluActivation.h"

ndBrainLayerLeakyReluActivation::ndBrainLayerLeakyReluActivation(ndInt32 neurons, ndBrainFloat leakDerivative)
	:ndBrainLayerActivation(neurons)
	,m_leakDerivative(leakDerivative)
{
	ndAssert(m_leakDerivative >= ndBrainFloat(0.0f));
	ndAssert(m_leakDerivative <= ndBrainFloat(0.5f));
}

ndBrainLayerLeakyReluActivation::ndBrainLayerLeakyReluActivation(const ndBrainLayerLeakyReluActivation& src)
	:ndBrainLayerActivation(src)
	,m_leakDerivative(src.m_leakDerivative)
{
}

ndBrainLayer* ndBrainLayerLeakyReluActivation::Clone() const
{
	return new ndBrainLayerLeakyReluActivation(*this);
}

const char* ndBrainLayerLeakyReluActivation::GetLabelId() const
{
	return "ndBrainLayerLeakyReluActivation";
}

ndBrainLayer* ndBrainLayerLeakyReluActivation::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();

	loadSave->ReadString(buffer);
	ndBrainFloat leakDerivative = loadSave->ReadFloat();
	ndBrainLayerLeakyReluActivation* const layer = new ndBrainLayerLeakyReluActivation(inputs, leakDerivative);

	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerLeakyReluActivation::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerActivation::Save(loadSave);
	char buffer[1024];
	sprintf(buffer, "\tleakDerivative %f\n", m_leakDerivative);
	loadSave->WriteData(buffer);
}

void ndBrainLayerLeakyReluActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat inValue0 = input[i];
		ndBrainFloat inValue1 = inValue0 * m_leakDerivative;
		output[i] = (inValue0 > ndBrainFloat(0.0f)) ? inValue0 : inValue1;
		ndAssert(ndCheckFloat(output[i]));
	}
}

void ndBrainLayerLeakyReluActivation::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	ndBrainFloat derivatine0 = ndBrainFloat(1.0f);
	ndBrainFloat derivatine1 = m_leakDerivative;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		inputDerivative[i] = (output[i] > ndBrainFloat(0.0f)) ? derivatine0 : derivatine1;
		ndAssert(ndCheckFloat(inputDerivative[i]));
	}
	inputDerivative.Mul(outputDerivative);
	inputDerivative.FlushToZero();
}
