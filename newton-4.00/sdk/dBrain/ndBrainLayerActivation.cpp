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
#include "ndBrainLayerActivation.h"

ndBrainLayerActivation::ndBrainLayerActivation()
	:ndBrainLayer()
{
	ndAssert(0);
}

ndBrainLayerActivation::ndBrainLayerActivation(const ndBrainLayerActivation& src)
	:ndBrainLayer(src)
{
	ndAssert(0);
}

ndBrainLayerActivation::~ndBrainLayerActivation()
{
	ndAssert(0);
}

bool ndBrainLayerActivation::HasParameters() const
{
	ndAssert(0);
	return false;
}

const char* ndBrainLayerActivation::GetLabelId() const
{
	ndAssert(0);
	return "ndBrainLayerActivation";
}

ndBrainLayer* ndBrainLayerActivation::Clone() const
{
	ndAssert(0);
	return nullptr;
}

ndInt32 ndBrainLayerActivation::GetOuputSize() const
{
	ndAssert(0);
	return 0;
}

ndInt32 ndBrainLayerActivation::GetInputSize() const
{
	ndAssert(0);
	return 0;
}

void ndBrainLayerActivation::MakePrediction(const ndBrainVector&, ndBrainVector&)
{
	ndAssert(0);
}


ndBrainActivationType ndBrainLayerActivation::GetActivationType() const
{
	ndAssert(0);
	return m_noActivation;
}

void ndBrainLayerActivation::CopyFrom(const ndBrainLayer&)
{
	ndAssert(0);
}

void ndBrainLayerActivation::Blend(const ndBrainLayer&, ndReal)
{
	ndAssert(0);
}

bool ndBrainLayerActivation::Compare(const ndBrainLayer&) const
{
	ndAssert(0);
	return false;
}

void ndBrainLayerActivation::Load(const ndBrainLoad* const)
{
	ndAssert(0);
}

void ndBrainLayerActivation::Save(const ndBrainSave* const) const
{
	ndAssert(0);
}

void ndBrainLayerActivation::ApplyActivation(ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayerActivation::ActivationDerivative(const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayerActivation::InitWeightsXavierMethod()
{
}

void ndBrainLayerActivation::InitWeights(ndReal, ndReal)
{
}

