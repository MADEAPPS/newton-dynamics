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
#include "ndBrainLayer.h"

ndBrainLayer::ndBrainLayer(const ndBrainLayer& src)
	:ndClassAlloc(src)
{
	ndAssert(0);
}

ndBrainLayer::ndBrainLayer()
	:ndClassAlloc()
{
}

ndBrainLayer::~ndBrainLayer()
{
	ndAssert(0);
}

ndBrainLayer* ndBrainLayer::Clone() const
{
	ndAssert(0);
	return nullptr;
}

const char* ndBrainLayer::GetLabelId() const
{
	ndAssert(0);
	return "ndBrainLayer";
}

ndInt32 ndBrainLayer::GetInputSize() const
{
	ndAssert(0);
	return 0;
}
 
ndInt32 ndBrainLayer::GetOutputSize() const
{
	ndAssert(0);
	return 0;
}

void ndBrainLayer::CopyFrom(const ndBrainLayer&)
{
	ndAssert(0);
}

bool ndBrainLayer::HasParameters() const
{
	ndAssert(0);
	return false;
}

void ndBrainLayer::InitWeights(ndReal, ndReal)
{
	ndAssert(0);
}

void ndBrainLayer::InitWeightsXavierMethod()
{
	ndAssert(0);
}

//ndBrainActivationType ndBrainLayer::GetActivationType() const
//{
//	ndAssert(0);
//	return m_noActivation;
//}
//
//void ndBrainLayer::Blend(const ndBrainLayer&, ndReal)
//{
//	ndAssert(0);
//}
//
//bool ndBrainLayer::Compare(const ndBrainLayer&) const
//{
//	ndAssert(0);
//	return false;
//}
//
//void ndBrainLayer::Load(const ndBrainLoad* const)
//{
//	ndAssert(0);
//}
//
//void ndBrainLayer::Save(const ndBrainSave* const) const
//{
//	ndAssert(0);
//}
//
//void ndBrainLayer::ApplyActivation(ndBrainVector&) const
//{
//	ndAssert(0);
//}
//
//void ndBrainLayer::ActivationDerivative(const ndBrainVector&, ndBrainVector&) const
//{
//	ndAssert(0);
//}


void ndBrainLayer::ClearGradAcc(ndBrainVector&, ndBrainMatrix&) const
{
	ndAssert(0);
}

void ndBrainLayer::MakePrediction(const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayer::ActivationDerivative(const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayer::CalculateOutputLayersParamGradients(const ndBrainVector&, const ndBrainVector&, ndBrainVector&, ndBrainMatrix&) const
{
	ndAssert(0);
}

void ndBrainLayer::InputDerivative(const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}