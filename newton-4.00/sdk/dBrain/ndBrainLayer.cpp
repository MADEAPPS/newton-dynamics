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
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuIntegerBuffer.h"
#include "ndBrainGpuUniformBuffer.h"

class ndBrainGpuContext;

ndBrainLayer::ndBufferOffsetPair::ndBufferOffsetPair() 
	:m_buffer(nullptr)
	,m_offsets()
{
}

ndBrainLayer::ndBufferOffsetPair::~ndBufferOffsetPair()
{
	if (m_buffer)
	{
		delete m_buffer;
	}
}

ndBrainLayer::ndBrainLayer(const ndBrainLayer& src)
	:ndClassAlloc(src)
{
}

ndBrainLayer::ndBrainLayer()
	:ndClassAlloc()
{
}

ndBrainLayer::~ndBrainLayer()
{
}

ndBrainLayer* ndBrainLayer::Clone() const
{
	ndAssert(0);
	return nullptr;
}

ndInt32 ndBrainLayer::GetNumberOfParameters() const
{
	return 0;
}

void ndBrainLayer::GetNumberOfGPUParameters(ndBrainVector&, ndArray<ndInt32>&) const
{
	ndAssert(0);
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

ndInt32 ndBrainLayer::GetOutputBufferSize() const
{
	return GetOutputSize();
}

bool ndBrainLayer::HasParameters() const
{
	ndAssert(0);
	return false;
}

void ndBrainLayer::UpdateDropOut()
{
}

void ndBrainLayer::EnableDropOut(bool)
{
}

void ndBrainLayer::InitWeights(ndBrainFloat, ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainLayer::InitWeightsXavierMethod()
{
	ndAssert(0);
}

void ndBrainLayer::Clear()
{
	ndAssert(0);
}

void ndBrainLayer::FlushToZero()
{
	ndAssert(0);
}

void ndBrainLayer::Scale(ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainLayer::Set(const ndBrainLayer&)
{
	ndAssert(0);
}

void ndBrainLayer::Add(const ndBrainLayer&)
{
	ndAssert(0);
}

void ndBrainLayer::Mul(const ndBrainLayer&)
{
	ndAssert(0);
}

void ndBrainLayer::ScaleAdd(const ndBrainLayer&, ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainLayer::Blend(const ndBrainLayer&, ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainLayer::AdamUpdate(const ndBrainLayer&, const ndBrainLayer&, ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainLayer::Save(const ndBrainSave* const) const
{
	ndAssert(0);
}

void ndBrainLayer::MakePrediction(const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayer::InputDerivative(const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayer::CalculateParamGradients(const ndBrainVector&, const ndBrainVector&, const ndBrainVector&, ndBrainVector&, ndBrainLayer* const) const
{
	ndAssert(0);
}

ndBrainGpuCommand* ndBrainLayer::AssemblyGPUCommand(ndBrainGpuContext* const, ndInt32, ndInt32, ndFixSizeArray<ndBufferOffsetPair*, 8>&)
{
	ndAssert(0);
	return nullptr;
}
