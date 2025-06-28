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

class ndBrainContext;

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

bool ndBrainLayer::HasGpuSupport() const
{
	return false;
}

void ndBrainLayer::CopyCpuWeights(ndBrainVector&) const
{
}

void ndBrainLayer::CopyGpuWeights(ndBrainVector&) const
{
}

void ndBrainLayer::SetCpuWeights(const ndBrainVector&)
{
}

void ndBrainLayer::SetGpuWeights(const ndBrainVector&)
{
}


void ndBrainLayer::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const, ndInt32) const
{
	ndAssert(0);
}

void ndBrainLayer::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const, ndInt32) const
{
	ndAssert(0);
}

ndBrainLayerFeedForwardCpuCommand* ndBrainLayer::GetLayerCpuFeedForwardCommand()
{
	ndAssert(0);
	return nullptr;
}

ndBrainLayerBackPropagateCpuCommand* ndBrainLayer::GetLayerCpuBackPropagateCommand()
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

ndInt32 ndBrainLayer::GetOutputBufferSize() const
{
	return GetOutputSize();
}

bool ndBrainLayer::HasParameters() const
{
	ndAssert(0);
	return false;
}

void ndBrainLayer::ApplyDropOut(ndFloat32)
{
}

void ndBrainLayer::InitWeights()
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

void ndBrainLayer::AddReqularizerL1(const ndBrainLayer&, ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainLayer::AddReqularizerL2(const ndBrainLayer&, ndBrainFloat)
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

void ndBrainLayer::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayer::CalculateParamGradients(const ndBrainVector&, const ndBrainVector&, const ndBrainVector&, ndBrainVector&, ndBrainLayer* const) const
{
	ndAssert(0);
}

ndBrainLayer::ndCommandShareInfo ndBrainLayer::GetCpuCommandSharedInfo() const
{
	ndAssert(0);
	return ndCommandShareInfo(this);
}

ndBrainLayer::ndCommandShareInfo ndBrainLayer::GetGpuCommandSharedInfo() const
{
	ndAssert(0);
	return ndCommandShareInfo(this);
}

ndBrainCommandBuffer* ndBrainLayer::CreateGpuFeedForwardCommand(
	ndBrainTrainerInference* const, const ndBrainLayer::ndCommandShareInfo&, 
	ndBrainContext* const, ndInt32, const ndSharedPtr<ndBrainUniformBuffer>&,
	ndBrainFloatBuffer* const, ndBrainFloatBuffer* const) const
{
	ndAssert(0);
	return nullptr;
}

ndBrainCommandBuffer* ndBrainLayer::CreateGpuBackPropagateCommand(
	ndBrainTrainerInference* const, const ndBrainLayer::ndCommandShareInfo&,
	ndBrainContext* const, ndInt32, const ndSharedPtr<ndBrainUniformBuffer>&,
	ndBrainFloatBuffer* const, ndBrainFloatBuffer* const, 
	ndBrainFloatBuffer* const, ndBrainFloatBuffer* const) const
{
	ndAssert(0);
	return nullptr;
}

