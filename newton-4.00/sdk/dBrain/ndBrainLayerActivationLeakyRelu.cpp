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
#include "ndBrainSimdFloat8.h"
#include "gpu/ndBrainGpuContext.h"
#include "ndBrainLayerActivationLeakyRelu.h"

ndBrainLayerActivationLeakyRelu::ndBrainLayerActivationLeakyRelu(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationLeakyRelu::ndBrainLayerActivationLeakyRelu(const ndBrainLayerActivationLeakyRelu& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationLeakyRelu::Clone() const
{
	return new ndBrainLayerActivationLeakyRelu(*this);
}

const char* ndBrainLayerActivationLeakyRelu::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_LEAKY_RELU_NAME;
}

ndBrainLayer* ndBrainLayerActivationLeakyRelu::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationLeakyRelu* const layer = new ndBrainLayerActivationLeakyRelu(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationLeakyRelu::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());

	const ndBrainSimdFloat8 zero(0.0f);
	const ndBrainSimdFloat8 leakyGrad(ND_LEAKY_LRU_NEG_GRADIENT);
	ndBrainFloat* const dst = &output[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 mask(x >= zero);
		const ndBrainSimdFloat8 negOut(leakyGrad * x);
		const ndBrainSimdFloat8 value((x & mask) | (negOut & (~mask)));
		value.Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= (roundCount * 8); --i)
	{
		output[i] = (input[i] >= 0) ? input[i] : ND_LEAKY_LRU_NEG_GRADIENT * input[i];
	}
}

void ndBrainLayerActivationLeakyRelu::InputDerivative(const ndBrainVector& input, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(input.GetCount() == outputDerivative.GetCount());
	ndAssert(input.GetCount() == inputDerivative.GetCount());

	const ndBrainSimdFloat8 one(1.0f);
	const ndBrainSimdFloat8 zero(0.0f);
	const ndBrainSimdFloat8 leakyGrad(ND_LEAKY_LRU_NEG_GRADIENT);
	ndBrainFloat* const dst = &inputDerivative[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 mask(x >= zero);
		const ndBrainSimdFloat8 value((one & mask) | (leakyGrad & (~mask)));
		value.Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= (roundCount * 8); --i)
	{
		inputDerivative[i] = (input[i] >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ND_LEAKY_LRU_NEG_GRADIENT;
	}

	inputDerivative.Mul(outputDerivative);
}

ndBrainGpuCommand* ndBrainLayerActivationLeakyRelu::AssemblyGPUCommand(ndBrainGpuContext* const context, ndInt32 layerIndex, ndInt32 batchCount, ndFixSizeArray<ndBufferOffsetPair*, 8>& params)
{
	return AssemblyGPUCommandCommon(context, layerIndex, batchCount, params, context->m_ndBrainLayerReluActivation);
}