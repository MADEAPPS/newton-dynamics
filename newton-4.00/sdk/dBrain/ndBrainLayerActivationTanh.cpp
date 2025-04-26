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
#include "ndBrainFloat4.h"
#include "ndBrainSaveLoad.h"
#include "gpu/ndBrainGpuContext.h"
#include "ndBrainLayerActivationTanh.h"

ndBrainLayerActivationTanh::ndBrainLayerActivationTanh(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationTanh::ndBrainLayerActivationTanh(const ndBrainLayerActivationTanh& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationTanh::Clone() const
{
	return new ndBrainLayerActivationTanh(*this);
}

const char* ndBrainLayerActivationTanh::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_TANGH_NAME;
}

ndBrainLayer* ndBrainLayerActivationTanh::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationTanh* const layer = new ndBrainLayerActivationTanh(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationTanh::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());

#ifdef D_NEWTON_USE_AVX2_OPTION
	ndBrainFloat4 maxVal(30.0f);
	ndBrainFloat4 minVal(-30.0f);
	ndBrainFloat4* const dst = (ndBrainFloat4*)&output[0];
	const ndBrainFloat4* const src = (ndBrainFloat4*)&input[0];
	const ndInt32 count = ndInt32(input.GetCount()) >> 2;
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndBrainFloat4 value(maxVal.Min(minVal.Max(src[i])));
		dst[i] = value.Tanh();
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= (count * 4); --i)
	{
		ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		output[i] = ndBrainFloat(ndTanh(value));
	}
#else
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		output[i] = ndBrainFloat(ndTanh(value));
	}
#endif

	output.FlushToZero();
}

void ndBrainLayerActivationTanh::InputDerivative(const ndBrainVector&, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	inputDerivative.Set(ndBrainFloat(1.0f));
	inputDerivative.MulSub(output, output);
	inputDerivative.Mul(outputDerivative);
	inputDerivative.FlushToZero();
}

ndBrainGpuCommand* ndBrainLayerActivationTanh::AssemblyGPUCommand(ndBrainGpuContext* const context, ndInt32 layerIndex, ndInt32 batchCount, ndFixSizeArray<ndBufferOffsetPair*, 8>& params)
{
	return AssemblyGPUCommandCommon(context, layerIndex, batchCount, params, context->m_ndBrainLayerTanhActivation);
}