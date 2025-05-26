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
#include "ndBrainTrainerCpuInference.h"
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

	const ndBrainSimdFloat8 max(30.0f);
	const ndBrainSimdFloat8 min(-30.0f);
	ndBrainFloat* const dst = &output[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 value(x.Clamp(min, max));
		value.Tanh().Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	{
		ndBrainFloat value = ndClamp(src[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		dst[i] = ndBrainFloat(ndTanh(value));
	}
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

bool ndBrainLayerActivationTanh::HasGpuSupport() const
{
	return true;
}

ndBrainLayer::ndLayerUniformDataCpu* ndBrainLayerActivationTanh::GetLayerUniformDataCpu() const
{
	ndLayerUniformDataCpu* const data = new ndLayerUniformDataCpu(this);

	data->m_inputSize = GetInputSize();
	data->m_outputSize = GetOutputSize();
	return data;
}

ndBrainLayer::ndLayerUniformDataGpu ndBrainLayerActivationTanh::GetLayerUniformDataGpu(const ndBrainGpuContext* const context) const
{
	ndLayerUniformDataGpu data;
	data.m_shader = context->m_ndBrainLayerTanhActivation;
	data.m_inputSize = GetInputSize();
	data.m_outputSize = GetOutputSize();

	return data;
}

void ndBrainLayerActivationTanh::FeedForward(const ndLayerUniformDataCpu* const info, ndInt32 miniBatchIndex) const
{
	const ndBrainTrainerCpuInference* const trainer = info->m_owner;

	ndInt32 inputSize = info->m_inputSize;
	ndInt32 outputSize = info->m_outputSize;

	ndInt32 offset = miniBatchIndex * info->m_inputOutputSize + info->m_inputOutputStartOffset;
	const ndBrainMemVector input(&trainer->m_inputOutputBuffer[offset], inputSize);
	ndBrainMemVector output(&trainer->m_inputOutputBuffer[offset + inputSize], outputSize);

	const ndBrainSimdFloat8 max(30.0f);
	const ndBrainSimdFloat8 min(-30.0f);
	ndBrainFloat* const dst = &output[0];
	const ndBrainFloat* const src = &input[0];
	const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&src[i]);
		const ndBrainSimdFloat8 value(x.Clamp(min, max));
		value.Tanh().Store(&dst[i]);
	}
	for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	{
		ndBrainFloat value = ndClamp(src[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
		dst[i] = ndBrainFloat(ndTanh(value));
	}
	output.FlushToZero();

	// verify
	//ndBrainFixSizeVector<1000> xxxx;
	//xxxx.SetCount(outputSize);
	//MakePrediction(input, xxxx);
}