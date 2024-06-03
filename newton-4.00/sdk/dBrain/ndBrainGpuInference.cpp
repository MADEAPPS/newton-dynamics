/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuInference.h"

#if defined (D_USE_VULKAN_SDK)

ndBrainGpuInference::ndBrainGpuInference(ndBrainGpuContext* const context, ndBrain* const brain, const ndBrainMatrix& input, ndInt32 inputBatchSize)
	:ndClassAlloc()
	,m_brain(brain)
	,m_context(context)
{
	m_inputBatchSize = inputBatchSize;
	SetWorkingBuffer();
	SetParameterVector();
	LoadInput(input);
}

ndBrainGpuInference::~ndBrainGpuInference()
{
	delete m_input;
	delete m_gpuParameters;
	delete m_gpuWorkingBuffer;
	delete m_gpuParametersOffsets;
}

void ndBrainGpuInference::SetParameterVector()
{
	m_offsets.SetCount(0);
	m_parameters.SetCount(0);

	const ndArray<ndBrainLayer*>& layers = *m_brain;
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		const ndBrainLayer* const layer = layers[i];
		layer->GetNumberOfParameters(m_parameters, m_offsets);
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_offsets.GetCount(); ++i)
	{
		ndInt32 count = m_offsets[i];
		m_offsets[i] = sum;
		sum += count;
	}
	m_offsets.PushBack(sum);
	m_gpuParametersOffsets = new ndBrainGpuIntegerBuffer(m_context, m_offsets);
	m_gpuParameters = new ndBrainGpuFloatBuffer(m_context, m_parameters);
}

void ndBrainGpuInference::SetWorkingBuffer()
{
	ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat);
	ndAssert(!(rounding & (rounding - 1)));
	const ndArray<ndBrainLayer*>& layers = *m_brain;

	ndInt32 size = (layers[0]->GetInputSize() + rounding - 1) & -rounding;
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		size += (layers[i]->GetOutputBufferSize() + rounding - 1) & -rounding;
	}

	size = size * m_inputBatchSize;
	m_gpuWorkingBuffer = new ndBrainGpuFloatBuffer(m_context, size);
}

void ndBrainGpuInference::LoadInput(const ndBrainMatrix& input)
{
	ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat); 
	ndInt32 width = (input.GetColumns() + rounding - 1) & -rounding;
	ndInt32 size = width * input.GetRows();

	ndBrainVector temp;
	temp.SetCount(size);
	for (ndInt32 i = 0; i < input.GetRows(); ++i)
	{
		const ndBrainVector& src = input[i];
		ndBrainMemVector dst(&temp[i * width], src.GetCount());
		dst.Set(src);
	}
	m_input = new ndBrainGpuFloatBuffer(m_context, temp);
}

#endif