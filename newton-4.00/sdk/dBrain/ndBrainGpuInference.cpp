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
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuInference.h"


#if defined (D_USE_VULKAN_SDK)

class ndBrainGpuInference::ndBrainLoadInputData : public ndBrainGpuCommand
{
	public:
	struct UniformBufferObject
	{
		int m_batchIndex;
		int m_inputSize;
		int m_inputBlockSize;
		int m_outputBlockSize;
	};

	ndBrainLoadInputData(ndBrainGpuInference* const me, const ndBrainMatrix& input)
		:ndBrainGpuCommand(me->m_context)
		,m_parammeters(me->m_context, sizeof (UniformBufferObject))
	{
		UniformBufferObject uniformParam;
		uniformParam.m_batchIndex = 0;
		uniformParam.m_inputSize = input.GetColumns();
		uniformParam.m_inputBlockSize = me->m_inputBuffer.m_offsets[0];
		uniformParam.m_outputBlockSize = me->m_workingBuffer.m_offsets[me->m_workingBuffer.m_offsets.GetCount() - 1];

		m_parammeters.LoadData(sizeof (uniformParam), &uniformParam);

		ndBrainGpuBuffer* params[3];
		params[0] = &m_parammeters;
		params[1] = me->m_inputBuffer.m_buffer;
		params[2] = me->m_workingBuffer.m_buffer;
		Assembly(me->m_context->m_copyInputData, me->m_inputBatchSize, 3, params);
	}

	ndBrainGpuUniformBuffer m_parammeters;
};

ndBrainGpuInference::ndBrainGpuInference(ndBrainGpuContext* const context, ndBrain* const brain, const ndBrainMatrix& input, ndInt32 inputBatchSize)
	:ndClassAlloc()
	,m_brain(brain)
	,m_context(context)
	,m_inputBuffer()
	,m_workingBuffer()
	,m_displayList()
	,m_inputBatchSize(inputBatchSize)
{
	SetInputBuffer(input);
	SetWorkingBuffer();
	//SetParameterVector();
	
	BuildDisplayList(input);
}

ndBrainGpuInference::~ndBrainGpuInference()
{
	for (ndInt32 i = m_displayList.GetCount() - 1; i >= 0; --i)
	{
		delete m_displayList[i];
	}

	if (m_inputBuffer.m_buffer)
	{
		delete m_inputBuffer.m_buffer;
	}

	if (m_workingBuffer.m_buffer)
	{
		delete m_workingBuffer.m_buffer;
	}
}

void ndBrainGpuInference::SetParameterVector()
{
	ndAssert(0);
	//m_offsets.SetCount(0);
	//m_parameters.SetCount(0);
	//
	//const ndArray<ndBrainLayer*>& layers = *m_brain;
	//for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	//{
	//	const ndBrainLayer* const layer = layers[i];
	//	layer->GetNumberOfParameters(m_parameters, m_offsets);
	//}
	//
	//ndInt32 sum = 0;
	//for (ndInt32 i = 0; i < m_offsets.GetCount(); ++i)
	//{
	//	ndInt32 count = m_offsets[i];
	//	m_offsets[i] = sum;
	//	sum += count;
	//}
	//m_offsets.PushBack(sum);
	//m_gpuParametersOffsets = new ndBrainGpuIntegerBuffer(m_context, m_offsets);
	//m_gpuParameters = new ndBrainGpuFloatBuffer(m_context, m_parameters);
}

void ndBrainGpuInference::SetWorkingBuffer()
{
	ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat);
	ndAssert(!(rounding & (rounding - 1)));
	const ndArray<ndBrainLayer*>& layers = *m_brain;
	
	ndInt32 size = (layers[0]->GetInputSize() + rounding - 1) & -rounding;
	m_workingBuffer.m_offsets.PushBack(size);
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		size = (layers[i]->GetOutputBufferSize() + rounding - 1) & -rounding;
		m_workingBuffer.m_offsets.PushBack(size);
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_workingBuffer.m_offsets.GetCount(); ++i)
	{
		ndInt32 offset = m_workingBuffer.m_offsets[i];
		m_workingBuffer.m_offsets[i] = sum;
		sum += offset;
	}

	m_workingBuffer.m_offsets.PushBack(sum);
	m_workingBuffer.m_buffer = new ndBrainGpuFloatBuffer(m_context, sum * m_inputBatchSize);
}

void ndBrainGpuInference::SetInputBuffer(const ndBrainMatrix& input)
{
	ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat); 
	ndInt32 width = (input.GetColumns() + rounding - 1) & -rounding;
	m_inputBuffer.m_offsets.PushBack(width);

	ndInt32 size = width * input.GetRows();
	ndBrainVector temp;
	temp.SetCount(size);
	for (ndInt32 i = 0; i < input.GetRows(); ++i)
	{
		const ndBrainVector& src = input[i];
		ndBrainMemVector dst(&temp[i * width], src.GetCount());
		dst.Set(src);
	}
	m_inputBuffer.m_buffer = new ndBrainGpuFloatBuffer(m_context, temp);
}

void ndBrainGpuInference::BuildDisplayList(const ndBrainMatrix& input)
{
	ndBrainGpuCommand* const comand = new ndBrainLoadInputData(this, input);
	m_displayList.PushBack(comand);


	m_context->SubmitQueue(&m_displayList[0], m_displayList.GetCount());

	ndBrainVector xxxxx;
	m_workingBuffer.m_buffer->UnloadData(xxxxx);
	ndInt32 stride = m_workingBuffer.m_offsets[m_workingBuffer.m_offsets.GetCount() - 1];
	for (ndInt32 i = 0; i < m_inputBatchSize; ++i)
	{
		const ndBrainVector& src0 = input[i];
		const ndBrainMemVector src1(&xxxxx[i * stride], src0.GetCount());
		for (ndInt32 j = 0; j < m_inputBatchSize; ++j)
		{
			ndAssert(src0[j] == src1[j]);
		}
	}
}
#endif