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
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainCpuContext.h"
#include "ndBrainCpuFloatBuffer.h"

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, ndInt64 sizeInFloat)
	:ndBrainBuffer(context, sizeInFloat * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(0);
	m_buffer.SetCount(sizeInFloat);
}

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsCpuContext());
	m_buffer.SetCount(ndInt64(m_sizeInBytes / sizeof(ndReal)));
	LoadData(input.GetCount() * sizeof (ndReal), &input[0]);
}

ndBrainCpuFloatBuffer::ndBrainCpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainBuffer(context, matrix.GetColumns() * matrix.GetRows() * ndInt32(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsCpuContext());
	m_buffer.SetCount(ndInt64(m_sizeInBytes / sizeof(ndReal)));
	LoadBuffer(&matrix);
}

//void ndBrainCpuFloatBuffer::LoadData(size_t sizeInBytes, const void* const sourceData)
void ndBrainCpuFloatBuffer::LoadData(size_t, const void* const)
{
	ndAssert(0);
	//ndAssert(m_context->GetAsGpuContext());
	//
	//cl_int error = 0;
	//ndAssert(sizeInBytes == m_sizeInBytes);
	//ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	//error =	queue->enqueueWriteBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, sourceData);
	//ndAssert(error == CL_SUCCESS);
}

//void ndBrainCpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const destinationData) const
void ndBrainCpuFloatBuffer::UnloadData(size_t, void* const) const
{
	ndAssert(0);
	//cl_int error = 0;
	//ndAssert(sizeInBytes == m_sizeInBytes);
	//ndAssert(m_context->GetAsGpuContext());
	//ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	//error = queue->enqueueReadBuffer(m_buffer, CL_TRUE, 0, sizeInBytes, destinationData);
	//ndAssert(error == CL_SUCCESS);
}

void ndBrainCpuFloatBuffer::LoadBuffer(const ndBrainMatrix* const matrix)
{
	for (ndInt32 i = 0; i < matrix->GetRows(); ++i)
	{
		ndBrainMemVector dst (&m_buffer[i * matrix->GetColumns()], matrix->GetColumns());
		dst.Set((*matrix)[i]);
	}
}