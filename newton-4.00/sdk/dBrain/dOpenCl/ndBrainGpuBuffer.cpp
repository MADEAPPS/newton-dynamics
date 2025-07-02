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
#include "ndBrainBuffer.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"

//ndBrainGpuBuffer::ndBrainGpuBuffer(ndBrainContext* const context, ndInt64 sizeInByte, bool memoryMapped)
ndBrainGpuBuffer::ndBrainGpuBuffer(const ndBrainBuffer* const owner)
	:ndClassAlloc()
	,m_buffer()
	,m_owner((ndBrainBuffer*)owner)
	,m_memory(nullptr)
{
	cl_int error = CL_SUCCESS;
	ndBrainGpuContext* const context = m_owner->m_context->GetAsGpuContext();
	ndAssert(context);

	size_t size = m_owner->SizeInBytes();
	if (m_owner->m_isMemoryMapped && context->SupportsMappedMemory())
	{
		ndAssert(0);
		m_memory = (ndUnsigned32*)clSVMAlloc(context->m_context->get(), CL_MEM_READ_WRITE | CL_MEM_SVM_FINE_GRAIN_BUFFER, size, 32);
		ndAssert(m_memory);
		m_buffer = ndSharedPtr<cl::Buffer>(new cl::Buffer(**context->m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, size, m_memory, &error));
	}
	else
	{
		m_owner->m_isMemoryMapped = false;
		m_buffer = ndSharedPtr<cl::Buffer> (new cl::Buffer(**context->m_context, CL_MEM_READ_WRITE, size, nullptr, &error));
	}
	ndAssert(error == CL_SUCCESS);
}

ndBrainGpuBuffer::~ndBrainGpuBuffer()
{
	ndAssert(0);
	//m_buffer = ndSharedPtr<cl::Buffer>();
	//if (m_memory)
	//{
	//	ndAssert(m_isMemoryMapped);
	//	//m_context->GetAsGpuContext()->m_context->lcE
	//	clSVMFree(m_context->GetAsGpuContext()->m_context->get(), m_memory);
	//}
}

void* ndBrainGpuBuffer::GetPtr()
{
	ndAssert(0);
	return nullptr;
	//return &m_memory[0];
}

#if 0
void ndBrainGpuBuffer::MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputMemory)
{
	LoadData(offsetInBytes, sizeInBytes, inputMemory);
}

void ndBrainGpuBuffer::MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	UnloadData(offsetInBytes, sizeInBytes, outputMemory);
}

void ndBrainGpuBuffer::BrainVectorFromDevice(ndBrainVector& vector) const
{
	ndInt64 size = ndInt64(m_sizeInBytes / sizeof(ndReal));
	vector.SetCount(size);
	UnloadData(0, m_sizeInBytes, &vector[0]);
}

void ndBrainGpuBuffer::BrainVectorToDevice(const ndBrainVector& vector)
{
	ndAssert(vector.GetCount() * sizeof (ndReal) <= m_sizeInBytes);
	LoadData(0, vector.GetCount() * sizeof(ndReal), &vector[0]);
}

void ndBrainGpuBuffer::CopyBuffer(const ndBrainBuffer& parameterBuffer, ndInt32 workGroupCount, const ndBrainBuffer& srcBuffer)
{
	ndAssert(m_context->GetAsGpuContext());
	ndBrainGpuContext* const context = m_context->GetAsGpuContext();
	ndAssert(context);

	ndBrainGpuFloatBuffer& dst = *(ndBrainGpuFloatBuffer*)this;
	ndBrainGpuFloatBuffer& src = *(ndBrainGpuFloatBuffer*)&srcBuffer;
	ndBrainGpuUniformBuffer& uniforms = *(ndBrainGpuUniformBuffer*)&parameterBuffer;

	context->CopyBuffer(uniforms, workGroupCount, dst, src);
}

void ndBrainGpuBuffer::CopyBufferIndirect(const ndBrainBuffer& parameterBuffer, const ndBrainBuffer& indexBuffer, const ndBrainBuffer& srcBuffer)
{
	ndBrainGpuContext* const context = m_context->GetAsGpuContext();
	ndAssert(context);

	ndBrainGpuFloatBuffer& dst = *(ndBrainGpuFloatBuffer*)this;
	ndBrainGpuFloatBuffer& src = *(ndBrainGpuFloatBuffer*)&srcBuffer;
	ndBrainGpuIntegerBuffer& index = *(ndBrainGpuIntegerBuffer*)&indexBuffer;
	ndBrainGpuUniformBuffer& uniforms = *(ndBrainGpuUniformBuffer*)&parameterBuffer;

	context->CopyBufferIndirect(uniforms, index, dst, src);
}

void ndBrainGpuBuffer::LoadData(size_t offsetInBytes, size_t sizeInBytes, const void* const sourceData)
{
	ndAssert(m_context->GetAsGpuContext());

	cl_int error = CL_SUCCESS;
	ndAssert(sizeInBytes <= m_sizeInBytes);
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;

	ndAssert((sizeInBytes & 3) == 0);
	if (m_memory)
	{
		ndMemCpy(&m_memory[ndInt64(offsetInBytes / sizeof(ndUnsigned32))], (ndUnsigned32*)sourceData, ndInt64(sizeInBytes / sizeof(ndUnsigned32)));
	}
	else
	{
		error = queue->enqueueWriteBuffer(**m_buffer, CL_TRUE, offsetInBytes, sizeInBytes, sourceData);
	}
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuBuffer::UnloadData(size_t offsetInBytes, size_t sizeInBytes, void* const destinationData) const
{
	cl_int error = CL_SUCCESS;
	ndAssert(m_context->GetAsGpuContext());
	ndSharedPtr<cl::CommandQueue>& queue = m_context->GetAsGpuContext()->m_queue;
	if (m_memory)
	{
		ndMemCpy((ndUnsigned32*)destinationData, &m_memory[ndInt64(offsetInBytes / sizeof(ndUnsigned32))], ndInt64(sizeInBytes / sizeof(ndUnsigned32)));
	}
	else
	{
		error = queue->enqueueReadBuffer(**m_buffer, CL_TRUE, offsetInBytes, sizeInBytes, destinationData);
	}
	ndAssert(error == CL_SUCCESS);
}
#endif