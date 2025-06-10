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
#include "ndBrainGpuUniformBuffer.h"

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes)
	:ndBrainGpuBuffer(context, sizeInBytes, ndUniformData, ndCpuMappable)
{
}

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainGpuBuffer(context, sizeInBytes, ndUniformData, ndCpuMappable)
{
	LoadData(size_t(sizeInBytes), data);
}

ndBrainGpuUniformBuffer* ndBrainGpuUniformBuffer::GetAsUniformBuffer()
{
	return this;
}

void ndBrainGpuUniformBuffer::UnloadData(size_t, void* const) const
{
}

void ndBrainGpuUniformBuffer::LoadData(size_t sizeIntBytes, const void* const sourceData)
{
	//const UniformBufferObject* const xxx = (UniformBufferObject*)sourceData;
	m_data.SetCount(0);
	const char* const src = (const char*) sourceData;
	for (ndInt32 i = 0; i < ndInt32 (sizeIntBytes); ++i)
	{
		m_data.PushBack(src[i]);
	}
	//const UniformBufferObject* const xxx1 = (UniformBufferObject*)&m_data[0];
	//const UniformBufferObject* const xxx0 = (UniformBufferObject*)sourceData;
}
