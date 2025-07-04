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
#include "ndBrainGpuScopeMapBuffer.h"

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes)
	:ndBrainGpuBuffer(context, sizeInBytes, ndUniformData, ndCpuMappable)
{
}

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data)
	:ndBrainGpuBuffer(context, sizeInBytes, ndUniformData, ndCpuMappable)
{
	LoadData(size_t(sizeInBytes), data);
}

void ndBrainGpuUniformBuffer::UnloadData(size_t sizeInBytes, void* const outputData) const
{
	ndAssert(m_deviceBufferType == ndCpuMappable);
	ndScopeMapBuffer mapBuffer(*this);
	const ndUnsigned8* const src = (ndUnsigned8*)mapBuffer.GetPointer();
	if (src)
	{
		ndAssert(sizeInBytes <= m_sizeInBytes);
		ndUnsigned8* const dst = (ndUnsigned8*)outputData;
		ndMemCpy(dst, src, ndInt64(sizeInBytes));
	}
}

void ndBrainGpuUniformBuffer::LoadData(size_t sizeInBytes, const void* const inputData)
{
	ndAssert(m_deviceBufferType == ndCpuMappable);
	ndScopeMapBuffer mapBuffer(*this);
	ndUnsigned8* const dst = (ndUnsigned8*)mapBuffer.GetPointer();
	if (dst)
	{
		ndAssert(sizeInBytes <= m_sizeInBytes);
		ndUnsigned8* const src = (ndUnsigned8*)inputData;
		ndMemCpy(dst, src, ndInt64(sizeInBytes));
	}
}

