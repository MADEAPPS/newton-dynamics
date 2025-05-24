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
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuScopeMapBuffer.h"

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt64 size, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, size * ndInt32(sizeof(ndReal)), ndStorageData, deviceType)
{
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, input.GetCount() * ndInt32(sizeof(ndReal)), ndStorageData, deviceType)
{
	LoadData(ndInt32 (input.GetCount() * sizeof (ndReal)),  & input[0]);
}

#if defined (D_USE_VULKAN_SDK)
void ndBrainGpuFloatBuffer::LoadData(ndInt32 sizeInBytes, const void* const inputData)
{
	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		ndReal* const dst = (ndReal*)mapBuffer.GetPointer();
		if (dst)
		{
			ndAssert(sizeInBytes <= m_sizeInBytes);

			const ndReal* const src = (ndReal*)inputData;
			const ndInt64 size = sizeInBytes / ndInt32(sizeof(ndReal));

			ndBrainMemVector dstData(dst, size);
			const ndBrainMemVector srcData(src, size);
			dstData.Set(srcData);
		}
	}
}

void ndBrainGpuFloatBuffer::UnloadData(ndInt32 sizeInBytes, void* const outputData)
{
	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		const ndReal* const src = (ndReal*)mapBuffer.GetPointer();
		if (src)
		{
			ndAssert(sizeInBytes <= m_sizeInBytes);

			const ndInt64 size = sizeInBytes / ndInt32(sizeof(ndReal));
			const ndBrainMemVector srcData(src, size);
			ndBrainMemVector dstData((ndReal*)outputData, size);
			dstData.Set(srcData);
		}
	}
}
#else

void ndBrainGpuFloatBuffer::LoadData(ndInt32, const void* const)
{
}

void ndBrainGpuFloatBuffer::UnloadData(ndInt32, void* const)
{
}

#endif



