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
	LoadData(input);
}

void ndBrainGpuFloatBuffer::LoadData(const ndBrainVector& input)
{
	input.GetCount();
#if defined (D_USE_VULKAN_SDK)
	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		ndReal* const dst = (ndReal*)mapBuffer.GetPointer();
		if (dst)
		{
			const ndInt64 size = m_sizeInBytes / ndInt32(sizeof(ndReal));
			ndAssert(size == input.GetCount());

			for (ndInt64 i = 0; i < size; ++i)
			{
				dst[i] = ndReal(input[i]);
			}
		}
	}
#endif
}

void ndBrainGpuFloatBuffer::UnloadData(ndBrainVector& output)
{
	output.GetCount();
#if defined (D_USE_VULKAN_SDK)
	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		const ndReal* const src = (ndReal*)mapBuffer.GetPointer();
		if (src)
		{
			const ndInt64 size = m_sizeInBytes / ndInt32(sizeof(ndReal));
			output.SetCount(size);
			const ndBrainMemVector srcData(src, size);
			output.Set(srcData);
		}
	}
#endif
}



