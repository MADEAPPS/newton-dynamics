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
	LoadData(input.GetCount() * sizeof (ndReal),  & input[0]);
}

void ndBrainGpuFloatBuffer::LoadData(size_t sizeInBytes, const void* const inputData)
{
	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		ndReal* const dst = (ndReal*)mapBuffer.GetPointer();
		if (dst)
		{
			ndAssert(sizeInBytes <= m_sizeInBytes);

			const ndReal* const src = (ndReal*)inputData;
			const size_t size = sizeInBytes / sizeof(ndReal);

			ndBrainMemVector dstData(dst, ndInt64(size));
			const ndBrainMemVector srcData(src, ndInt64(size));
			dstData.Set(srcData);
		}
	}
}

void ndBrainGpuFloatBuffer::UnloadData(size_t sizeInBytes, void* const outputData) const
{
	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		const ndReal* const src = (ndReal*)mapBuffer.GetPointer();
		if (src)
		{
			ndAssert(sizeInBytes <= m_sizeInBytes);

			const size_t size = sizeInBytes / sizeof(ndReal);
			const ndBrainMemVector srcData(src, ndInt64(size));
			ndBrainMemVector dstData((ndReal*)outputData, ndInt64(size));
			dstData.Set(srcData);
		}
	}
}
