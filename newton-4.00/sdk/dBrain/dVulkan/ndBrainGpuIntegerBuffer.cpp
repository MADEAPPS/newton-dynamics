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
#include "ndBrainGpuIntegerBuffer.h"
#include "ndBrainGpuScopeMapBuffer.h"

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, ndInt64 size, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, size* ndInt32(sizeof(ndInt32)), ndStorageData, deviceType)
{
}

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input, ndDeviceBufferType deviceType)
	:ndBrainGpuBuffer(context, input.GetCount()* ndInt32(sizeof(ndInt32)), ndStorageData, deviceType)
{
	LoadData(input);
}

void ndBrainGpuIntegerBuffer::LoadData(const ndArray<ndInt32>& input)
{
	input.GetCount();

	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		ndInt32* const dst = (ndInt32*)mapBuffer.GetPointer();
		if (dst)
		{
			size_t size = m_sizeInBytes / sizeof(ndInt32);
			ndAssert(size == size_t(input.GetCount()));

			for (ndInt64 i = 0; i < ndInt64(size); ++i)
			{
				dst[i] = input[i];
			}
		}
	}
}

void ndBrainGpuIntegerBuffer::UnloadData(ndArray<ndInt32>& output)
{
	output.GetCount();
	if (m_deviceBufferType == ndCpuMappable)
	{
		ndScopeMapBuffer mapBuffer(*this);
		const ndInt32* const src = (ndInt32*)mapBuffer.GetPointer();
		if (src)
		{
			size_t size = m_sizeInBytes / sizeof(ndInt32);
			output.SetCount(ndInt64(size));
			for (ndInt64 i = 0; i < ndInt64(size); ++i)
			{
				output[i] = src[i];
			}
		}
	}
}
