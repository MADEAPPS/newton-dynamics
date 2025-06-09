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
#ifndef __ND_BRAIN_GPU_INTEGER_BUFFER_H__
#define __ND_BRAIN_GPU_INTEGER_BUFFER_H__

#include "ndBrainGpuBuffer.h"


class ndBrainGpuIntegerBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, ndInt64 size, ndDeviceBufferType deviceType);
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input, ndDeviceBufferType deviceType);

	virtual void* GetBuffer() override;
	void UnloadData(ndArray<ndInt32>& output);
	void LoadData(const ndArray<ndInt32>& input);
};

#endif