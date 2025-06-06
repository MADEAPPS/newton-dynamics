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
#ifndef __ND_BRAIN_GPU_FLOAT_BUFFER_H__
#define __ND_BRAIN_GPU_FLOAT_BUFFER_H__

#include "ndBrainStdafx.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainVector.h"

class ndBrainGpuFloatBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt64 size, ndDeviceBufferType deviceType = ndGpuOnly);
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input, ndDeviceBufferType deviceType = ndGpuOnly);

	virtual void LoadData(size_t sizeInBytes, const void* const inputData) override;
	virtual void UnloadData(size_t sizeInBytes, void* const outputData) const override;

	ndBrainVector m_buffer;
};

#endif