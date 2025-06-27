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

#include "ndBrainGpuBuffer.h"

class ndBrainGpuFloatBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuFloatBuffer(ndBrainContext* const context, ndInt64 sizeInElements, bool memoryMapped = false);
	ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input, bool memoryMapped = false);
	ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix, bool memoryMapped = false);
};

#endif