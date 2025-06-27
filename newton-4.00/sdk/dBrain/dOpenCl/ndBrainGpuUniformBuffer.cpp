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
#include "ndBrainGpuContext.h"
#include "ndBrainGpuUniformBuffer.h"

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, bool memoryMapped)
	:ndBrainGpuBuffer(context, sizeInBytes, memoryMapped)
{
}

ndBrainGpuUniformBuffer::ndBrainGpuUniformBuffer(ndBrainContext* const context, ndInt32 sizeInBytes, const void* const data, bool memoryMapped)
	:ndBrainGpuBuffer(context, sizeInBytes, memoryMapped)
{
	LoadData(0, size_t(sizeInBytes), data);
}
