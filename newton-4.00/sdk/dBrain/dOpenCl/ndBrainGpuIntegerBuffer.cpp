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
#include "ndBrainMatrix.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuIntegerBuffer.h"

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainContext* const context, ndInt64 sizeInElements, bool memoryMapped)
	:ndBrainGpuBuffer(context, sizeInElements * ndInt64(sizeof(ndUnsigned32)), memoryMapped)
{
}

ndBrainGpuIntegerBuffer::ndBrainGpuIntegerBuffer(ndBrainContext* const context, ndInt64 numberOfElements, const ndUnsigned32* const indexArray, bool memoryMapped)
	:ndBrainGpuBuffer(context, numberOfElements * ndInt64(sizeof(ndUnsigned32)), memoryMapped)
{
	ndAssert(m_context->GetAsGpuContext());
	MemoryToDevice(0, m_sizeInBytes, indexArray);
}


