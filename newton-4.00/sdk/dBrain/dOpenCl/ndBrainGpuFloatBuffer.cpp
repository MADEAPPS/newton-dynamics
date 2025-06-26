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
#include "ndBrainGpuFloatBuffer.h"

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, ndInt64 sizeInElements)
	:ndBrainGpuBuffer(context, sizeInElements * ndInt64(sizeof(ndReal)), ndStorageData)
{
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainVector& input)
	:ndBrainGpuBuffer(context, input.GetCount() * ndInt64(sizeof(ndReal)), ndStorageData)
{
	ndAssert(m_context->GetAsGpuContext());
	BrainVectorToDevice(input);
}

ndBrainGpuFloatBuffer::ndBrainGpuFloatBuffer(ndBrainContext* const context, const ndBrainMatrix& matrix)
	:ndBrainGpuBuffer(context, matrix.GetColumns() * matrix.GetRows() * ndInt64(sizeof(ndReal)), ndStorageData)
{
	ndBrainVector flatArray;
	ndAssert(m_context->GetAsGpuContext());
	for (ndInt32 i = 0; i < matrix.GetRows(); i++)
	{
		for (ndInt32 j = 0; j < matrix.GetColumns(); j++)
		{
			flatArray.PushBack(matrix[i][j]);
		}
	}
	BrainVectorToDevice(flatArray);
}
