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
#include "ndBrainGpuContext.h"

const char* ndBrainGpuContext::m_kernelSource =
R""""(

    typedef struct
    {
        uint m_inputSize;
	    uint m_outputSize;
	    uint m_parametersBatchSize;
	    uint m_parametersStartOffset;
	    uint m_inputOutputSize;
	    uint m_inputOutputStartOffset;
	    uint m_unused[4];
    }  UniformBufferObject ;

    __kernel void brainCopyInput(__local const UniformBufferObject* parameters, __global float* inputBuffer, __global float* inputOutputData)
    {                                                                      
        size_t itemId = get_local_id(0);
        size_t groupId = get_group_id(0);
        size_t workGroupSize = get_local_size(0);
        
        size_t srcBase = groupId * parameters->m_inputSize;
        size_t dstBase = groupId * parameters->m_inputOutputSize + parameters->m_inputOutputStartOffset;
        
        size_t iterations = parameters->m_inputSize / workGroupSize;
        size_t modWorkGroupSize = iterations * workGroupSize;
        size_t reminderworkGroupSize = parameters->m_inputSize - modWorkGroupSize;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputBuffer[srcBase + i + itemId];
            inputOutputData[dstBase + i + itemId] = a;
        }
        if (itemId < reminderworkGroupSize)
        {
            float a = inputBuffer[srcBase + modWorkGroupSize + itemId];
            inputOutputData[dstBase + modWorkGroupSize + itemId] = a;
        }
    }

    __kernel void brainCopyOutput(__local const UniformBufferObject* parameters, __global float* outputBuffer, __global float* inputOutputData) 
    {
        size_t itemId = get_local_id(0);
        size_t groupId = get_group_id(0);
        size_t workGroupSize = get_local_size(0);
        
        uint dstBase = groupId * parameters->m_outputSize;
        uint srcBase = groupId * parameters->m_inputOutputSize + parameters->m_inputOutputStartOffset;
        
        uint iterations = parameters->m_outputSize / workGroupSize;
        uint modWorkGroupSize = iterations * workGroupSize;
        uint reminderworkGroupSize = parameters->m_outputSize - modWorkGroupSize;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputOutputData[srcBase + itemId];
            outputBuffer[dstBase + i + itemId] = a;
        }
        if (itemId < reminderworkGroupSize)
        {
            float a = inputOutputData[srcBase + modWorkGroupSize + itemId];
            outputBuffer[dstBase + modWorkGroupSize + itemId] = a;
        }
    }

)"""";


ndSharedPtr<ndBrainGpuShader> ndBrainGpuContext::CreateKerner(const cl::Program& program, const char* const functionMame) const
{
    cl_int errcode_ret = 0;
    ndSharedPtr<ndBrainGpuShader> kernel(ndSharedPtr<ndBrainGpuShader>(new ndBrainGpuShader(program, functionMame, &errcode_ret)));
    ndAssert(errcode_ret == 0);
    return kernel;
}

void ndBrainGpuContext::CreateKerners()
{
    cl_int errcode_ret = 0;
    const std::string source(m_kernelSource);
    cl::Program program (**m_context, source, CL_TRUE, &errcode_ret);
    ndAssert(errcode_ret == 0);
    m_ndBrainCopyInput = CreateKerner(program, "brainCopyInput");
    m_ndBrainCopyOutput = CreateKerner(program, "brainCopyOutput");
}