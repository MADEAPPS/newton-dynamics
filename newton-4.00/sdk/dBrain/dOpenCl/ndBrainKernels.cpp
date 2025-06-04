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

    #define WORKGROUP_SIZE  256

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

    __kernel void brainCopyInput(__local const UniformBufferObject* parameters, __global float* inputOutputData, __global float* inputBuffer)
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

    __kernel void brainCopyOutput(__local const UniformBufferObject* parameters, __global float* inputOutputData, __global float* outputBuffer) 
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

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    __kernel void brainLayerLinear(__local const UniformBufferObject* parameters, __global float* inputOutputData, __global float* weightsAndBias) 
    {
        __local float cachedInput [1024 * 2];
        __local float cachedOutput [1024 * 2];
        __local float reductionBuffer [WORKGROUP_SIZE / 2];

        size_t itemId = get_local_id(0);
        size_t groupId = get_group_id(0);
        size_t workGroupSize = get_local_size(0);

        size_t biasOffset = parameters->m_outputSize * parameters->m_inputSize + parameters->m_parametersStartOffset;
        size_t inputOffset = groupId * parameters->m_inputOutputSize + parameters->m_inputOutputStartOffset;
        size_t outputOffset = inputOffset + parameters->m_inputSize;
        
        size_t workGroupCount = parameters->m_inputSize / workGroupSize;
        size_t modWorkGroupSize = workGroupCount * workGroupSize;
        size_t reminderworkGroupSize = parameters->m_inputSize - modWorkGroupSize;

        for (size_t i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
        }
        if (itemId < reminderworkGroupSize)
        {
            cachedInput[modWorkGroupSize + itemId] = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        }
        
        size_t roundRowCount = parameters->m_outputSize / workGroupSize;
        size_t modRowCount = roundRowCount * workGroupSize;
        size_t reminderRowCount = parameters->m_outputSize - modRowCount;
        for (size_t i = 0; i < modRowCount; i += workGroupSize)
        {
            cachedOutput[i + itemId] = weightsAndBias[biasOffset + itemId];
        }
        if (itemId < reminderRowCount)
        {
            cachedOutput[modRowCount + itemId] = weightsAndBias[biasOffset + modRowCount + itemId];
        }
        
        for (size_t i = 0; i < parameters->m_outputSize; ++i)
        {
            float partialSum = 0.0f;
            size_t rowStartOffset = i * parameters->m_inputSize + parameters->m_parametersStartOffset;
            for (size_t j = 0; j < modWorkGroupSize; j += workGroupSize)
            {
                float b = cachedInput[itemId + j];
                float a = weightsAndBias[rowStartOffset + itemId + j];
                partialSum += a * b;
            }
            if (itemId < reminderworkGroupSize)
            {
                float b = cachedInput[modWorkGroupSize + itemId];
                float a = weightsAndBias[rowStartOffset + modWorkGroupSize + itemId];
                partialSum += a * b;
            }
            
            for (size_t j = workGroupSize / 2; j > 0; j = j >> 1)
            {
                if ((itemId >= j) && (itemId < j * 2))
                {
                    reductionBuffer[itemId - j] = partialSum;
                }
                //memoryBarrierShared();
                //barrier();
                barrier(CLK_GLOBAL_MEM_FENCE); 
                if (itemId < j)
                {
                    partialSum += reductionBuffer[itemId];
                }
            }
            if (itemId == 0)
            {
                cachedOutput[i] += partialSum;
            }
        }
        
        for (size_t i = 0; i < modRowCount; i += workGroupSize)
        {
            inputOutputData[outputOffset + i + itemId] = cachedOutput[i + itemId];
        }
        if (itemId < reminderRowCount)
        {
            inputOutputData[outputOffset + modRowCount + itemId] = cachedOutput[modRowCount + itemId];
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
    m_ndBrainLayerLinear = CreateKerner(program, "brainLayerLinear");
}