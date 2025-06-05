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
    }  UniformBufferObject;

    __kernel void brainCopyInput(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* inputBuffer)
    {                                                                      
        size_t itemId = get_local_id(0);
        size_t groupId = get_group_id(0);
        size_t workGroupSize = get_local_size(0);

        size_t inputSize = parameters->m_inputSize;
        size_t inputOutputSize = parameters->m_inputOutputSize;
        size_t inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        size_t srcBase = groupId * inputSize;
        size_t dstBase = groupId * inputOutputSize + inputOutputStartOffset;
        
        size_t workGroupSizeReminder = inputSize % workGroupSize;
        size_t modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputBuffer[srcBase + i + itemId];
            inputOutputData[dstBase + i + itemId] = a;
        }
        if (itemId < workGroupSizeReminder)
        {
            float a = inputBuffer[srcBase + modWorkGroupSize + itemId];
            inputOutputData[dstBase + modWorkGroupSize + itemId] = a;
        }
    }

    __kernel void brainCopyOutput(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* outputBuffer) 
    {
        size_t itemId = get_local_id(0);
        size_t groupId = get_group_id(0);
        size_t workGroupSize = get_local_size(0);

        size_t outputSize = parameters->m_outputSize;
        size_t inputOutputSize = parameters->m_inputOutputSize;
        size_t inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        size_t dstBase = groupId * outputSize;
        size_t srcBase = groupId * inputOutputSize + inputOutputStartOffset;

        size_t workGroupSizeReminder = outputSize % workGroupSize;
        size_t modWorkGroupSize = outputSize - workGroupSizeReminder;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputOutputData[srcBase + itemId];
            outputBuffer[dstBase + i + itemId] = a;
        }
        
        if (itemId < workGroupSizeReminder)
        {
            float a = inputOutputData[srcBase + modWorkGroupSize + itemId];
            outputBuffer[dstBase + modWorkGroupSize + itemId] = a;
        }
    }

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    __kernel void brainLayerLinear(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* weightsAndBias) 
    {
        __local float cachedInput [1024 * 2];
        __local float cachedOutput [1024 * 2];
        __local float reductionBuffer [1024];

        size_t itemId = get_local_id(0);
        size_t groupId = get_group_id(0);
        size_t workGroupSize = get_local_size(0);

        size_t inputSize = parameters->m_inputSize;
        size_t outputSize = parameters->m_outputSize;
        size_t inputOutputSize = parameters->m_inputOutputSize;
        size_t parametersStartOffset = parameters->m_parametersStartOffset;
        size_t inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        size_t biasOffset = outputSize * inputSize + parametersStartOffset;
        size_t inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        size_t outputOffset = inputOffset + inputSize;
        
        size_t workGroupSizeReminder = inputSize % workGroupSize;
        size_t modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (size_t i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
        }
        if (itemId < workGroupSizeReminder)
        {
            cachedInput[modWorkGroupSize + itemId] = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        }
        
        size_t rowCountReminder = outputSize % workGroupSize;
        size_t modRowCount = outputSize - rowCountReminder;

        for (size_t i = 0; i < modRowCount; i += workGroupSize)
        {
            cachedOutput[i + itemId] = weightsAndBias[biasOffset + itemId];
        }
        if (itemId < rowCountReminder)
        {
            cachedOutput[modRowCount + itemId] = weightsAndBias[biasOffset + modRowCount + itemId];
        }
        //barrier(CLK_LOCAL_MEM_FENCE); 

        for (size_t i = 0; i < outputSize; ++i)
        {
            float partialSum = 0.0f;
            size_t rowStartOffset = i * inputSize + parametersStartOffset;
            for (size_t j = 0; j < modWorkGroupSize; j += workGroupSize)
            {
                float b = cachedInput[itemId + j];
                float a = weightsAndBias[rowStartOffset + itemId + j];
                partialSum += a * b;
            }
            if (itemId < workGroupSizeReminder)
            {
                float b = cachedInput[modWorkGroupSize + itemId];
                float a = weightsAndBias[rowStartOffset + modWorkGroupSize + itemId];
                partialSum += a * b;
            }

            for (size_t j = workGroupSize / 2; j > 0; j = j >> 1)
            {
                barrier(CLK_LOCAL_MEM_FENCE); 
                if ((itemId >= j) && (itemId < j * 2))
                {
                    reductionBuffer[itemId - j] = partialSum;
                }

                barrier(CLK_LOCAL_MEM_FENCE); 
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
        barrier(CLK_LOCAL_MEM_FENCE); 
        
        for (size_t i = 0; i < modRowCount; i += workGroupSize)
        {
            inputOutputData[outputOffset + i + itemId] = cachedOutput[i + itemId];
        }
        if (itemId < rowCountReminder)
        {
            inputOutputData[outputOffset + modRowCount + itemId] = cachedOutput[modRowCount + itemId];
        }
    }

    __kernel void brainLayerLinearDropOutActivation(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        //uint outputSize = parameters->m_outputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        size_t workGroupSizeReminder = inputSize % workGroupSize;
        size_t modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = itemId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + inputSize;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = inputValue;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + groupId];
            float outputValue = inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
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
    m_ndBrainLayerLinearDropOutActivation = CreateKerner(program, "brainLayerLinearDropOutActivation");
}