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

const char* ndBrainGpuContext::m_kernelSource0 =
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
)"""";

const char* ndBrainGpuContext::m_kernelSource1 =
R""""(
    __kernel void brainCopyInput(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* inputBuffer)
    {                                                                      
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint srcBase = groupId * inputSize;
        uint dstBase = groupId * inputOutputSize + inputOutputStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
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
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint outputSize = parameters->m_outputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint dstBase = groupId * outputSize;
        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;

        uint workGroupSizeReminder = outputSize % workGroupSize;
        uint modWorkGroupSize = outputSize - workGroupSizeReminder;
        
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

    __kernel void brainCopyOutputGradients(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* outputBuffer) 
    {
        //uint itemId = get_local_id(0);
        //uint groupId = get_group_id(0);
        //uint workGroupSize = get_local_size(0);
        //
        //uint outputSize = parameters->m_outputSize;
        //uint inputOutputSize = parameters->m_inputOutputSize;
        //uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        //
        //uint dstBase = groupId * outputSize;
        //uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        //
        //uint workGroupSizeReminder = outputSize % workGroupSize;
        //uint modWorkGroupSize = outputSize - workGroupSizeReminder;
        //
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        //{
        //    float a = inputOutputData[srcBase + itemId];
        //    outputBuffer[dstBase + i + itemId] = a;
        //}
        //
        //if (itemId < workGroupSizeReminder)
        //{
        //    float a = inputOutputData[srcBase + modWorkGroupSize + itemId];
        //    outputBuffer[dstBase + modWorkGroupSize + itemId] = a;
        //}
    }

)"""";

const char* ndBrainGpuContext::m_kernelSource2 =
R""""(

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    __kernel void brainLayerLinear(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* weightsAndBias) 
    {
        __local float cachedInput [1024 * 2];
        __local float cachedOutput [1024 * 2];
        __local float reductionBuffer [1024];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint outputSize = parameters->m_outputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        uint biasOffset = outputSize * inputSize + parametersStartOffset;
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + inputSize;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
        }
        if (itemId < workGroupSizeReminder)
        {
            cachedInput[modWorkGroupSize + itemId] = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        }
        
        uint rowCountReminder = outputSize % workGroupSize;
        uint modRowCount = outputSize - rowCountReminder;

        for (uint i = 0; i < modRowCount; i += workGroupSize)
        {
            cachedOutput[i + itemId] = weightsAndBias[biasOffset + itemId];
        }
        if (itemId < rowCountReminder)
        {
            cachedOutput[modRowCount + itemId] = weightsAndBias[biasOffset + modRowCount + itemId];
        }

        for (uint i = 0; i < outputSize; ++i)
        {
            float partialSum = 0.0f;
            uint rowStartOffset = i * inputSize + parametersStartOffset;
            for (uint j = 0; j < modWorkGroupSize; j += workGroupSize)
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

            for (uint j = workGroupSize / 2; j > 0; j = j >> 1)
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
        
        for (uint i = 0; i < modRowCount; i += workGroupSize)
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
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + inputSize;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = inputValue;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            float outputValue = inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
)"""";

const char* ndBrainGpuContext::m_kernelSource3 =
R""""(

    __kernel void brainLayerReluActivation(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + inputSize;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = (inputValue >= 0.0) ? inputValue : 0.0f;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            float outputValue = (inputValue >= 0.0) ? inputValue : 0.0f;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }

    __kernel void brainLayerTanhActivation(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + inputSize;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = (inputValue > -30.0f) ? ((inputValue < 30.0f) ? inputValue : 30.0f) : -30.0f;
            inputOutputData[outputOffset + i + itemId] = tanh(outputValue);
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            float outputValue = (inputValue > -30.0f) ? ((inputValue < 30.0f) ? inputValue : 30.0f) : -30.0f;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = tanh(outputValue);
        }
    }

    __kernel void brainLayerSoftmaxActivation(__global const UniformBufferObject* parameters, __global float* inputOutputData, __global float* notUsed)
    {
        __local float tmpInputBuffer [1024 * 2];
        __local float reductionBuffer [1024];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + inputSize;
        
        // save input to local buffer, also get the max argument
        float maxArg = -1.0e30f;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            tmpInputBuffer[i + itemId] = inputValue;
            maxArg = (inputValue > maxArg) ? inputValue : maxArg;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            tmpInputBuffer[modWorkGroupSize + itemId] = inputValue;
            maxArg = (inputValue > maxArg) ? inputValue : maxArg;
        }
        
        for (uint j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE); 
            if ((itemId >= j) && (itemId < j * 2))
            {
                reductionBuffer[itemId - j] = maxArg;
            }

            barrier(CLK_LOCAL_MEM_FENCE); 
            if (itemId < j)
            {
                float inputValue = reductionBuffer[itemId];
                maxArg = (inputValue > maxArg) ? inputValue : maxArg;
            }
        }
        if (itemId == 0)
        {
            reductionBuffer[0] = maxArg;
        }
        barrier(CLK_LOCAL_MEM_FENCE); 
        
        maxArg = reductionBuffer[0];
        
        float sumArg = 0.0f;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = tmpInputBuffer[i + itemId] - maxArg;
            float outputValue = exp(inputValue);
            sumArg += outputValue;
            tmpInputBuffer[i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = tmpInputBuffer[modWorkGroupSize + itemId] - maxArg;
            float outputValue = exp(inputValue);
            sumArg += outputValue;
            tmpInputBuffer[modWorkGroupSize + itemId] = outputValue;
        }
        
        for (uint j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE); 
            if ((itemId >= j) && (itemId < j * 2))
            {
                reductionBuffer[itemId - j] = sumArg;
            }

            barrier(CLK_LOCAL_MEM_FENCE); 
            if (itemId < j)
            {
                float inputValue = reductionBuffer[itemId];
                sumArg += inputValue;
            }
        }
        if (itemId == 0)
        {
            reductionBuffer[0] = 1.0f / sumArg;
        }
        barrier(CLK_LOCAL_MEM_FENCE); 
        
        float invDen = reductionBuffer[0];
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = tmpInputBuffer[i + itemId];
            float outputValue = invDen * inputValue;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = tmpInputBuffer[modWorkGroupSize + itemId];
            float outputValue = invDen * inputValue;
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
    std::string source(m_kernelSource0);
    source += m_kernelSource1;
    source += m_kernelSource2;
    source += m_kernelSource3;

    cl::Program program (**m_context, source, CL_TRUE, &errcode_ret);
    ndAssert(errcode_ret == 0);

    m_ndBrainCopyInput = CreateKerner(program, "brainCopyInput");
    m_ndBrainCopyOutput = CreateKerner(program, "brainCopyOutput");
    m_ndBrainLayerLinear = CreateKerner(program, "brainLayerLinear");
    m_ndBrainCopyOutputGradients = CreateKerner(program, "brainCopyOutputGradients");
    m_ndBrainLayerReluActivation = CreateKerner(program, "brainLayerReluActivation");
    m_ndBrainLayerTanhActivation = CreateKerner(program, "brainLayerTanhActivation");
    m_ndBrainLayerSoftmaxActivation = CreateKerner(program, "brainLayerSoftmaxActivation");
    m_ndBrainLayerLinearDropOutActivation = CreateKerner(program, "brainLayerLinearDropOutActivation");
}