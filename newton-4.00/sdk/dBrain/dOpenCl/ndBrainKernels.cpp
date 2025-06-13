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

// feed forward kernels
const char* ndBrainGpuContext::m_commonKernelsSource =
R""""(

    #define ND_GPU_LOCAL_BUFFER_SIZE	512

    typedef struct
    {
        uint m_inputSize;
	    uint m_outputSize;
	    uint m_inputOutputSize;
	    uint m_inputOutputStartOffset;
	    uint m_parametersBatchSize;
	    uint m_parametersStartOffset;
	    uint m_unused[4];
    } UniformBufferLayerArguments;

    typedef struct
    {
		float m_beta;
		float m_alpha;
		float m_epsilon;
		float m_betaAcc;
		float m_alphaAcc;
		float m_learnRate;
		float m_invBeta;
		float m_invAlpha;
		float m_decayRegularizer;
		uint m_minibatchSize;
    } UniformBufferOptimizerArguments;


    uint CalculateWorkGroupRoundoff(uint value, uint workgroupSize) 
    {
        return (value + workgroupSize - 1) & -(int)workgroupSize;
    }

)"""";

const char* ndBrainGpuContext::m_feedForwardKernels_1 =
R""""(

    __kernel void brainCopyInput(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* inputBuffer)
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

    __kernel void brainCopyOutput(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* outputBuffer) 
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

)"""";

const char* ndBrainGpuContext::m_feedForwardKernels_2 =
R""""(

    __kernel void brainLayerMatrixTimeVector_old(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* weightsAndBias) 
    {
        __local float cachedInput [ND_GPU_LOCAL_BUFFER_SIZE * 2];
        __local float cachedOutput [ND_GPU_LOCAL_BUFFER_SIZE * 2];
        __local float reductionBuffer [ND_GPU_LOCAL_BUFFER_SIZE];
        
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
        uint outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
        }
        
        // copy the matrix bias
        for (uint i = 0; i < outputSize; i += workGroupSize)
        {
            cachedOutput[i + itemId] = weightsAndBias[biasOffset + itemId];
        }
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
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
        
        for (uint i = 0; i < outputSize; i += workGroupSize)
        {
            inputOutputData[outputOffset + i + itemId] = cachedOutput[i + itemId];
        }
    }

    __kernel void brainLayerMatrixTimeVector(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* weightsAndBias) 
    {
        __local float cachedInput[ND_GPU_LOCAL_BUFFER_SIZE * 2];

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
        uint outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupInputSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupInputSize = inputSize - workGroupInputSizeReminder;
        for (uint i = 0; i < modWorkGroupInputSize; i += workGroupSize)
        {
            cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
        }
        for (uint itemId = 0; itemId < workGroupInputSizeReminder; ++itemId)
        {
            cachedInput[modWorkGroupInputSize + itemId] = inputOutputData[inputOffset + itemId + modWorkGroupInputSize];
        }
       
        // copy the matrix bias 
        float accumulator = 0.0f;
        uint workGroupSizeReminder = outputSize % workGroupSize;
        if (itemId < workGroupSizeReminder)
        {
            accumulator = weightsAndBias[biasOffset + itemId];
        }
        //barrier(CLK_LOCAL_MEM_FENCE); 
       
        for (uint i = 0; i < inputSize; ++i)
        {
            float scaleScale = cachedInput[i];

            uint rowStartOffset = i * outputSize + parametersStartOffset;
            if (itemId < workGroupSizeReminder)
            {
                float matrixElement = weightsAndBias[rowStartOffset + itemId];
                accumulator += matrixElement * scaleScale;
            }
        }
        
        inputOutputData[outputOffset + itemId] = accumulator;
    }

    __kernel void brainLayerReluActivation(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        //uint workGroupSizeReminder = inputSize % workGroupSize;
        //uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = (inputValue >= 0.0f) ? inputValue : 0.0f;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        //if (itemId < workGroupSizeReminder)
        //{
        //    float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        //    float outputValue = (inputValue >= 0.0f) ? inputValue : 0.0f;
        //    inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        //}
    }

    __kernel void brainLayerTanhActivation(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        //uint workGroupSizeReminder = inputSize % workGroupSize;
        //uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = (inputValue > -30.0f) ? ((inputValue < 30.0f) ? inputValue : 30.0f) : -30.0f;
            inputOutputData[outputOffset + i + itemId] = tanh(outputValue);
        }
        //if (itemId < workGroupSizeReminder)
        //{
        //    float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        //    float outputValue = (inputValue > -30.0f) ? ((inputValue < 30.0f) ? inputValue : 30.0f) : -30.0f;
        //    inputOutputData[outputOffset + modWorkGroupSize + itemId] = tanh(outputValue);
        //}
    }

    __kernel void brainLayerDropOutActivation(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        //uint workGroupSizeReminder = inputSize % workGroupSize;
        //uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = inputValue;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        //if (itemId < workGroupSizeReminder)
        //{
        //    float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        //    float outputValue = inputValue;
        //    inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        //}
    }
)"""";

const char* ndBrainGpuContext::m_feedForwardKernels_3 =
R""""(

    __kernel void brainLayerSoftmaxActivation(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* notUsed)
    {
        __local float tmpInputBuffer [ND_GPU_LOCAL_BUFFER_SIZE * 2];
        __local float reductionBuffer [ND_GPU_LOCAL_BUFFER_SIZE];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        uint inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        uint outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

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
        //for (uint i = 0; i < inputSize; i += workGroupSize)
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

const char* ndBrainGpuContext::m_backPropagateKernels_1 =
R""""(

    __kernel void brainCopyInputGradients(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* notUsed, 
            __global float* miniBatchGradients, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint dstBase = groupId * inputSize;
        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputOutputGradients[srcBase + i + itemId];
            miniBatchGradients[dstBase + i + itemId] = a;
        }
        if (itemId < workGroupSizeReminder)
        {
            float a = inputOutputGradients[srcBase + modWorkGroupSize + itemId];
            miniBatchGradients[dstBase + modWorkGroupSize + itemId] = a;
        }
    }

    __kernel void brainCopyOutputGradients(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* notUsed, 
            __global float* miniBatchGradients, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint outputSize = parameters->m_outputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        uint srcBase = groupId * outputSize;        
        uint dstBase = groupId * inputOutputSize + inputOutputStartOffset;
        
        uint workGroupSizeReminder = outputSize % workGroupSize;
        uint modWorkGroupSize = outputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = miniBatchGradients[srcBase + i + itemId];
            inputOutputGradients[dstBase + i + itemId] = a;
        }
        if (itemId < workGroupSizeReminder)
        {
            float a = miniBatchGradients[srcBase + modWorkGroupSize + itemId];
            inputOutputGradients[dstBase + modWorkGroupSize + itemId] = a;
        }
    }

)"""";

const char* ndBrainGpuContext::m_backPropagateKernels_2 =
R""""(

    __kernel void brainLayerBrainReluBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* notUsed, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        uint dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        //uint workGroupSizeReminder = inputSize % workGroupSize;
        //uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            float inpuData = inputOutputData[srcBase + i + itemId];
            float gradient = (inpuData >= 0.0f) ? 1.0f : 0.0f;
            float outputGrad = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
        }
        //if (itemId < workGroupSizeReminder)
        //{
        //    float inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
        //    float gradient = (inpuData >= 0.0f) ? 1.0f : 0.0f;
        //    float outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        //}
    }

    __kernel void brainLayerBrainTanhBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* notUsed, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        uint dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        //uint workGroupSizeReminder = inputSize % workGroupSize;
        //uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            float outputData = inputOutputData[dstBase + i + itemId];
            float a = 1.0f - outputData * outputData;
            float b = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = a * b;
        }
        //for (uint itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    float outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
        //    float a = 1.0f - outputData * outputData;
        //    float b = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a * b;
        //}
    }

    __kernel void brainLayerBrainCathegoricalSoftmaxBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* notUsed, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        uint dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        //uint workGroupSizeReminder = inputSize % workGroupSize;
        //uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[dstBase + i + itemId];
            float gradient = inputValue - inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = gradient;
        }
        //if (itemId < workGroupSizeReminder)
        //{
        //    float a = inputOutputData[dstBase + modWorkGroupSize + itemId];
        //    a -= inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a;
        //}
    }

    __kernel void brainLayerBrainDropOutBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* notUsed, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;        
        uint dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        //uint workGroupSizeReminder = inputSize % workGroupSize;
        //uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            float outputGrad = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = outputGrad;
        }
        //if (itemId < workGroupSizeReminder)
        //{
        //    float outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = outputGrad;
        //}
    }

)"""";

const char* ndBrainGpuContext::m_backPropagateKernels_3 =
R""""(

    //__kernel void brainLayerBrainLinearBackPropagate_old(
    //        __global const UniformBufferLayerArguments* parameters, 
    //        __global float* inputOutputData, 
    //        __global float* weightAndBias, 
    //        __global float* inputOutputGradients,
    //        __global float* weightAndBiasGradients) 
    //{
    //    uint itemId = get_local_id(0);
    //    uint groupId = get_group_id(0);
    //    uint workGroupSize = get_local_size(0);
    //    
    //    uint inputSize = parameters->m_inputSize;
    //    uint outputSize = parameters->m_outputSize;
    //    uint inputOutputSize = parameters->m_inputOutputSize;
    //    uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
    //
    //    __local float cachedInput[ND_GPU_LOCAL_BUFFER_SIZE * 2];
    //    //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
    //    for (uint i = 0; i < inputSize; i += workGroupSize)
    //    {
    //        cachedInput[i + itemId] = 0.0f;
    //    }
    //    //for (uint itemId = 0; itemId < workGroupSizeReminder; ++itemId)
    //    //{
    //    //    cachedInput[modWorkGroupSize + itemId] = 0.0f;
    //    //}
    //    
    //    uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
    //    uint dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
    //    uint parametersStartOffset = parameters->m_parametersStartOffset;
    //
    //    uint workGroupSizeReminder = inputSize % workGroupSize;
    //    uint modWorkGroupSize = inputSize - workGroupSizeReminder;
    //    
    //    // calculate input gradients
    //    for (uint j = 0; j < outputSize; ++j)
    //    {
    //        float scale = inputOutputGradients[dstBase + j];
    //        uint weightOffset = j * inputSize + parametersStartOffset;
    //        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
    //        {
    //            float weight = weightAndBias[weightOffset + i + itemId];
    //            cachedInput[i + itemId] += weight * scale;
    //        }
    //        if(itemId < workGroupSizeReminder)
    //        {
    //           float weight = weightAndBias[weightOffset + modWorkGroupSize + itemId];
    //           cachedInput[modWorkGroupSize + itemId] += weight * scale;
    //        }
    //    }
    //    
    //    //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
    //    for (uint i = 0; i < inputSize; i += workGroupSize)
    //    {
    //        inputOutputGradients[srcBase + i + itemId] = cachedInput[i + itemId];
    //        cachedInput[i + itemId] = inputOutputData[srcBase + i + itemId];
    //    }
    //    //if(itemId < workGroupSizeReminder)
    //    //{
    //    //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = cachedInput[modWorkGroupSize + itemId];
    //    //    cachedInput[modWorkGroupSize + itemId] = inputOutputData[srcBase + modWorkGroupSize + itemId];
    //    //}
    //    
    //    // calculate weights and bias gradients
    //    uint matrixSize = inputSize * outputSize;
    //    uint workGroupOuputSizeReminder = outputSize % workGroupSize;
    //    uint modWorkGroupOuputSize = outputSize - workGroupOuputSizeReminder;
    //    uint weightAndBiasGradientOffset = groupId * parameters->m_parametersBatchSize + parametersStartOffset;
    //    
    //    // calculate bias Gradient
    //    //for (uint i = 0; i < modWorkGroupOuputSize; i += workGroupSize)
    //    for (uint j = 0; j < outputSize; j += workGroupSize)
    //    {
    //        float a = inputOutputGradients[dstBase + i + itemId];
    //        weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + i + itemId] = a;
    //    }
    //    //if(itemId < workGroupOuputSizeReminder)
    //    //{
    //    //    float a = inputOutputGradients[dstBase + modWorkGroupOuputSize + itemId];
    //    //    weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + modWorkGroupOuputSize + itemId] = a;
    //    //}
    //    
    //    // calculate matrix weight Gradient
    //    for (uint j = 0; j < outputSize; ++j)
    //    {
    //        float scale = inputOutputGradients[dstBase + j];
    //        uint weightRowOffset = j * inputSize + weightAndBiasGradientOffset;
    //
    //        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
    //        {
    //            float inputValue = cachedInput[i + itemId];
    //            float weightGradient = inputValue * scale;
    //            weightAndBiasGradients[weightRowOffset + i + itemId] = weightGradient;
    //        }
    //        if(itemId < workGroupSizeReminder)
    //        {
    //            float inputValue = cachedInput[modWorkGroupSize + itemId];
    //            float weightGradient = inputValue * scale;
    //            weightAndBiasGradients[weightRowOffset + modWorkGroupSize + itemId] = weightGradient;
    //        }
    //    }
    //}

    // optimizations pending.
    __kernel void brainLayerBrainLinearBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightAndBias, 
            __global float* inputOutputGradients,
            __global float* weightAndBiasGradients) 
    {
        __local float cachedInput[ND_GPU_LOCAL_BUFFER_SIZE * 2];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint outputSize = parameters->m_outputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            cachedInput[i + itemId] = 0.0f;
        }
        //for (uint itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    cachedInput[modWorkGroupSize + itemId] = 0.0f;
        //}
        
        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        uint dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        // calculate input gradients
        for (uint j = 0; j < outputSize; ++j)
        {
            float scale = inputOutputGradients[dstBase + j];
            uint weightOffset = j * inputSize + parametersStartOffset;
            for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
            {
                float weight = weightAndBias[weightOffset + i + itemId];
                cachedInput[i + itemId] += weight * scale;
            }
            if(itemId < workGroupSizeReminder)
            {
               float weight = weightAndBias[weightOffset + modWorkGroupSize + itemId];
               cachedInput[modWorkGroupSize + itemId] += weight * scale;
            }
        }
        
        //for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            inputOutputGradients[srcBase + i + itemId] = cachedInput[i + itemId];
            cachedInput[i + itemId] = inputOutputData[srcBase + i + itemId];
        }
        //if(itemId < workGroupSizeReminder)
        //{
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = cachedInput[modWorkGroupSize + itemId];
        //    cachedInput[modWorkGroupSize + itemId] = inputOutputData[srcBase + modWorkGroupSize + itemId];
        //}
        
        // calculate weights and bias gradients
        uint matrixSize = inputSize * outputSize;

        uint workGroupOuputSizeReminder = outputSize % workGroupSize;
        uint modWorkGroupOuputSize = outputSize - workGroupOuputSizeReminder;

        uint weightAndBiasGradientOffset = groupId * parameters->m_parametersBatchSize + parametersStartOffset;
        
        // calculate bias Gradient
        //for (uint i = 0; i < modWorkGroupOuputSize; i += workGroupSize)
        for (uint i = 0; i < outputSize; i += workGroupSize)
        {
            float a = inputOutputGradients[dstBase + i + itemId];
            weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + i + itemId] = a;
        }
        //if(itemId < workGroupOuputSizeReminder)
        //{
        //    float a = inputOutputGradients[dstBase + modWorkGroupOuputSize + itemId];
        //    weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + modWorkGroupOuputSize + itemId] = a;
        //}
        
        // calculate matrix weight Gradient
        for (uint j = 0; j < outputSize; ++j)
        {
            float scale = inputOutputGradients[dstBase + j];
            uint weightRowOffset = j * inputSize + weightAndBiasGradientOffset;
        
            for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
            {
                float inputValue = cachedInput[i + itemId];
                float weightGradient = inputValue * scale;
                weightAndBiasGradients[weightRowOffset + i + itemId] = weightGradient;
            }
            if(itemId < workGroupSizeReminder)
            {
                float inputValue = cachedInput[modWorkGroupSize + itemId];
                float weightGradient = inputValue * scale;
                weightAndBiasGradients[weightRowOffset + modWorkGroupSize + itemId] = weightGradient;
            }
        }
    }

)"""";

const char* ndBrainGpuContext::m_optimizerKernels =
R""""(

    //void Execute(uint groupId, uint workGroupSize)
    __kernel void brainAdamMomentumUpdate(__global UniformBufferOptimizerArguments* parameters) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        
        //ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        //ndBrainOptimizerAdamGpu::ndCommandShareInfo* const parameters = (ndBrainOptimizerAdamGpu::ndCommandShareInfo*)&buffer0->m_data[0];

        if (itemId == 0)
        {
            parameters->m_betaAcc *= parameters->m_beta;
            parameters->m_alphaAcc *= parameters->m_alpha;
            if (parameters->m_betaAcc < 1.0e-6f)
            {
                parameters->m_betaAcc = 0.0f;
            }
            if (parameters->m_alphaAcc < 1.0e-6f)
            {
                parameters->m_alphaAcc = 0.0f;
            }
        }
    }

    __kernel void brainAccumulateGradients(
            __global const UniformBufferLayerArguments* parameters,
            __global float* gradientBuffer)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        //ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        //ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        //UniformBufferLayerArguments* const parameters = (UniformBufferLayerArguments*)&buffer0->m_data[0];
        //float* const gradientBuffer = &buffer1->m_buffer[0];

        uint inputSize = parameters->m_inputSize;
        uint miniBatchSize = parameters->m_inputOutputSize;
        
        uint start = groupId * workGroupSize;
        float weightFactor = 1.0f / (float)workGroupSize;
        
        float sum = 0.0f;
        for (uint j = 0; j < workGroupSize; ++j)
        {
            uint base = start + j * inputSize;
            sum += gradientBuffer[base + itemId];
        }
        gradientBuffer[start + itemId] = sum * weightFactor;
    }

    __kernel void brainAdamUpdateLassoRegularizer(
        __global const UniformBufferOptimizerArguments* parameters,
        __global float* weightAndBiasBuffer, __global float* weightAndBiasGradientBuffer,
        __global float* vdw, __global float* vdw2)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        float descendRate = -parameters->m_learnRate;
        float regularizer = -parameters->m_decayRegularizer;

        uint start = groupId * workGroupSize;

        for (uint itemId = 0; itemId < workGroupSize; ++itemId)
        {
            float temp = weightAndBiasGradientBuffer[start + itemId];
            float a = vdw[start + itemId] * parameters->m_alpha + temp * (1.0f - parameters->m_alpha);
            vdw[start + itemId] = a;
            
            // calcuate RMS
            float b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (1.0f - parameters->m_beta);
            vdw2[start + itemId] = b;
        
            float vdwCorrected = a * parameters->m_invAlpha;
            float vdw2Corrected = b * parameters->m_invBeta;
            
            float bias_den = 1.0f / (sqrt(vdw2Corrected) + parameters->m_epsilon);
            float gradient = vdwCorrected * bias_den;
             
            float weight = weightAndBiasBuffer[start + itemId];
            float lassoRegularizer = (weight >= 0.0f) ? regularizer : -regularizer;
            gradient += weight * lassoRegularizer;
            weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
        }
    }

    __kernel void brainAdamUpdateRidgeRegularizer(
        __global const UniformBufferOptimizerArguments* parameters,
        __global float* weightAndBiasBuffer, __global float* weightAndBiasGradientBuffer,
        __global float* vdw, __global float* vdw2)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        float descendRate = -parameters->m_learnRate;
        float regularizer = -parameters->m_decayRegularizer;

        uint start = groupId * workGroupSize;
        for (uint itemId = 0; itemId < workGroupSize; ++itemId)
        {
            float temp = weightAndBiasGradientBuffer[start + itemId];
            float a = vdw[start + itemId] * parameters->m_alpha + temp * (1.0f - parameters->m_alpha);
            vdw[start + itemId] = a;
            
            // calcuate RMS
            float b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (1.0f - parameters->m_beta);
            vdw2[start + itemId] = b;
        
            float vdwCorrected = a * parameters->m_invAlpha;
            float vdw2Corrected = b * parameters->m_invBeta;
            
            float bias_den = 1.0f / (sqrt(vdw2Corrected) + parameters->m_epsilon);
            float gradient = vdwCorrected * bias_den;
             
            float weight = weightAndBiasBuffer[start + itemId];
            gradient += weight * regularizer;
            weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
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

#include <vector>
void ndBrainGpuContext::CreateKerners()
{
    cl_int errcode_ret = 0;
    std::string source(m_commonKernelsSource);
    source += m_optimizerKernels;
    source += m_feedForwardKernels_1;
    source += m_feedForwardKernels_2;
    source += m_feedForwardKernels_3;
    source += m_backPropagateKernels_1;
    source += m_backPropagateKernels_2;
    source += m_backPropagateKernels_3;

    cl::Program program (**m_context, source, CL_TRUE, &errcode_ret);
    ndAssert(errcode_ret == 0);

#if 0
    // this only seems to work with nvidia PTX        
    size_t bin_sz;
    errcode_ret = clGetProgramInfo(program.get(), CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &bin_sz, NULL);
    
    unsigned char* code = (unsigned char*)malloc(bin_sz);
    errcode_ret = clGetProgramInfo(program.get(), CL_PROGRAM_BINARIES, sizeof(unsigned char*), &code, NULL);
    free(code);
#endif

    // create all feed foward shaders
    m_ndBrainCopyInput = CreateKerner(program, "brainCopyInput");
    m_ndBrainCopyOutput = CreateKerner(program, "brainCopyOutput");
    m_ndBrainLayerLinear = CreateKerner(program, "brainLayerMatrixTimeVector");
    m_ndBrainLayerReluActivation = CreateKerner(program, "brainLayerReluActivation");
    m_ndBrainLayerTanhActivation = CreateKerner(program, "brainLayerTanhActivation");
    m_ndBrainLayerSoftmaxActivation = CreateKerner(program, "brainLayerSoftmaxActivation");
    m_ndBrainLayerDropOutActivation = CreateKerner(program, "brainLayerDropOutActivation");

    // create all backpropagate shaders
    m_ndBrainCopyInputGradients = CreateKerner(program, "brainCopyInputGradients");
    m_ndBrainCopyOutputGradients = CreateKerner(program, "brainCopyOutputGradients");
    m_ndBrainLayerReluBackPropagate = CreateKerner(program, "brainLayerBrainReluBackPropagate");
    m_ndBrainLayerTanhBackPropagate = CreateKerner(program, "brainLayerBrainTanhBackPropagate");
    m_ndBrainLayerLinearBackPropagate = CreateKerner(program, "brainLayerBrainLinearBackPropagate");
    m_ndBrainLayerDropOutBackPropagate = CreateKerner(program, "brainLayerBrainDropOutBackPropagate");
    m_ndBrainLayerCathegoricalSoftmaxBackPropagate = CreateKerner(program, "brainLayerBrainCathegoricalSoftmaxBackPropagate");

    // accumulate gradient kernels
    m_ndBrainAccumulateGradients = CreateKerner(program, "brainAccumulateGradients");

    // optimizer kernels
    m_ndBrainAdamMomentumUpdate = CreateKerner(program, "brainAdamMomentumUpdate");
    m_ndBrainAdamRidgeOptimizerUpdate = CreateKerner(program, "brainAdamUpdateRidgeRegularizer");
    m_ndBrainAdamLassoOptimizerUpdate = CreateKerner(program, "brainAdamUpdateLassoRegularizer");
}