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
#include "ndBrainKernel.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"

// feed forward kernels
const char* ndBrainGpuContext::m_commonKernelsInclude =
R""""(

    #define ND_GPU_TILED_MATRIX_ROWS_BITS       4
    #define ND_GPU_TILED_MATRIX_ROWS            (1<<ND_GPU_TILED_MATRIX_ROWS_BITS)

    #define ND_DEFAULT_WORKGROUP_SIZE           256
    #define ND_GPU_LOCAL_BUFFER_SIZE	        1024 * 4

    #define ND_GPU_LEAKY_LRU_GRADIENT		    0.01

    typedef struct
    {
        void* m_unUsed;
        uint m_inputSize;
        uint m_outputSize;
        uint m_inputOutputSize;
        uint m_inputOutputStartOffset;
        uint m_parametersBatchSize;
        uint m_parametersStartOffset;
        uint m_matrixDimensionK;
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
		float m_minibathScale;
        float m_decayRegularizer;
    } UniformBufferOptimizerArguments;

    typedef struct 
    {
	    uint m_strideInByte;
	    uint m_srcStrideInByte;
	    uint m_srcOffsetInByte;
	    uint m_dstStrideInByte;
	    uint m_dstOffsetInByte;
    } CopyBufferCommandInfo;

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
        
        long srcBase = groupId * (long)inputSize;
        long dstBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        
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

    __kernel void brainCopyOutput(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* inputOutputData, 
        __global float* outputBuffer) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint outputSize = parameters->m_outputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long dstBase = groupId * (long)outputSize;
        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        
        uint workGroupSizeReminder = outputSize % workGroupSize;
        uint modWorkGroupSize = outputSize - workGroupSizeReminder;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputOutputData[srcBase + i + itemId];
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
    __kernel void brainLayerReluActivation(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* inputOutputData, 
        __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long inputOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = (inputValue >= 0.0f) ? inputValue : 0.0f;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            float outputValue = (inputValue >= 0.0f) ? inputValue : 0.0f;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }

    __kernel void brainLayerLeakyReluActivation(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* inputOutputData, 
        __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long inputOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[inputOffset + i + itemId];
            float outputValue = (inputValue >= 0.0f) ? inputValue : ND_GPU_LEAKY_LRU_GRADIENT * inputValue;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            float outputValue = (inputValue >= 0.0f) ? inputValue : ND_GPU_LEAKY_LRU_GRADIENT * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }

    __kernel void brainLayerTanhActivation(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long inputOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
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

    __kernel void brainLayerLinearDropOutActivation(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* notUsed)  
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long inputOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
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

    __kernel void brainLayerLinearActivation(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* inputOutputData, 
        __global float* notUsed,
        __global float* biasPtr,
        __global float* slopesPtr)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long inputOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float  bias = biasPtr[i + itemId];
            float  slope = slopesPtr[i + itemId];
            float inputValue = inputOutputData[inputOffset + i + itemId];

            float outputValue =  bias + slope * inputValue;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float bias = biasPtr[modWorkGroupSize + itemId];
            float slope = slopesPtr[modWorkGroupSize + itemId];
            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];

            float outputValue = bias + slope * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }

    __kernel void brainLayerOffPolicyActivation(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* inputOutputData, 
        __global float* notUsed)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long inputOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint halfSize = inputSize / 2;
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float blend1 = ((i + itemId) >= halfSize) ? 1.0 : 0.0;
            float blend0 = 1.0 - blend1;

            float inputValue = inputOutputData[inputOffset + i + itemId];
            float expenential = exp(inputValue);

            float outputValue = inputValue * blend0 + expenential * blend1;
            inputOutputData[outputOffset + i + itemId] = outputValue;
        }
        if (itemId < workGroupSizeReminder)
        {
            float blend1 = ((modWorkGroupSize + itemId) >= halfSize) ? 1.0 : 0.0;
            float blend0 = 1.0 - blend1;

            float inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            float expenential = exp(inputValue);

            float outputValue = inputValue * blend0 + expenential * blend1;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }

)"""";

const char* ndBrainGpuContext::m_feedForwardKernels_3 =
R""""(
    __kernel void brainLayerSoftmaxActivation(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* notUsed)
    {
        __local float reductionBuffer [1024];
        __local float tmpInputBuffer [ND_GPU_LOCAL_BUFFER_SIZE];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long inputOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;

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
        // need a barrier here so that I can reload maxArg into all workItems.
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
            reductionBuffer[0] = 1.0 / sumArg;
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

const char* ndBrainGpuContext::m_backPropagateKernels_1 =
R""""(
    __kernel void brainCopyInputGradients(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* miniBatchGradients, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long dstBase = groupId * (long)inputSize;
        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        
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
            __global float* miniBatchGradients, 
            __global float* inputOutputGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint outputSize = parameters->m_outputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        long srcBase = groupId * (long)outputSize;        
        long dstBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        
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
            __global float* weightsAndBias, 
            __global float* inputOutputGradients,
            __global float* weightsAndBiasGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inpuData = inputOutputData[srcBase + i + itemId];
            float gradient = (inpuData >= 0.0f) ? 1.0 : 0.0f;
            float outputGrad = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
            float gradient = (inpuData >= 0.0f) ? 1.0 : 0.0f;
            float outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        }
    }

    __kernel void brainLayerBrainLeakyReluBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightsAndBias, 
            __global float* inputOutputGradients,
            __global float* weightsAndBiasGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inpuData = inputOutputData[srcBase + i + itemId];
            float gradient = (inpuData >= 0.0f) ? 1.0 : ND_GPU_LEAKY_LRU_GRADIENT;
            float outputGrad = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
            float gradient = (inpuData >= 0.0f) ? 1.0 : ND_GPU_LEAKY_LRU_GRADIENT;
            float outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        }
    }

    __kernel void brainLayerBrainTanhBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightsAndBias, 
            __global float* inputOutputGradients,
            __global float* weightsAndBiasGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float outputData = inputOutputData[dstBase + i + itemId];
            float a = 1.0 - outputData * outputData;
            float b = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = a * b;
        }
        for (uint itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            float outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
            float a = 1.0 - outputData * outputData;
            float b = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a * b;
        }
    }

    __kernel void brainLayerBrainLinearBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightsAndBias, 
            __global float* inputOutputGradients,
            __global float* weightsAndBiasGradients,
            __global float* slopesPtr) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float slope = slopesPtr[i + itemId];
            float outputGrad = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = slope * outputGrad;
        }
        for (uint itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            float slope = slopesPtr[modWorkGroupSize + itemId];
            float outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = slope * outputGrad;
        }
    }

    __kernel void brainLayerBrainCathegoricalSoftmaxBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightsAndBias, 
            __global float* inputOutputGradients,
            __global float* weightsAndBiasGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inputValue = inputOutputData[dstBase + i + itemId];
            float gradient = inputValue - inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = gradient;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inputValue = inputOutputData[dstBase + modWorkGroupSize + itemId];
            float gradient = inputValue - inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient;
        }
    }

    __kernel void brainLayerBrainDropOutBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightsAndBias, 
            __global float* inputOutputGradients,
            __global float* weightsAndBiasGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;        
        long dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float outputGrad = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = outputGrad;
        }
        if (itemId < workGroupSizeReminder)
        {
            float outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = outputGrad;
        }
    }

    __kernel void brainLayerBrainOffPolicyBackPropagate(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightsAndBias, 
            __global float* inputOutputGradients,
            __global float* weightsAndBiasGradients) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint inputOutputSize = parameters->m_inputOutputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        long srcBase = groupId * (long)inputOutputSize + inputOutputStartOffset;        
        long dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        uint halfSize = inputSize / 2;
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float blend1 = ((i + itemId) >= halfSize) ? 1.0 : 0.0;
            float blend0 = 1.0 - blend1;
            
            float gradient = 1.0;
            float outputData = inputOutputData[dstBase + i + itemId];
            float gradSelect = gradient * blend0 + outputData * blend1;

            float inputGradient = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = gradSelect * inputGradient;
        }
        if (itemId < workGroupSizeReminder)
        {
            float blend1 = ((modWorkGroupSize + itemId) >= halfSize) ? 1.0 : 0.0;
            float blend0 = 1.0 - blend1;
            
            float gradient = 1.0;
            float outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
            float gradSelect = gradient * blend0 + outputData * blend1;

            float inputGradient = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradSelect * inputGradient;
        }
    }

)"""";

const char* ndBrainGpuContext::m_optimizerKernels =
R""""(
    __kernel void brainAdamMomentumUpdate(__global UniformBufferOptimizerArguments* parameters) 
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);

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
        float miniBatchWeight = parameters->m_minibathScale;

        float temp = miniBatchWeight * weightAndBiasGradientBuffer[start + itemId];
        float a = vdw[start + itemId] * parameters->m_alpha + temp * (1.0 - parameters->m_alpha);
        vdw[start + itemId] = a;
            
        // calcuate RMS
        float b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (1.0 - parameters->m_beta);
        vdw2[start + itemId] = b;
        
        float vdwCorrected = a * parameters->m_invAlpha;
        float vdw2Corrected = b * parameters->m_invBeta;
            
        float bias_den = 1.0 / (sqrt(vdw2Corrected) + parameters->m_epsilon);
        float gradient = vdwCorrected * bias_den;
             
        float weight = weightAndBiasBuffer[start + itemId];
        float lassoRegularizer = (weight >= 0.0f) ? regularizer : -regularizer;
        gradient += weight * lassoRegularizer;
        weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
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

        long start = groupId * (long)workGroupSize;
        float miniBatchWeight = parameters->m_minibathScale;

        float temp = miniBatchWeight * weightAndBiasGradientBuffer[start + itemId];
        float a = vdw[start + itemId] * parameters->m_alpha + temp * (1.0 - parameters->m_alpha);
        vdw[start + itemId] = a;
            
        // calcuate RMS
        float b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (1.0 - parameters->m_beta);
        vdw2[start + itemId] = b;
        
        float vdwCorrected = a * parameters->m_invAlpha;
        float vdw2Corrected = b * parameters->m_invBeta;
            
        float bias_den = 1.0 / (sqrt(vdw2Corrected) + parameters->m_epsilon);
        float gradient = vdwCorrected * bias_den;
             
        float weight = weightAndBiasBuffer[start + itemId];
        gradient += weight * regularizer;
        weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
    }

)"""";

const char* ndBrainGpuContext::m_matrixWeightsAndBiasGradients =
R""""(
    __kernel void brainLayerBrainBackPropagateMatrixClearBiasGradients(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* dummy0, 
        __global float* partialBiasSumBuffer, 
        __global float* inputOutputGradientsBuffer,
        __global float* dummy1) 
    {
        const uint itemId = get_local_id(0);
        const uint groupId = get_group_id(0);
        const uint workGroupSize = get_local_size(0);
        
        const uint inputSize = parameters->m_inputSize;
        const uint outputSize = parameters->m_outputSize;
        const uint inputOutputSize = parameters->m_inputOutputSize;
        const uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        const uint alignedOffset = CalculateWorkGroupRoundoff(outputSize, workGroupSize);
        const uint dstOffset = groupId * alignedOffset;

        long inputGradientOffset = groupId * (long)inputOutputSize + inputOutputStartOffset;
        long outputGradientOffset = inputGradientOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        const uint workGroupSizeReminder = outputSize % workGroupSize;
        const uint modWorkGroupSize = outputSize - workGroupSizeReminder;
        
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float outputDerivative = inputOutputGradientsBuffer[outputGradientOffset + i + itemId];
            partialBiasSumBuffer[dstOffset + i + itemId] = outputDerivative;
        }
        if (itemId < workGroupSizeReminder)
        {
            float outputDerivative = inputOutputGradientsBuffer[outputGradientOffset + modWorkGroupSize + itemId];
            partialBiasSumBuffer[dstOffset + modWorkGroupSize + itemId] = outputDerivative;
        }
    }

    __kernel void brainLayerBrainBackPropagateMatrixPartialSumBiasGradients(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* dummy0, 
        __global float* partialBiasSumBuffer, 
        __global float* dummy1,
        __global float* dummy2) 
    {
        const uint itemId = get_local_id(0);
        const uint groupId = get_group_id(0);
        const uint workGroupSize = get_local_size(0);
        
        const uint numberOfIndex = parameters->m_matrixDimensionK;
        const uint srcIndex = groupId + (numberOfIndex + 1) /2;
        if (srcIndex < numberOfIndex)
        {
            const uint outputSize = parameters->m_outputSize;
            const uint alignedOffset = (outputSize + 255) & -256;
        
            const uint dstOffset = groupId * alignedOffset;
            const uint srcOffset = srcIndex * alignedOffset;
        
            const uint workGroupSizeReminder = outputSize % workGroupSize;
            const uint modWorkGroupSize = outputSize - workGroupSizeReminder;
            for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
            {
                float biasGradient = partialBiasSumBuffer[srcOffset + i + itemId];
                partialBiasSumBuffer[dstOffset + i + itemId] += biasGradient;
            }
            if (itemId < workGroupSizeReminder)
            {
                float biasGradient = partialBiasSumBuffer[srcOffset + modWorkGroupSize + itemId];
                partialBiasSumBuffer[dstOffset + modWorkGroupSize + itemId] += biasGradient;
            }
        }
    }

    __kernel void brainLayerBrainBackPropagateMatrixBiasGradients(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* dummy0, 
        __global float* partialBiasSumBuffer, 
        __global float* dummy1,
        __global float* weightAndBiasGradients) 
    {
        const uint itemId = get_local_id(0);
        const uint groupId = get_group_id(0);
        const uint workGroupSize = get_local_size(0);
        
        const uint inputSize = parameters->m_inputSize;
        const uint outputSize = parameters->m_outputSize;
        const uint height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const uint width = (inputSize + ND_GPU_TILED_MATRIX_ROWS * 2 - 1) & -ND_GPU_TILED_MATRIX_ROWS * 2;
        const uint matrixSize = CalculateWorkGroupRoundoff(width * height, ND_DEFAULT_WORKGROUP_SIZE);

        const long parametersStartOffset = (long)parameters->m_parametersStartOffset + matrixSize;
        
        const uint workGroupSizeReminder = outputSize % workGroupSize;
        const uint modWorkGroupSize = outputSize - workGroupSizeReminder;
        for (uint rowBlock = 0; rowBlock < modWorkGroupSize; rowBlock += workGroupSize)
        {
            float biasDerivative = partialBiasSumBuffer[rowBlock + itemId];
            weightAndBiasGradients[parametersStartOffset + rowBlock + itemId] = biasDerivative;
        }
        if (itemId < workGroupSizeReminder)
        {
            float biasDerivative = partialBiasSumBuffer[modWorkGroupSize + itemId];
            weightAndBiasGradients[parametersStartOffset + modWorkGroupSize + itemId] = biasDerivative;
        }
    }

    __kernel void brainLayerBrainBackPropagateMatrixWeightsGradients(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* inputOutputData, 
        __global float* weightAndBias, 
        __global float* inputOutputGradients,
        __global float* weightAndBiasGradients) 
    {
        __local float cachedRowGradient[1024];
        __local float cachedOutputGradients[ND_GPU_LOCAL_BUFFER_SIZE];
        
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint numberOfRows = parameters->m_matrixDimensionK;
        uint inputOutputSize = parameters->m_inputOutputSize;
        
        long inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        long dstBase = inputOutputStartOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        
        if (itemId == 0)
        {
            for (uint row = 0; row < numberOfRows; ++row)
            {
                float outputDerivative = inputOutputGradients[dstBase + row * inputOutputSize + groupId];
                cachedOutputGradients[row] = outputDerivative;
            }
        }
        
        const uint workGroupSizeReminder = inputSize % workGroupSize;
        const uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        long parametersStartOffset = (long)parameters->m_parametersStartOffset;
        uint width = (inputSize + ND_GPU_TILED_MATRIX_ROWS * 2 - 1) & -ND_GPU_TILED_MATRIX_ROWS * 2;
        
        barrier(CLK_LOCAL_MEM_FENCE); 
        for (uint rowBlock = 0; rowBlock < modWorkGroupSize; rowBlock += workGroupSize)
        {
            for (uint itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedRowGradient[itemId] = 0.0f;
            }
            long inputOffsetBase = inputOutputStartOffset + rowBlock;
            for (uint row = 0; row < numberOfRows; ++row)
            {
                long inputOffset = inputOffsetBase + row * inputOutputSize;
                float outputDerivative = cachedOutputGradients[row];
        
                float inputValue = inputOutputData[inputOffset + itemId];
                cachedRowGradient[itemId] += outputDerivative * inputValue;
            }
            // store this weight gradient sum
            long parametersOffset = parametersStartOffset + rowBlock + width * groupId;
            weightAndBiasGradients[parametersOffset + itemId] = cachedRowGradient[itemId];
        }
        
        if (itemId < workGroupSizeReminder)
        {
            cachedRowGradient[itemId] = 0.0f;
        
            long inputOffsetBase = inputOutputStartOffset + modWorkGroupSize;
            for (uint row = 0; row < numberOfRows; ++row)
            {
                long inputOffset = inputOffsetBase + row * inputOutputSize;
                float outputDerivative = cachedOutputGradients[row];
        
                float inputValue = inputOutputData[inputOffset + itemId];
                cachedRowGradient[itemId] += outputDerivative * inputValue;
            }
            // store this weight gradient sum
            long parametersOffset = parametersStartOffset + modWorkGroupSize + width * groupId;
            float weightGradeint = cachedRowGradient[itemId];
            weightAndBiasGradients[parametersOffset + itemId] = weightGradeint;
        }
    }
)"""";

const char* ndBrainGpuContext::m_matrixMultiply =
R""""(
    __kernel void brainLayerMatrixMatrixMultiply(
            __global const UniformBufferLayerArguments* parameters, 
            __global float* inputOutputData, 
            __global float* weightsAndBias) 
    {
        const uint tileSize = ND_GPU_TILED_MATRIX_ROWS;
        const uint tileSizeBits = ND_GPU_TILED_MATRIX_ROWS_BITS;

        //__local float tile_acc[tileSize][tileSize + 1];
        __local float tile_inputs[tileSize * 2][tileSize + 1];
        __local float tile_weights[tileSize * 2][tileSize + 1];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        const uint inputSize = parameters->m_inputSize;
        const uint outputSize = parameters->m_outputSize;
        const uint height = (outputSize + tileSize - 1) & -tileSize;
        const uint width = (inputSize + tileSize * 2 - 1) & -tileSize * 2;

        //const uint minibatchBlock = parameters->m_matrixDimensionK >> tileSizeBits;
        const uint minibatchBlock = (outputSize + tileSize - 1) >> tileSizeBits;
        const uint groupId_y = groupId / minibatchBlock;
        const uint groupId_x = groupId - groupId_y * minibatchBlock;

        //Initialise the accumulation register
        const long blockBase = groupId_x * tileSize;
        const long parametersStartOffset = blockBase * width + parameters->m_parametersStartOffset;
        const long parametersBiasOffset = blockBase + width * height + parameters->m_parametersStartOffset;

        uint acc_x = itemId & (tileSize-1);
        uint acc_y = itemId >> tileSizeBits;
        if (acc_x < tileSize)
        {
            float a = weightsAndBias[parametersBiasOffset + acc_x];
            tile_inputs[0][acc_x] = a;
        }

        const uint inputOutputStride = parameters->m_inputOutputSize;
        const long inputOffset = groupId_y * (long)inputOutputStride * tileSize + parameters->m_inputOutputStartOffset;

        // Loop over all tiles
        uint halfTileStart = tileSize / 2;
        uint itemId_x = itemId & (tileSize * 2 - 1);
        uint itemId_y = itemId >> (tileSizeBits + 1);
        
        float acc = 0.0;
        barrier(CLK_LOCAL_MEM_FENCE); 

        float matrixBias = tile_inputs[0][acc_y];
        for (uint tile = 0; tile < width; tile += tileSize * 2)
        {
            // read the transpose of the tiles (GPU style, but too slow for CPU)
            long inputStartOffset = tile + inputOffset;
            long weightOffsetStart = tile + parametersStartOffset;

            float weight0 = weightsAndBias[weightOffsetStart + itemId_y * width + itemId_x];
            float inputData0 = inputOutputData[inputStartOffset + itemId_y * inputOutputStride + itemId_x];
            tile_weights[itemId_x][itemId_y] = weight0;
            tile_inputs[itemId_x][itemId_y] = inputData0;

            float weight1 = weightsAndBias[weightOffsetStart + (itemId_y + halfTileStart) * width + itemId_x];
            float inputData1 = inputOutputData[inputStartOffset + (itemId_y + halfTileStart) * inputOutputStride + itemId_x];
            tile_weights[itemId_x][itemId_y + halfTileStart] = weight1;
            tile_inputs[itemId_x][itemId_y + halfTileStart] = inputData1;
            barrier(CLK_LOCAL_MEM_FENCE); 

            // Perform the computation for a single tile
            for (uint i = 0; i < tileSize * 2; ++i)
            {
                float a = tile_weights[i][acc_y];
                acc += a * tile_inputs[i][acc_x];
            }
            barrier(CLK_LOCAL_MEM_FENCE); 
        }
        // transpose the flat array results
        tile_inputs[acc_x][acc_y] = acc + matrixBias;

        const uint numberOutput = ((groupId_x + 1) * ND_GPU_TILED_MATRIX_ROWS < outputSize) ? ND_GPU_TILED_MATRIX_ROWS : outputSize - groupId_x * ND_GPU_TILED_MATRIX_ROWS;
        long outputOffset = groupId_x * ND_GPU_TILED_MATRIX_ROWS + (long)inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        outputOffset += acc_y * inputOutputStride + acc_x;
        barrier(CLK_LOCAL_MEM_FENCE); 
        
        //float value = tile_acc[acc_y][acc_x];
        float value = tile_inputs[acc_y][acc_x];
        inputOutputData[outputOffset] = value;
    }

    __kernel void brainLayerBrainBackPropagateMatrixInputGradients(
        __global const UniformBufferLayerArguments* parameters, 
        __global float* inputOutputData, 
        __global float* weightAndBias, 
        __global float* inputOutputGradients,
        __global float* weightAndBiasGradients) 
    {
        const uint tileSize = ND_GPU_TILED_MATRIX_ROWS;
        const uint tileSizeBits = ND_GPU_TILED_MATRIX_ROWS_BITS;

        __local float tile_weights[tileSize][tileSize+1];
        __local float tile_outputGradients[tileSize][tileSize+1];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        uint inputSize = parameters->m_inputSize;
        uint outputSize = parameters->m_outputSize;
        uint inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        const uint minibatchBlock = parameters->m_matrixDimensionK >> tileSizeBits;
        const uint groupId_y = groupId / minibatchBlock;
        const uint groupId_x = groupId - groupId_y * minibatchBlock;

        const uint inputOutputStride = parameters->m_inputOutputSize;
        const uint width = (inputSize + (tileSize * 2) - 1) & -(tileSize * 2);
        const long inputOffset = groupId_x * (long)tileSize * inputOutputStride + inputOutputStartOffset;
        const long outputOffset = inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        const long parametersStartOffset = groupId_y * (long)tileSize + parameters->m_parametersStartOffset;

        const uint itemId_x = itemId & (tileSize - 1);
        const uint itemId_y = itemId >> tileSizeBits;

        // Loop over all tiles
        float acc = 0.0f;
        const uint dimensionK = ((outputSize + tileSize - 1) & -tileSize);
        for (uint tile = 0; tile < dimensionK; tile += tileSize)
        {
            // Load one transposed tile A and B into local memory (cpu style)
            long outputStartOffset = tile + outputOffset;
            long weightOffsetStart = tile * width + parametersStartOffset;
            float weight = weightAndBias[weightOffsetStart + itemId_y * width + itemId_x];
            float outputGradient = inputOutputGradients[outputStartOffset + itemId_y * inputOutputStride + itemId_x];
            tile_weights[itemId_y][itemId_x] = weight;
            tile_outputGradients[itemId_x][itemId_y] = outputGradient;
            barrier(CLK_LOCAL_MEM_FENCE); 
            
            // Perform the computation for a single tile
            for (uint i = 0; i < tileSize; ++i)
            {
                float a = tile_outputGradients[i][itemId_y];
                acc += a * tile_weights[i][itemId_x];
            }
            barrier(CLK_LOCAL_MEM_FENCE); 
        }

        // the result is not transposed
        long dstOffset = inputOffset + groupId_y * tileSize + itemId_y * inputOutputStride; 
        inputOutputGradients[dstOffset + itemId_x] = acc;
    }

)"""";

const char* ndBrainGpuContext::m_otherShaderFunctions =
R""""(
    __kernel void brainCopyStridedBuffer(
        uint strideInByte,
        uint srcStrideInByte,
        uint srcOffsetInByte,
        uint dstStrideInByte,
        uint dstOffsetInByte,
        __global float* outputData,
        __global float* inputData)
    {                                                                      
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        long dstOffset = (groupId * (long)dstStrideInByte + dstOffsetInByte) / sizeof (uint);
        long srcOffset = (groupId * (long)srcStrideInByte + srcOffsetInByte) / sizeof (uint);
        
        uint dstStride = strideInByte / sizeof (uint);       
        uint workGroupSizeReminder = dstStride % workGroupSize;
        uint modWorkGroupSize = dstStride - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputData[srcOffset + i + itemId];
            outputData[dstOffset + i + itemId] = a;
        }
        if (itemId < workGroupSizeReminder)
        {
            float a = inputData[srcOffset + modWorkGroupSize + itemId];
            outputData[dstOffset + modWorkGroupSize + itemId] = a;
        }
    }

    __kernel void brainCopyStridedBufferIndirect(
        uint strideInByte,
        uint srcStrideInByte,
        uint srcOffsetInByte,
        uint dstStrideInByte,
        uint dstOffsetInByte,
        __global float* outputData,
        __global float* inputData, 
        __global uint* indexBuffer) 
    {                                                                      
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        long dstOffset = (groupId * (long) dstStrideInByte + dstOffsetInByte) / sizeof (uint);
        long srcOffset = (indexBuffer[groupId] * (long) srcStrideInByte + srcOffsetInByte) / sizeof (uint);
        uint dstStride = strideInByte / sizeof (uint);
        
        uint workGroupSizeReminder = dstStride % workGroupSize;
        uint modWorkGroupSize = dstStride - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float a = inputData[srcOffset + i + itemId];
            outputData[dstOffset + i + itemId] = a;
        }
        if (itemId < workGroupSizeReminder)
        {
            float a = inputData[srcOffset + modWorkGroupSize + itemId];
            outputData[dstOffset + modWorkGroupSize + itemId] = a;
        }
    }

)"""";

const char* ndBrainGpuContext::m_mathOpsCommand =
R""""(
    __kernel void brainBufferAssigment(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = inputData[global_id];
        }
    }

    __kernel void brainAdd(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = outputData[global_id] + inputData[global_id];
        }
    }

    __kernel void brainSub(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = outputData[global_id] - inputData[global_id];
        }
    }

    __kernel void brainMul(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = outputData[global_id] * inputData[global_id];
        }
    }

    __kernel void brainMin(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            float a = outputData[global_id];
            float b = inputData[global_id];
            outputData[global_id] = (a <= b) ? a : b;
        }
    }

    __kernel void brainMax(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            float a = outputData[global_id];
            float b = inputData[global_id];
            outputData[global_id] = (a >= b) ? a : b;
        }
    }

    __kernel void brainLessEqual(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = (outputData[global_id] <= inputData[global_id]) ? 1.0 : 0.0;
        }
    }

    __kernel void brainGreaterEqual(
        int numberOfElements,
        __global float* outputData,
        __global float* inputData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = (outputData[global_id] >= inputData[global_id]) ? 1.0 : 0.0;
        }
    }

    __kernel void brainLessScalar(
        int numberOfElements,
        __global float* outputData,
        float test)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = (outputData[global_id] < test) ? 1.0 : 0.0;
        }
    }

    __kernel void brainGreaterScalar(
        int numberOfElements,
        __global float* outputData,
        float test)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = (outputData[global_id] > test) ? 1.0 : 0.0;
        }
    }

    __kernel void brainLessEqualScalar(
        int numberOfElements,
        __global float* outputData,
        float test)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = (outputData[global_id] <= test) ? 1.0 : 0.0;
        }
    }

    __kernel void brainGreaterEqualScalar(
        int numberOfElements,
        __global float* outputData,
        float test)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = (outputData[global_id] >= test) ? 1.0 : 0.0;
        }
    }

    __kernel void brainScale(
        int numberOfElements,
        __global float* outputData, 
        float scale)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = outputData[global_id] * scale;
        }
    }

    __kernel void brainSelect(
        int numberOfElements,
        __global float* outputData, 
        __global float* mask, 
        float a, float b)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            float t1 = mask[global_id];
            float t0 = 1.0 - t1;
            outputData[global_id] = a * t0 + b * t1;
        }
    }

    __kernel void brainSet(
        int numberOfElements,
        __global float* outputData, 
        float value)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = value;
        }
    }

    __kernel void brainScaleAdd(
        int numberOfElements,
        __global float* outputData, 
        __global float* inputData, 
        float scale)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            outputData[global_id] = outputData[global_id] + inputData[global_id] * scale;
        }
    }

    __kernel void brainBlendScale(
        int numberOfElements,
        __global float* outputData, 
        __global float* inputData,
        float blend)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            float value = outputData[global_id] * (1.0 - blend) + inputData[global_id] * blend;
            outputData[global_id] = value;
        }
    }

    __kernel void brainBlendVector(
        int numberOfElements,
        __global float* outputData, 
        __global float* inputData,
        __global float* blendData)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            float t1 = blendData[global_id];
            float t0 = 1.0 - t1;
            float value = outputData[global_id] * t0 + inputData[global_id] * t1;
            outputData[global_id] = value;
        }
    }

    __kernel void brainBroadcastScalar(
        int numberOfElements,
        __global float* outputData, 
        __global float* inputData)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint dstOffset = groupId * numberOfElements;
        uint workGroupSizeReminder = numberOfElements % workGroupSize;
        uint modWorkGroupSize = numberOfElements - workGroupSizeReminder;

        float value = inputData[groupId];    
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            outputData[dstOffset + i + itemId] = value;
        }
        if (itemId < workGroupSizeReminder)
        {
           outputData[dstOffset + modWorkGroupSize + itemId] = value;
        }
    }

    __kernel void brainMinScalar(
        int numberOfElements,
        __global float* outputData,
        float scalar)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            float a = outputData[global_id];
            outputData[global_id] = (a <= scalar) ? a : scalar;
        }
    }

    __kernel void brainMaxScalar(
        int numberOfElements,
        __global float* outputData,
        float scalar)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            float a = outputData[global_id];
            outputData[global_id] = (a >= scalar) ? a : scalar;
        }
    }

)"""";

const char* ndBrainGpuContext::m_probabilitiesKernels =
R""""(
    __kernel void brainNormalDistribution(int numberOfElements, __global float* uniformRandom)
    {
        int global_id = get_global_id(0);
        if (global_id < numberOfElements)
        {
            const float c0 = 2.515517;
		    const float c1 = 0.802853;
		    const float c2 = 0.010328;
		    const float d0 = 1.432788;
		    const float d1 = 0.189269;
		    const float d2 = 0.001308;

            float clipRand = uniformRandom[global_id];
            if (clipRand < 1.0e-6)
            {
                clipRand = 1.0e-6;
            }
            else if (clipRand > (1.0 - 1.0e-6))
            {
                clipRand = 1.0 - 1.0e-6;
            }
            
            float r = (clipRand >= 0.5) ? (1.0 - clipRand) : clipRand;
            float t = sqrt (-2.0f * log(r));

		    float numerator = c0 + (c2 * t + c1) * t;
		    float denominator = 1.0 + ((d2 * t + d1) * t + d0) * t;
		    float normal = t - numerator / denominator;
            if (clipRand < 0.5)
            {
                normal = -normal;
            }
            uniformRandom[global_id] = normal;
        }
    }

    __kernel void brainEntropyReqularization(
        int numberOfElements,
        __global float* outputBuffer,
        __global float* meanBuffer,
        __global float* varianceBuffer,
        float regularizationTemperature)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
        
        __local float reductionBuffer [1024];
        
        uint srcOffset = groupId * numberOfElements;
        
		float sample = meanBuffer[srcOffset + itemId];
		float sigma = varianceBuffer[srcOffset + itemId];
        float entropy = 0.5 * sample * sample / (sigma * sigma) + log(sigma);
        reductionBuffer[itemId] = entropy;
        barrier(CLK_LOCAL_MEM_FENCE); 
        
        if (itemId == 0)
        {
            float ent = 0.5 * 1.837877 * (float)numberOfElements;
            for (int i = 0; i < numberOfElements; ++i)
            {
                ent += reductionBuffer[i];
            }
            outputBuffer[groupId] = -ent * regularizationTemperature;
        }
    }

    __kernel void brainEntropyReqularizationGradient(
        int numberOfElements,
        __global float* outputBuffer,
        __global float* meanBuffer,
        __global float* varianceBuffer,
        float regularizationTemperature)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint srcOffset = groupId * numberOfElements;
        uint dstOffset = srcOffset * 2;
        
		float sample = meanBuffer[srcOffset + itemId];
		float sigma = varianceBuffer[srcOffset + itemId];

        float invSigma = 1.0 / sigma;
        float invSigma2 = invSigma * invSigma;
        
        float meanGradient = regularizationTemperature * sample * invSigma2;
        float sigmaGradient = regularizationTemperature * invSigma * (sample * sample * invSigma2 - 1.0);

        outputBuffer[dstOffset + itemId] = meanGradient;
        outputBuffer[dstOffset + numberOfElements + itemId] = sigmaGradient;
    }
   
)"""";


ndSharedPtr<ndBrainKernel> ndBrainGpuContext::CreateKerner(const cl::Program& program, const char* const functionMame) const
{
    cl_int errcode_ret = 0;
    ndSharedPtr<cl::Kernel> shader(new cl::Kernel(program, functionMame, &errcode_ret));
    ndAssert(errcode_ret == 0);

    ndSharedPtr<ndBrainKernel> kernel(new OpenclKernel((ndBrainGpuContext*)this, shader));
    return kernel;
}

void ndBrainGpuContext::CreateKerners()
{
    std::string source(m_commonKernelsInclude);
    source += m_mathOpsCommand;
    source += m_matrixMultiply;
    source += m_optimizerKernels;
    source += m_otherShaderFunctions;
    source += m_feedForwardKernels_1;
    source += m_feedForwardKernels_2;
    source += m_feedForwardKernels_3;
    source += m_probabilitiesKernels;
    source += m_backPropagateKernels_1;
    source += m_backPropagateKernels_2;
    source += m_matrixWeightsAndBiasGradients;

    class ClProgram : public cl::Program
    {
        public:
        #if 1
        ClProgram(const cl::Context& context, const std::string& source)
            :cl::Program(context, source, CL_TRUE, &m_errcode)
        {
        }
        #else
        ClProgram(const cl::Context& context, const std::string& source)
            :cl::Program()
        {
            const char* strings = source.c_str();
            const size_t length = source.size();
            
            object_ = ::clCreateProgramWithSource(context(), (cl_uint)1, &strings, &length, &m_errcode);
            if (m_errcode == CL_SUCCESS)
            {
                //const char* const compilerOptions = "-cl-std=CL3.0 -cl-opt-disable";
                //const char* const compilerOptions = "-cl-std=CL2.0 -cl-opt-disable";
                //const char* const compilerOptions = "-cl-std=CL2.0";
                const char* const compilerOptions = "-cl-std=CL1.2";
                m_errcode = ::clBuildProgram(object_, 0, nullptr, compilerOptions, nullptr, nullptr);
            }
        }
        #endif
        cl_int m_errcode;
    };

    ClProgram program(**m_context, source);
    ndAssert(program.m_errcode == 0);

    #if 0
    // this only seems to work with nvidia PTX        
    size_t bin_sz;
    cl_int errcode = 0;
    errcode = clGetProgramInfo(program.get(), CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &bin_sz, NULL);
    ndAssert(errcode == 0);
    
    unsigned char* code = (unsigned char*)malloc(bin_sz);
    errcode = clGetProgramInfo(program.get(), CL_PROGRAM_BINARIES, sizeof(unsigned char*), &code, NULL);
    ndAssert(errcode == 0);
    free(code);
    #endif

    // create all feed foward shaders
    m_brainCopyInput = CreateKerner(program, "brainCopyInput");
    m_brainCopyOutput = CreateKerner(program, "brainCopyOutput");
    m_brainLayerReluActivation = CreateKerner(program, "brainLayerReluActivation");
    m_brainLayerTanhActivation = CreateKerner(program, "brainLayerTanhActivation");
    m_brainLayerLinearActivation = CreateKerner(program, "brainLayerLinearActivation");
    m_brainLayerSoftmaxActivation = CreateKerner(program, "brainLayerSoftmaxActivation");
    m_brainLayerLeakyReluActivation = CreateKerner(program, "brainLayerLeakyReluActivation");
    m_brainLayerOffPolicyActivation = CreateKerner(program, "brainLayerOffPolicyActivation");
    m_brainLayerDropOutActivation = CreateKerner(program, "brainLayerLinearDropOutActivation");
    m_brainLayerMatrixMatrixMultiply = CreateKerner(program, "brainLayerMatrixMatrixMultiply");

    // create all backpropagate shaders
    m_brainCopyInputGradients = CreateKerner(program, "brainCopyInputGradients");
    m_brainCopyOutputGradients = CreateKerner(program, "brainCopyOutputGradients");
    m_brainLayerLinearPropagate = CreateKerner(program, "brainLayerBrainLinearBackPropagate");
    m_brainLayerReluBackPropagate = CreateKerner(program, "brainLayerBrainReluBackPropagate");
    m_brainLayerTanhBackPropagate = CreateKerner(program, "brainLayerBrainTanhBackPropagate");
    m_brainLayerDropOutBackPropagate = CreateKerner(program, "brainLayerBrainDropOutBackPropagate");
    m_brainLayerLeakyReluBackPropagate = CreateKerner(program, "brainLayerBrainLeakyReluBackPropagate");
    m_brainLayerOffPolicyBackPropagate = CreateKerner(program, "brainLayerBrainOffPolicyBackPropagate");
    m_brainLayerCathegoricalSoftmaxBackPropagate = CreateKerner(program, "brainLayerBrainCathegoricalSoftmaxBackPropagate");
    m_brainLayerMatrixBackPropagateBiasGradients = CreateKerner(program, "brainLayerBrainBackPropagateMatrixBiasGradients");
    m_brainLayerMatrixBackPropagateInputGradients = CreateKerner(program, "brainLayerBrainBackPropagateMatrixInputGradients");
    m_brainLayerMatrixBackPropagateWeightGradients = CreateKerner(program, "brainLayerBrainBackPropagateMatrixWeightsGradients");
    m_brainLayerMatrixBackPropagateClearBiasGradients = CreateKerner(program, "brainLayerBrainBackPropagateMatrixClearBiasGradients");
    m_brainLayerMatrixBackPropagateAddBiasGradients = CreateKerner(program, "brainLayerBrainBackPropagateMatrixPartialSumBiasGradients");

    // accumulate gradient kernels and optimizer kernels
    m_brainAdamMomentumUpdate = CreateKerner(program, "brainAdamMomentumUpdate");
    m_brainAdamRidgeOptimizerUpdate = CreateKerner(program, "brainAdamUpdateRidgeRegularizer");
    m_brainAdamLassoOptimizerUpdate = CreateKerner(program, "brainAdamUpdateLassoRegularizer");

    // other shaders
    m_brainCopyStridedBuffer = CreateKerner(program, "brainCopyStridedBuffer");
    m_brainCopyStridedBufferIndirect = CreateKerner(program, "brainCopyStridedBufferIndirect");

    // math operations
    m_brainSet = CreateKerner(program, "brainSet");
    m_brainAdd = CreateKerner(program, "brainAdd");
    m_brainSub = CreateKerner(program, "brainSub");
    m_brainMul = CreateKerner(program, "brainMul");
    m_brainMin = CreateKerner(program, "brainMin");
    m_brainMax = CreateKerner(program, "brainMax");
    m_brainScale = CreateKerner(program, "brainScale");
    m_brainSelect = CreateKerner(program, "brainSelect");
    m_brainScaleAdd = CreateKerner(program, "brainScaleAdd");
    m_brainMinScalar = CreateKerner(program, "brainMinScalar");
    m_brainMaxScalar = CreateKerner(program, "brainMaxScalar");
    m_brainLessEqual = CreateKerner(program, "brainLessEqual");
    m_brainBlendScale = CreateKerner(program, "brainBlendScale");
    m_brainLessScalar = CreateKerner(program, "brainLessScalar");
    m_brainBlendVector = CreateKerner(program, "brainBlendVector");
    m_brainGreaterEqual = CreateKerner(program, "brainGreaterEqual");
    m_brainAssigment = CreateKerner(program, "brainBufferAssigment");
    m_brainGreaterScalar = CreateKerner(program, "brainGreaterScalar");
    m_brainBroadcastScalar = CreateKerner(program, "brainBroadcastScalar");
    m_brainLessEqualScalar = CreateKerner(program, "brainLessEqualScalar");
    m_brainGreaterEqualScalar = CreateKerner(program, "brainGreaterEqualScalar");
    m_brainNormalDistribution = CreateKerner(program, "brainNormalDistribution");
    m_brainEntropyRegularization = CreateKerner(program, "brainEntropyReqularization");
    m_brainEntropyRegularizationGradient = CreateKerner(program, "brainEntropyReqularizationGradient");
}