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

    #define ND_GPU_LOCAL_BUFFER_SIZE	        512
    #define ND_GPU_TILED_MATRIX_ROWS_BITS       4
    #define ND_GPU_TILED_MATRIX_COLUMNS_BITS    5
    #define ND_GPU_TILED_MATRIX_ROWS            (1<<ND_GPU_TILED_MATRIX_ROWS_BITS)
    #define ND_GPU_TILED_MATRIX_COLUMNS         (1<<ND_GPU_TILED_MATRIX_COLUMNS_BITS)

    typedef struct
    {
        uint m_inputSize;
        uint m_outputSize;
        uint m_inputOutputSize;
        uint m_inputOutputStartOffset;
        uint m_parametersBatchSize;
        uint m_parametersStartOffset;
        uint m_tiledStride;
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
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float inpuData = inputOutputData[srcBase + i + itemId];
            float gradient = (inpuData >= 0.0f) ? 1.0f : 0.0f;
            float outputGrad = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
        }
        if (itemId < workGroupSizeReminder)
        {
            float inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
            float gradient = (inpuData >= 0.0f) ? 1.0f : 0.0f;
            float outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        }
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

        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            float outputData = inputOutputData[dstBase + i + itemId];
            float a = 1.0f - outputData * outputData;
            float b = inputOutputGradients[dstBase + i + itemId];
            inputOutputGradients[srcBase + i + itemId] = a * b;
        }
        for (uint itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            float outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
            float a = 1.0f - outputData * outputData;
            float b = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a * b;
        }
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

)"""";

const char* ndBrainGpuContext::m_optimizerKernels =
R""""(

    //void Execute(uint groupId, uint workGroupSize)
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

    __kernel void brainAccumulateGradientsAndAverage(
            __global const UniformBufferLayerArguments* parameters,
            __global float* gradientBuffer)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint miniBatchSize = parameters->m_inputOutputSize;
        
        float sum = 0.0f;
        uint start = groupId * workGroupSize;
        for (uint j = 0; j < miniBatchSize; ++j)
        {
            uint base = start + j * inputSize;
            sum += gradientBuffer[base + itemId];
        }
        float weightFactor = 1.0f / (float)miniBatchSize;
        barrier(CLK_LOCAL_MEM_FENCE); 

        gradientBuffer[start + itemId] = sum * weightFactor;
    }

    __kernel void brainAccumulateGradients(
            __global const UniformBufferLayerArguments* parameters,
            __global float* gradientBuffer)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint inputSize = parameters->m_inputSize;
        uint miniBatchSize = parameters->m_inputOutputSize;
        
        uint dst = parameters->m_parametersStartOffset + groupId * workGroupSize;
        uint src = dst + parameters->m_parametersBatchSize;
        float a = gradientBuffer[src + itemId];
        gradientBuffer[dst + itemId] += a;
    }

    __kernel void brainAverageGradients(
            __global const UniformBufferLayerArguments* parameters,
            __global float* gradientBuffer)
    {
        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);

        uint start = groupId * workGroupSize;
        uint miniBatchSize = parameters->m_inputOutputSize;
        
        float weightFactor = 1.0f / (float)miniBatchSize;
        float sum = gradientBuffer[start + itemId];
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

)"""";

const char* ndBrainGpuContext::m_matrixMultiply =
R""""(
    __kernel void brainLayerMatrixMatrix(__global const UniformBufferLayerArguments* parameters, __global float* inputOutputData, __global float* weightsAndBias) 
    {
        __local float tile_acc[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS+1];
        __local float tile_inputs[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS+1];
        __local float tile_weights[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS+1];

        uint itemId = get_local_id(0);
        uint groupId = get_group_id(0);
        uint workGroupSize = get_local_size(0);
    
        const uint inputSize = parameters->m_inputSize;
        const uint outputSize = parameters->m_outputSize;
        const uint height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const uint width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;

        const uint groupId_x = groupId % parameters->m_tiledStride;
        const uint groupId_y = (groupId - groupId_x) / parameters->m_tiledStride;

        //Initialise the accumulation register
        const uint parametersBiasOffset = width * height + parameters->m_parametersStartOffset;
        const uint parametersStartOffset = groupId_x * width * ND_GPU_TILED_MATRIX_ROWS + parameters->m_parametersStartOffset;
        
        uint acc_x = itemId & (ND_GPU_TILED_MATRIX_ROWS-1);
        uint acc_y = itemId >> ND_GPU_TILED_MATRIX_ROWS_BITS;

        //__local float biasValue[ND_GPU_TILED_MATRIX_ROWS];
        //if (acc_x < ND_GPU_TILED_MATRIX_ROWS)
        //{
        //    biasValue[acc_x] = weightsAndBias[parametersBiasOffset + acc_x];
        //}
        //barrier(CLK_LOCAL_MEM_FENCE); 
        //tile_acc[acc_y][acc_x] = biasValue[acc_x];
        //barrier(CLK_LOCAL_MEM_FENCE); 
        //float acc = tile_acc[acc_y][acc_x];

        if (acc_x < ND_GPU_TILED_MATRIX_ROWS)
        {
            tile_acc[acc_x][0] = weightsAndBias[parametersBiasOffset + acc_x];
        }
        barrier(CLK_LOCAL_MEM_FENCE); 
        float acc = tile_acc[acc_y][0];

        const uint inputOutputStride = parameters->m_inputOutputSize;
        const uint inputOffset = groupId_y * inputOutputStride * ND_GPU_TILED_MATRIX_ROWS + parameters->m_inputOutputStartOffset;

        // Loop over all tiles
        const uint tile_x = itemId & (ND_GPU_TILED_MATRIX_COLUMNS-1);
        const uint tile_y0 = itemId >> ND_GPU_TILED_MATRIX_COLUMNS_BITS;
        const uint tile_y1 = tile_y0 + ND_GPU_TILED_MATRIX_ROWS/2;
        for (uint tile = 0; tile < width; tile += ND_GPU_TILED_MATRIX_COLUMNS)
        {
            // Load one tile of weights and one tile of inputs into local memory
            uint inputStartOffset = tile + inputOffset;
            uint weightOffsetStart = tile + parametersStartOffset;
            tile_weights[tile_y0][tile_x] = weightsAndBias[weightOffsetStart + width * tile_y0 + tile_x];
            tile_weights[tile_y1][tile_x] = weightsAndBias[weightOffsetStart + width * tile_y1 + tile_x];
            tile_inputs[tile_y0][tile_x] = inputOutputData[inputStartOffset + inputOutputStride * tile_y0 + tile_x];
            tile_inputs[tile_y1][tile_x] = inputOutputData[inputStartOffset + inputOutputStride * tile_y1 + tile_x];
            barrier(CLK_LOCAL_MEM_FENCE); 
           
            // Perform the computation for a single tile
            for (uint i = 0; i < ND_GPU_TILED_MATRIX_COLUMNS; ++i)
            {
                float a = tile_weights[acc_y][i];
                //tile_acc[acc_x][acc_y] += a * tile_inputs[acc_x][i];
                acc += a * tile_inputs[acc_x][i];
            }
            // it seems a barrier should not be nesserary here, but the kernel fail without one.
            barrier(CLK_LOCAL_MEM_FENCE); 
        }

        // trnaspose the flat array of results
        tile_acc[acc_x][acc_y] = acc;
        barrier(CLK_LOCAL_MEM_FENCE); 
        
        const uint numberOutput = ((groupId_x + 1) * ND_GPU_TILED_MATRIX_ROWS < outputSize) ? ND_GPU_TILED_MATRIX_ROWS : outputSize - groupId_x * ND_GPU_TILED_MATRIX_ROWS;
        uint outputOffset = groupId_x * ND_GPU_TILED_MATRIX_ROWS + inputOffset + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        float value = tile_acc[acc_y][acc_x];
        inputOutputData[outputOffset + acc_y * inputOutputStride + acc_x] = value;
    }

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

        for (uint i = 0; i < inputSize; i += workGroupSize)
        {
            cachedInput[i + itemId] = 0.0f;
        }
        
        uint srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        uint dstBase = srcBase + CalculateWorkGroupRoundoff(inputSize, workGroupSize);
        uint parametersStartOffset = parameters->m_parametersStartOffset;
        
        uint workGroupSizeReminder = inputSize % workGroupSize;
        uint modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        // calculate input gradients
        uint width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
        for (uint j = 0; j < outputSize; ++j)
        {
            float scale = inputOutputGradients[dstBase + j];
            uint weightOffset = j * width + parametersStartOffset;
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
        for (uint i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            inputOutputGradients[srcBase + i + itemId] = cachedInput[i + itemId];
            cachedInput[i + itemId] = inputOutputData[srcBase + i + itemId];
        }
        if (itemId < workGroupSizeReminder)
        {
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = cachedInput[modWorkGroupSize + itemId];
            cachedInput[modWorkGroupSize + itemId] = inputOutputData[srcBase + modWorkGroupSize + itemId];
        }
        
        // calculate weights and bias gradients
        uint height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        uint matrixSize = width * height;
        uint weightAndBiasGradientOffset = groupId * parameters->m_parametersBatchSize + parametersStartOffset;

        // calculate bias Gradient
        uint workGroupOuputSizeReminder = outputSize % workGroupSize;
        uint modWorkGroupOuputSize = outputSize - workGroupOuputSizeReminder;
        for (uint i = 0; i < modWorkGroupOuputSize; i += workGroupSize)
         {
            float a = inputOutputGradients[dstBase + i + itemId];
            weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + i + itemId] = a;
        }
        if(itemId < workGroupOuputSizeReminder)
        {
            float a = inputOutputGradients[dstBase + modWorkGroupOuputSize + itemId];
            weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + modWorkGroupOuputSize + itemId] = a;
        }
        
        // calculate matrix weight Gradient
        for (uint j = 0; j < outputSize; ++j)
        {
            float scale = inputOutputGradients[dstBase + j];
            uint weightRowOffset = j * width + weightAndBiasGradientOffset;
        
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


ndSharedPtr<ndBrainGpuShader> ndBrainGpuContext::CreateKerner(const cl::Program& program, const char* const functionMame) const
{
    cl_int errcode_ret = 0;
    ndSharedPtr<ndBrainGpuShader> kernel(ndSharedPtr<ndBrainGpuShader>(new ndBrainGpuShader(program, functionMame, &errcode_ret)));
    ndAssert(errcode_ret == 0);
    return kernel;
}

//#include <vector>
void ndBrainGpuContext::CreateKerners()
{
    std::string source(m_commonKernelsSource);
    source += m_matrixMultiply;
    source += m_optimizerKernels;
    source += m_feedForwardKernels_1;
    source += m_feedForwardKernels_2;
    source += m_feedForwardKernels_3;
    source += m_backPropagateKernels_1;
    source += m_backPropagateKernels_2;

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
    m_brainLayerMatrixMatrixMultiply = CreateKerner(program, "brainLayerMatrixMatrix");
    m_brainLayerSoftmaxActivation = CreateKerner(program, "brainLayerSoftmaxActivation");
    m_brainLayerDropOutActivation = CreateKerner(program, "brainLayerDropOutActivation");

    // create all backpropagate shaders
    m_brainCopyInputGradients = CreateKerner(program, "brainCopyInputGradients");
    m_brainCopyOutputGradients = CreateKerner(program, "brainCopyOutputGradients");
    m_brainLayerReluBackPropagate = CreateKerner(program, "brainLayerBrainReluBackPropagate");
    m_brainLayerTanhBackPropagate = CreateKerner(program, "brainLayerBrainTanhBackPropagate");
    m_brainLayerDropOutBackPropagate = CreateKerner(program, "brainLayerBrainDropOutBackPropagate");
    m_brainLayerMatrixVectorBackPropagate = CreateKerner(program, "brainLayerBrainLinearBackPropagate");
    m_brainLayerCathegoricalSoftmaxBackPropagate = CreateKerner(program, "brainLayerBrainCathegoricalSoftmaxBackPropagate");

    // accumulate gradient kernels
    m_brainAverageGradients = CreateKerner(program, "brainAverageGradients");
    m_brainAccumulateGradients = CreateKerner(program, "brainAccumulateGradients");
    m_brainAccumulateGradientsAndAverage = CreateKerner(program, "brainAccumulateGradientsAndAverage");

    // optimizer kernels
    m_brainAdamMomentumUpdate = CreateKerner(program, "brainAdamMomentumUpdate");
    m_brainAdamRidgeOptimizerUpdate = CreateKerner(program, "brainAdamUpdateRidgeRegularizer");
    m_brainAdamLassoOptimizerUpdate = CreateKerner(program, "brainAdamUpdateLassoRegularizer");
}