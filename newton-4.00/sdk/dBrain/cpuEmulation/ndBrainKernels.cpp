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
#include "ndBrainLayer.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"
#include "ndBrainOptimizerAdamGpu.h"

#define ND_GPU_LOCAL_BUFFER_SIZE	512

inline ndInt32 __cpuKernelRoundoff(ndInt32 value, ndInt32 workgroupSize)
{
    return (value + workgroupSize - 1) & -workgroupSize;
}

class brainCopyInput : public ndBrainGpuShader
{
    public:
    brainCopyInput(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const inputBuffer = &buffer2->m_buffer[0];
        
        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);
        
        ndInt32 srcBase = groupId * inputSize;
        ndInt32 dstBase = groupId * inputOutputSize + inputOutputStartOffset;
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputBuffer[srcBase + i + itemId];
                inputOutputData[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputBuffer[srcBase + modWorkGroupSize + itemId];
            inputOutputData[dstBase + modWorkGroupSize + itemId] = a;
        }
    }
};

class brainCopyOutput : public ndBrainGpuShader
{
    public:
    brainCopyOutput(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const outputBuffer = &buffer2->m_buffer[0];

        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);
            
        ndInt32 dstBase = groupId * outputSize;
        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
            
        ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;
            
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputOutputData[srcBase + i + itemId];
                outputBuffer[dstBase + i + itemId] = a;
            }
        }
            
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputOutputData[srcBase + modWorkGroupSize + itemId];
            outputBuffer[dstBase + modWorkGroupSize + itemId] = a;
        }
    }
};

class brainLayerReluActivation : public ndBrainGpuShader
{
    public:
    brainLayerReluActivation(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        //ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        //ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ndBrainFloat(0.0f);
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        //    ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ndBrainFloat(0.0f);
        //    inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        //}
    }
};

class brainLayerLinearDropOutActivation : public ndBrainGpuShader
{
    public:
    brainLayerLinearDropOutActivation(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        //ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        //ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        //    ndBrainFloat outputValue = inputValue;
        //    inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        //}
    }
};

class brainLayerTanhActivation : public ndBrainGpuShader
{
    public:
        brainLayerTanhActivation(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        //ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        //ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = (inputValue > ndBrainFloat(-30.0f)) ? ((inputValue < ndBrainFloat(30.0f)) ? inputValue : ndBrainFloat(30.0f)) : ndBrainFloat(-30.0f);
                inputOutputData[outputOffset + i + itemId] = ndTanh(outputValue);
            }
        }
        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        //    ndBrainFloat outputValue = (inputValue > ndBrainFloat (-30.0f)) ? ((inputValue < ndBrainFloat(30.0f)) ? inputValue : ndBrainFloat(30.0f)) : ndBrainFloat (-30.0f);
        //    inputOutputData[outputOffset + modWorkGroupSize + itemId] = ndTanh(outputValue);
        //}
    }
};

class brainLayerSoftmaxActivation : public ndBrainGpuShader
{
    public:
    brainLayerSoftmaxActivation(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloat maxArg[ND_GPU_LOCAL_BUFFER_SIZE];
        ndBrainFloat sumArg[ND_GPU_LOCAL_BUFFER_SIZE];
        ndBrainFloat tmpInputBuffer[ND_GPU_LOCAL_BUFFER_SIZE];
        ndBrainFloat reductionBuffer[ND_GPU_LOCAL_BUFFER_SIZE];

        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            maxArg[itemId] = ndBrainFloat(-1.0e30f);
        }

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                tmpInputBuffer[i + itemId] = inputValue;
                maxArg[itemId] = (inputValue > maxArg[itemId]) ? inputValue : maxArg[itemId];
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            tmpInputBuffer[modWorkGroupSize + itemId] = inputValue;
            maxArg[itemId] = (inputValue > maxArg[itemId]) ? inputValue : maxArg[itemId];
        }
        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
            {
                reductionBuffer[itemId - j] = maxArg[itemId];
            }
        
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                maxArg[itemId] = (inputValue > maxArg[itemId]) ? inputValue : maxArg[itemId];
            }
        }
        reductionBuffer[0] = maxArg[0];

        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            maxArg[itemId] = reductionBuffer[0];
        }
        
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            sumArg[itemId] = ndBrainFloat(0.0f);
        }

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = tmpInputBuffer[i + itemId] - maxArg[itemId];
                ndBrainFloat outputValue = exp(inputValue);
                sumArg[itemId] += outputValue;
                tmpInputBuffer[i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId] - maxArg[itemId];
            ndBrainFloat outputValue = exp(inputValue);
            sumArg[itemId] += outputValue;
            tmpInputBuffer[modWorkGroupSize + itemId] = outputValue;
        }
        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
            {
                reductionBuffer[itemId - j] = sumArg[itemId];
            }
        
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                sumArg[itemId] += inputValue;
            }
        }
        reductionBuffer[0] = ndBrainFloat(1.0f) / sumArg[0];

        ndBrainFloat invDen = reductionBuffer[0];
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        //for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = tmpInputBuffer[i + itemId];
                ndBrainFloat outputValue = invDen * inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId];
            ndBrainFloat outputValue = invDen * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
};

// back propagation kernels
class brainCopyInputGradients : public ndBrainGpuShader
{
    public:
    brainCopyInputGradients(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const miniBatchGradients = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 dstBase = groupId * inputSize;
        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i + itemId];
                miniBatchGradients[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputOutputGradients[srcBase + modWorkGroupSize + itemId];
            miniBatchGradients[dstBase + modWorkGroupSize + itemId] = a;
        }
    }
};

class brainCopyOutputGradients : public ndBrainGpuShader
{
    public:
    brainCopyOutputGradients(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const miniBatchGradients = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * outputSize;
        ndInt32 dstBase = groupId * inputOutputSize + inputOutputStartOffset;

        ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = miniBatchGradients[srcBase + i + itemId];
                inputOutputGradients[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = miniBatchGradients[srcBase + modWorkGroupSize + itemId];
            inputOutputGradients[dstBase + modWorkGroupSize + itemId] = a;
        }
    }
};

class brainLayerBrainCathegoricalSoftmaxBackPropagate : public ndBrainGpuShader
{
    public:
    brainLayerBrainCathegoricalSoftmaxBackPropagate(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputOutputData[dstBase + i + itemId];
                a -= inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputOutputData[dstBase + modWorkGroupSize + itemId];
            a -= inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a;
        }
    }
};

class brainLayerBrainReluBackPropagate : public ndBrainGpuShader
{
    public:
    brainLayerBrainReluBackPropagate(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inpuData = inputOutputData[srcBase + i + itemId];
                ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
            }
        }

        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
            ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
            ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        }
    }
};

class brainLayerBrainLinearDropOutBackPropagate : public ndBrainGpuShader
{
    public:
    brainLayerBrainLinearDropOutBackPropagate(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = outputGrad;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = outputGrad;
        }
    }
};

class brainLayerBrainTanhBackPropagate : public ndBrainGpuShader
{
    public:
    brainLayerBrainTanhBackPropagate(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat outputData = inputOutputData[dstBase + i + itemId];
                ndBrainFloat a = ndBrainFloat(1.0f) - outputData * outputData;
                ndBrainFloat b = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = a * b;
            }
        }

        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
            ndBrainFloat a = ndBrainFloat(1.0f) - outputData * outputData;
            ndBrainFloat b = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a * b;
        }
    }
};

// extra kernels
class brainAccumulateGradients : public ndBrainGpuShader
{
    public:
    brainAccumulateGradients(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloat accRegister[ND_GPU_LOCAL_BUFFER_SIZE];

        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const gradientBuffer = &buffer1->m_buffer[0];

        //ndInt32 inputSize = ndInt32(__cpuKernelRoundoff(parameters->m_inputSize, workGroupSize);
        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 miniBatchSize = ndInt32(parameters->m_inputOutputSize);
     
        ndInt32 start = groupId * workGroupSize;
        ndBrainFloat weightFactor = ndBrainFloat(1.0f) / ndBrainFloat(miniBatchSize);
        
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            accRegister[itemId] = ndBrainFloat(0.0f);
        }
        for (ndInt32 j = 0; j < miniBatchSize; ++j)
        {
            ndInt32 base = start + j * inputSize;
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = gradientBuffer[base + itemId];
                accRegister[itemId] += a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndBrainFloat sum = accRegister[itemId];
            gradientBuffer[start + itemId] = sum * weightFactor;
        }
    }
};

class brainAdamUpdateLassoRegularizer : public ndBrainGpuShader
{
    public:
    brainAdamUpdateLassoRegularizer(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];
        ndBrainGpuFloatBuffer* const buffer4 = (ndBrainGpuFloatBuffer*)m_parameters[4];

        ndBrainOptimizerAdamGpu::ndCommandShareInfo* const parameters = (ndBrainOptimizerAdamGpu::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const weightAndBiasBuffer = &buffer1->m_buffer[0];
        ndBrainFloat* const weightAndBiasGradientBuffer = &buffer2->m_buffer[0];
        ndBrainFloat* const vdw = &buffer3->m_buffer[0];
        ndBrainFloat* const vdw2 = &buffer4->m_buffer[0];

        ndBrainFloat descendRate = -parameters->m_learnRate;
        ndBrainFloat regularizer = -parameters->m_decayRegularizer;

        ndInt32 start = groupId * workGroupSize;

        const ndBrainMemVector vdw__(&vdw[start], workGroupSize);
        const ndBrainMemVector vdw2__(&vdw[start], workGroupSize);
        const ndBrainMemVector weight___(&weightAndBiasBuffer[start], workGroupSize);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndBrainFloat temp = weightAndBiasGradientBuffer[start + itemId];
            ndBrainFloat a = vdw[start + itemId] * parameters->m_alpha + temp * (ndBrainFloat(1.0f) - parameters->m_alpha);
            vdw[start + itemId] = a;

            ndBrainFloat b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (ndBrainFloat(1.0f) - parameters->m_beta);
            vdw2[start + itemId] = b;

            ndBrainFloat vdwCorrected = a * parameters->m_invAlpha;
            ndBrainFloat vdw2Corrected = b * parameters->m_invBeta;

            ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected)) + parameters->m_epsilon);
            ndBrainFloat gradient = vdwCorrected * bias_den;

            ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
            ndBrainFloat lassoRegularizer = (weight >= ndBrainFloat(0.0f)) ? regularizer : -regularizer;
            gradient += weight * lassoRegularizer;
            weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
        }
    }
};

class brainAdamUpdateRidgeRegularizer : public ndBrainGpuShader
{
    public:
    brainAdamUpdateRidgeRegularizer(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];
        ndBrainGpuFloatBuffer* const buffer4 = (ndBrainGpuFloatBuffer*)m_parameters[4];
        
        ndBrainOptimizerAdamGpu::ndCommandShareInfo* const parameters = (ndBrainOptimizerAdamGpu::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const weightAndBiasBuffer = &buffer1->m_buffer[0];
        ndBrainFloat* const weightAndBiasGradientBuffer = &buffer2->m_buffer[0];
        ndBrainFloat* const vdw = &buffer3->m_buffer[0];
        ndBrainFloat* const vdw2 = &buffer4->m_buffer[0];

        ndBrainFloat descendRate = -parameters->m_learnRate;
        ndBrainFloat regularizer = -parameters->m_decayRegularizer;

        ndInt32 start = groupId * workGroupSize;

        //const ndBrainMemVector vdw__(&vdw[start], workGroupSize);
        //const ndBrainMemVector vdw2__(&vdw[start], workGroupSize);
        //const ndBrainMemVector weight___(&weightAndBiasBuffer[start], workGroupSize);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            //temp.Set(grad);
            ndBrainFloat temp = weightAndBiasGradientBuffer[start + itemId];
            
            // calculate moving average
            //vdw.Scale(m_alpha);
            //vdw.ScaleAdd(temp, ndBrainFloat(1.0f) - m_alpha);

            ndBrainFloat a = vdw[start + itemId] * parameters->m_alpha + temp * (ndBrainFloat(1.0f) - parameters->m_alpha);
            vdw[start + itemId] = a;
            
            //// caluate RMS
            //vdw2.Scale(m_beta);
            //temp.Mul(temp);
            //vdw2.ScaleAdd(temp, ndBrainFloat(1.0) - m_beta);
            ndBrainFloat b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (ndBrainFloat(1.0f) - parameters->m_beta);
            vdw2[start + itemId] = b;
            
            //vdwCorrected.Set(vdw);
            //vdw2Corrected.Set(vdw2);
            //if (m_alphaAcc > ndBrainFloat(0.0f))
            //{
            //	vdwCorrected.Scale(ndBrainFloat(1.0f / (1.0f - m_alphaAcc)));
            //}
            //
            //if (m_betaAcc > ndBrainFloat(0.0f))
            //{
            //	vdw2Corrected.Scale(ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_betaAcc));
            //}

            ndBrainFloat vdwCorrected = a * parameters->m_invAlpha;
            ndBrainFloat vdw2Corrected = b * parameters->m_invBeta;
            
            //for (ndInt32 j = ndInt32(grad.GetCount() - 1); j >= 0; --j)
            //{
            //	ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected[j])) + m_epsilon);
            //	grad[j] = vdwCorrected[j] * bias_den;
            //}
            ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected)) + parameters->m_epsilon);
            ndBrainFloat gradient = vdwCorrected * bias_den;
             
            //ndBrainMemVector weights (&parameters[start], count);
            //switch (m_regularizerType)
            //{
            //	case m_lasso:
            //	{
            //		ndBrainFloat negativeRegularizer = -regularizer;
            //		for (ndInt32 j = ndInt32(grad.GetCount()) - 1; j >= 0; --j)
            //		{
            //			ndBrainFloat b = grad[j];
            //			grad[j] += (b > ndFloat32(0.0f)) ? regularizer : negativeRegularizer;
            //		}
            //		break;
            //	}
            //
            //	case m_ridge:
            //	{
            //		grad.ScaleAdd(weights, regularizer);
            //		break;
            //	}
            //
            //	case m_none:;
            //}
            ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
            gradient += weight * regularizer;
             
            //weights.ScaleAdd(grad, descendRate);
            weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
        }
    }
};

class brainAdamMomentumUpdate : public ndBrainGpuShader
{
    public:
    brainAdamMomentumUpdate(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    //void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    void Execute(ndInt32, ndInt32)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainOptimizerAdamGpu::ndCommandShareInfo* const parameters = (ndBrainOptimizerAdamGpu::ndCommandShareInfo*)&buffer0->m_data[0];

        parameters->m_betaAcc *= parameters->m_beta;
        parameters->m_alphaAcc *= parameters->m_alpha;
        if (parameters->m_betaAcc < ndBrainFloat(1.0e-6f))
        {
            parameters->m_betaAcc = ndBrainFloat(0.0f);
        }
        if (parameters->m_alphaAcc < ndBrainFloat(1.0e-6f))
        {
            parameters->m_alphaAcc = ndBrainFloat(0.0f);
        }
    }
};

// matrix vector operation kernels.
class brainLayerMatrixMatrixMultiply : public ndBrainGpuShader
{
    public:
    brainLayerMatrixMatrixMultiply(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloat tile_inputs[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS];
        ndBrainFloat tile_weights[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS];
        ndBrainFloat tile_acc[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];

        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];

        ndBrainFloat* const weightsAndBias = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];

        const ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        const ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        const ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;

        const ndInt32 groupId_x = groupId % parameters->m_tiledStride;
        const ndInt32 groupId_y = (groupId - groupId_x) / parameters->m_tiledStride;
        
        //Initialise the accumulation register
        const ndInt32 parametersStartOffset = groupId_x * width * ND_GPU_TILED_MATRIX_ROWS + ndInt32(parameters->m_parametersStartOffset);
        const ndInt32 parametersBiasOffset = width * height + ndInt32(parameters->m_parametersStartOffset);
        
        ndBrainFloat biasValue[ND_GPU_TILED_MATRIX_ROWS];
        for (ndInt32 itemId_y = 0; itemId_y < ND_GPU_TILED_MATRIX_ROWS; ++itemId_y)
        {
            biasValue[itemId_y] = weightsAndBias[parametersBiasOffset + itemId_y];
        }
        for (ndInt32 itemId_x = 0; itemId_x < ND_GPU_TILED_MATRIX_ROWS; ++itemId_x)
        {
            for (ndInt32 itemId_y = 0; itemId_y < ND_GPU_TILED_MATRIX_ROWS; ++itemId_y)
            {
                tile_acc[itemId_x][itemId_y] = biasValue[itemId_y];
            }
        }

        const ndInt32 inputOutputStride = ndInt32(parameters->m_inputOutputSize);
        const ndInt32 inputOffset = groupId_y * inputOutputStride * ND_GPU_TILED_MATRIX_ROWS + ndInt32(parameters->m_inputOutputStartOffset);
        
        // Loop over all tiles
        for (ndInt32 tile = 0; tile < width; tile += ND_GPU_TILED_MATRIX_COLUMNS)
        {
            // Load one tile of A and B into local memory
            ndInt32 inputStartOffset = tile + inputOffset;
            ndInt32 weightOffsetStart = tile + parametersStartOffset;
            for (ndInt32 itemId_y = 0; itemId_y < ND_GPU_TILED_MATRIX_ROWS; ++itemId_y)
            {
                for (ndInt32 itemId_x = 0; itemId_x < ND_GPU_TILED_MATRIX_COLUMNS; ++itemId_x)
                {
                    tile_weights[itemId_y][itemId_x] = weightsAndBias[weightOffsetStart + itemId_x];
                    tile_inputs[itemId_y][itemId_x] = inputOutputData[inputStartOffset + itemId_x];
                }
                weightOffsetStart += width;
                inputStartOffset += inputOutputStride;
            }
            //barrier
            
            // Perform the computation for a single tile
            for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_COLUMNS; ++i)
            {
                for (ndInt32 itemId_y = 0; itemId_y < ND_GPU_TILED_MATRIX_ROWS; itemId_y++)
                {
                    ndBrainFloat a = tile_weights[itemId_y][i];
                    for (ndInt32 itemId_x = 0; itemId_x < ND_GPU_TILED_MATRIX_ROWS; itemId_x++)
                    {
                        tile_acc[itemId_x][itemId_y] += a * tile_inputs[itemId_x][i];
                    }
                }
            }
            // barrier
        }

        const ndInt32 numberOutput = ((groupId_x + 1) * ND_GPU_TILED_MATRIX_ROWS < outputSize) ? ND_GPU_TILED_MATRIX_ROWS : outputSize - groupId_x * ND_GPU_TILED_MATRIX_ROWS;
        ndInt32 outputOffset = groupId_x * ND_GPU_TILED_MATRIX_ROWS + inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        for (ndInt32 itemId_y = 0; itemId_y < ND_GPU_TILED_MATRIX_ROWS; ++itemId_y)
        {
            for (ndInt32 itemId_x = 0; itemId_x < numberOutput; ++itemId_x)
            {
                float value = tile_acc[itemId_y][itemId_x];
                inputOutputData[outputOffset + itemId_x] = value;
            }
            outputOffset += inputOutputStride;
        }
    }
};

class brainLayerBrainLinearBackPropagate : public ndBrainGpuShader
{
    public:
    brainLayerBrainLinearBackPropagate(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloat cachedInput[ND_GPU_LOCAL_BUFFER_SIZE * 2];
        ndAssert(ND_GPU_LOCAL_BUFFER_SIZE * 2 >= 1024);

        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];
        ndBrainGpuFloatBuffer* const buffer4 = (ndBrainGpuFloatBuffer*)m_parameters[4];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const weightAndBias = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];
        ndBrainFloat* const weightAndBiasGradients = &buffer4->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = ndBrainFloat(0.0f);
            }
        }

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        // calculate input gradients
        ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt32 weightOffset = j * width + parametersStartOffset;
            for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
            {
                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    ndBrainFloat weight = weightAndBias[weightOffset + i + itemId];
                    cachedInput[i + itemId] += weight * scale;
                }
            }
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                ndBrainFloat weight = weightAndBias[weightOffset + modWorkGroupSize + itemId];
                cachedInput[modWorkGroupSize + itemId] += weight * scale;
            }
        }
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                inputOutputGradients[srcBase + i + itemId] = cachedInput[i + itemId];
                cachedInput[i + itemId] = inputOutputData[srcBase + i + itemId];
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = cachedInput[modWorkGroupSize + itemId];
            cachedInput[modWorkGroupSize + itemId] = inputOutputData[srcBase + modWorkGroupSize + itemId];
        }

        // calculate weights and bias gradients
        //ndInt32 matrixSize = inputSize * outputSize;
        ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        ndInt32 matrixSize = width * height;
        //ndInt32 workGroupOuputSizeReminder = outputSize % workGroupSize;
        //ndInt32 modWorkGroupOuputSize = outputSize - workGroupOuputSizeReminder;
        ndInt32 weightAndBiasGradientOffset = groupId * ndInt32(parameters->m_parametersBatchSize) + parametersStartOffset;

        // calculate bias Gradient
        //for (ndInt32 i = 0; i < modWorkGroupOuputSize; i += workGroupSize)
        for (ndInt32 i = 0; i < outputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                float a = inputOutputGradients[dstBase + i + itemId];
                weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + i + itemId] = a;
            }
        }
        //for (ndInt32 itemId = 0; itemId < workGroupOuputSizeReminder; ++itemId)
        //{
        //    float a = inputOutputGradients[dstBase + modWorkGroupOuputSize + itemId];
        //    weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + modWorkGroupOuputSize + itemId] = a;
        //}

        // calculate matrix weight Gradient
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt32 weightRowOffset = j * width + weightAndBiasGradientOffset;

            for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
            {
                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    ndBrainFloat inputValue = cachedInput[i + itemId];
                    ndBrainFloat weightGradient = inputValue * scale;
                    weightAndBiasGradients[weightRowOffset + i + itemId] = weightGradient;
                }
            }
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                ndBrainFloat inputValue = cachedInput[modWorkGroupSize + itemId];
                ndBrainFloat weightGradient = inputValue * scale;
                weightAndBiasGradients[weightRowOffset + modWorkGroupSize + itemId] = weightGradient;
            }
        }
    }
};

void ndBrainGpuContext::CreateKerners()
{   
    // create all feed foward shaders
    m_brainCopyInput = ndSharedPtr<ndBrainGpuShader> (new brainCopyInput(this));
    m_brainCopyOutput = ndSharedPtr<ndBrainGpuShader> (new brainCopyOutput(this));
    m_brainLayerReluActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerReluActivation(this));
    m_brainLayerTanhActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerTanhActivation(this));
    m_brainLayerSoftmaxActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerSoftmaxActivation(this));
    m_brainLayerDropOutActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerLinearDropOutActivation(this));
    m_brainLayerMatrixMatrixMultiply = ndSharedPtr<ndBrainGpuShader>(new brainLayerMatrixMatrixMultiply(this));

    // create all backpropagate shaders
    m_brainCopyInputGradients = ndSharedPtr<ndBrainGpuShader>(new brainCopyInputGradients(this));
    m_brainCopyOutputGradients = ndSharedPtr<ndBrainGpuShader>(new brainCopyOutputGradients(this));
    m_brainLayerReluBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainReluBackPropagate(this));
    m_brainLayerTanhBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainTanhBackPropagate(this));
    m_brainLayerMatrixVectorBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainLinearBackPropagate(this));
    m_brainLayerDropOutBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainLinearDropOutBackPropagate(this));
    m_brainLayerCathegoricalSoftmaxBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainCathegoricalSoftmaxBackPropagate(this));

    // accumulate gradient kernels
    m_brainAccumulateGradients = ndSharedPtr<ndBrainGpuShader>(new brainAccumulateGradients(this));

    // optimizer kernels
    m_brainAdamMomentumUpdate = ndSharedPtr<ndBrainGpuShader>(new brainAdamMomentumUpdate(this));
    m_brainAdamRidgeOptimizerUpdate = ndSharedPtr<ndBrainGpuShader>(new brainAdamUpdateRidgeRegularizer(this));
    m_brainAdamLassoOptimizerUpdate = ndSharedPtr<ndBrainGpuShader>(new brainAdamUpdateLassoRegularizer(this));
}