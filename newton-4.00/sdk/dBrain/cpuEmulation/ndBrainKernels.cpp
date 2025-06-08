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
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"

//#define ND_GPU_LOCAL_BUFFER_SIZE	1024
#define ND_GPU_LOCAL_BUFFER_SIZE	512

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
        UniformBufferObject* const parameters = &buffer0->m_data;
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
        UniformBufferObject* const parameters = &buffer0->m_data;
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
        //ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        UniformBufferObject* const parameters = &buffer0->m_data;
        //ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const miniBatchGradients = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * outputSize;
        ndInt32 dstBase = groupId * inputOutputSize + inputOutputStartOffset;

        ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;

        ndBrainMemVector xxxx(&inputOutputGradients[dstBase], outputSize);
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

class brainLayerLinear : public ndBrainGpuShader
{
    public:
    brainLayerLinear(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloat cachedInput [ND_GPU_LOCAL_BUFFER_SIZE * 2];
        ndBrainFloat cachedOutput [ND_GPU_LOCAL_BUFFER_SIZE * 2];
        ndBrainFloat reductionBuffer [ND_GPU_LOCAL_BUFFER_SIZE];
        
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const weightsAndBias = &buffer2->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);
        
        ndInt32 biasOffset = outputSize * inputSize + parametersStartOffset;
        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + inputSize;
            
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            cachedInput[modWorkGroupSize + itemId] = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        }
            
        ndInt32 rowCountReminder = outputSize % workGroupSize;
        ndInt32 modRowCount = inputSize - rowCountReminder;

        for (ndInt32 i = 0; i < modRowCount; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedOutput[i + itemId] = weightsAndBias[biasOffset + itemId];
            }
        }
        //if (itemId < rowCountReminder)
        for (ndInt32 itemId = 0; itemId < rowCountReminder; ++itemId)
        {
            cachedOutput[modRowCount + itemId] = weightsAndBias[biasOffset + modRowCount + itemId];
        }
            
        for (ndInt32 i = 0; i < outputSize; ++i)
        {
            ndBrainFloat partialSum[ND_GPU_LOCAL_BUFFER_SIZE * 2];
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                partialSum[itemId] = 0.0f;
            }
            ndInt32 rowStartOffset = i * inputSize + parametersStartOffset;
            for (ndInt32 j = 0; j < modWorkGroupSize; j += workGroupSize)
            {
                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    ndBrainFloat b = cachedInput[itemId + j];
                    ndBrainFloat a = weightsAndBias[rowStartOffset + itemId + j];
                    partialSum[itemId] += a * b;
                }
            }
            //if (itemId < workGroupSizeReminder)
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                ndBrainFloat b = cachedInput[modWorkGroupSize + itemId];
                ndBrainFloat a = weightsAndBias[rowStartOffset + modWorkGroupSize + itemId];
                partialSum[itemId] += a * b;
            }
                
            for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
            {
                //if ((itemId >= j) && (itemId < j * 2))
                for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
                {
                    reductionBuffer[itemId - j] = partialSum[itemId];
                }
                // barrier here
                //if (itemId < j)
                for (ndInt32 itemId = 0; itemId < j; ++itemId)
                {
                    partialSum[itemId] += reductionBuffer[itemId];
                }
            }
            //if (itemId == 0)
            {
                cachedOutput[i] += partialSum[0];
            }
        }
            
        for (ndInt32 i = 0; i < modRowCount; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                inputOutputData[outputOffset + i + itemId] = cachedOutput[i + itemId];
            }
        }
        for (ndInt32 itemId = 0; itemId < rowCountReminder; ++itemId)
        {
            inputOutputData[outputOffset + modRowCount + itemId] = cachedOutput[modRowCount + itemId];
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
        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + inputSize;

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = (inputValue >= 0.0) ? inputValue : 0.0f;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = (inputValue >= 0.0) ? inputValue : 0.0f;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
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
        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + inputSize;

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
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
        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + inputSize;

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = (inputValue > ndBrainFloat(-30.0f)) ? ((inputValue < ndBrainFloat(30.0f)) ? inputValue : ndBrainFloat(30.0f)) : ndBrainFloat(-30.0f);
                inputOutputData[outputOffset + i + itemId] = ndTanh(outputValue);
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = (inputValue > ndBrainFloat (-30.0f)) ? ((inputValue < ndBrainFloat(30.0f)) ? inputValue : ndBrainFloat(30.0f)) : ndBrainFloat (-30.0f);
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = ndTanh(outputValue);
        }
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
        ndBrainFloat tmpInputBuffer[ND_GPU_LOCAL_BUFFER_SIZE * 2];
        ndBrainFloat reductionBuffer[ND_GPU_LOCAL_BUFFER_SIZE];

        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + inputSize;

        //ndBrainFloat maxArg = ndBrainFloat (-1.0e30f);
        ndBrainFloat maxArg[ND_GPU_LOCAL_BUFFER_SIZE * 2];
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
            //if ((itemId >= j) && (itemId < j * 2))
            for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
            {
                reductionBuffer[itemId - j] = maxArg[itemId];
            }
        
            //if (itemId < j)
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                maxArg[itemId] = (inputValue > maxArg[itemId]) ? inputValue : maxArg[itemId];
            }
        }
        //if (itemId == 0)
        {
            reductionBuffer[0] = maxArg[0];
        }

        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            maxArg[itemId] = reductionBuffer[0];
        }
      
        ndBrainFloat sumArg[ND_GPU_LOCAL_BUFFER_SIZE * 2];
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
        //if (itemId < workGroupSizeReminder)
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId] - maxArg[itemId];
            ndBrainFloat outputValue = exp(inputValue);
            sumArg[itemId] += outputValue;
            tmpInputBuffer[modWorkGroupSize + itemId] = outputValue;
        }
        
        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            //if ((itemId >= j) && (itemId < j * 2))
            for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
            {
                reductionBuffer[itemId - j] = sumArg[itemId];
            }
        
            //if (itemId < j)
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                sumArg[itemId] += inputValue;
            }
        }
        //if (itemId == 0)
        {
            reductionBuffer[0] = 1.0f / sumArg[0];
        }
        
        ndBrainFloat invDen = reductionBuffer[0];
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = tmpInputBuffer[i + itemId];
                ndBrainFloat outputValue = invDen * inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        //if (itemId < workGroupSizeReminder)
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId];
            ndBrainFloat outputValue = invDen * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
};


// back propagation kernels
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
        //ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        //ndBrainFloat* const outputBuffer = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        //ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        //ndInt32 srcBase = groupId * outputSize;
        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + inputSize;

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        //ndBrainMemVector xxxx0(&inputOutputData[dstBase], inputSize);
        //ndBrainMemVector xxxx1(&inputOutputGradients[dstBase], inputSize);
        //ndBrainMemVector xxxx(&inputOutputGradients[srcBase], inputSize);
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

        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + inputSize;

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        ///ndBrainMemVector xxxx0(&inputOutputData[srcBase], inputSize);
        //ndBrainMemVector xxxx1(&inputOutputGradients[dstBase], inputSize);
        //ndBrainMemVector xxxx(&inputOutputGradients[srcBase], inputSize);
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
        //ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        UniformBufferObject* const parameters = &buffer0->m_data;
        //ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + inputSize;

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        //ndBrainMemVector xxxx0(&inputOutputData[srcBase], inputSize);
        //ndBrainMemVector xxxx1(&inputOutputGradients[dstBase], inputSize);
        //ndBrainMemVector xxxx(&inputOutputGradients[srcBase], inputSize);
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
        //ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        ndBrainGpuFloatBuffer* const buffer3 = (ndBrainGpuFloatBuffer*)m_parameters[3];

        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        //ndBrainFloat* const outputBuffer = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        //ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        //ndInt32 srcBase = groupId * outputSize;
        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + inputSize;

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        //ndBrainMemVector xxxx0(&inputOutputData[dstBase], inputSize);
        //ndBrainMemVector xxxx1(&inputOutputGradients[dstBase], inputSize);
        //ndBrainMemVector xxxx(&inputOutputGradients[srcBase], inputSize);
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

class brainLayerBrainLinearBackPropagate : public ndBrainGpuShader
{
    public:
    brainLayerBrainLinearBackPropagate(ndBrainGpuContext* const context)
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

        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const weightAndBias = &buffer2->m_buffer[0];
        ndBrainFloat* const inputOutputGradients = &buffer3->m_buffer[0];
        ndBrainFloat* const weightAndBiasGradients = &buffer4->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        ndBrainFloat cachedInput[ND_GPU_LOCAL_BUFFER_SIZE * 2];
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = ndBrainFloat (0.0f);
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            cachedInput[modWorkGroupSize + itemId] = ndBrainFloat(0.0f);
        }

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + inputSize;
        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);

        // calculate input gradients
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt32 weightOffset = j * inputSize + parametersStartOffset;
            for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
            {
                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    ndBrainFloat weight = weightAndBias[weightOffset + itemId];
                    cachedInput[i + itemId] += weight * scale;
                }
            }
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                ndBrainFloat weight = weightAndBias[modWorkGroupSize + itemId];
                cachedInput[modWorkGroupSize + itemId] += weight * scale;
            }
        }

        // store input gradients
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
        ndInt32 matrixSize = inputSize * outputSize;
        ndInt32 workGroupbOuputSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupOuputSize = outputSize - workGroupbOuputSizeReminder;
        ndInt32 weightAndBiasGradientOffset = groupId * ndInt32(parameters->m_parametersBatchSize) + parametersStartOffset;
        
        for (ndInt32 i = 0; i < modWorkGroupOuputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                float a = inputOutputGradients[dstBase + i + itemId];
                weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupbOuputSizeReminder; ++itemId)
        {
            float a = inputOutputGradients[dstBase + modWorkGroupOuputSize + itemId];
            weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + modWorkGroupOuputSize + itemId] = a;
        }
        
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt32 weightRowOffset = j * inputSize + weightAndBiasGradientOffset;

            //ndBrainMemVector xxxx(&weightAndBiasGradients[weightRowOffset], inputSize);
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
    m_ndBrainCopyInput = ndSharedPtr<ndBrainGpuShader> (new brainCopyInput(this));
    m_ndBrainCopyOutput = ndSharedPtr<ndBrainGpuShader> (new brainCopyOutput(this));
    m_ndBrainLayerLinear = ndSharedPtr<ndBrainGpuShader> (new brainLayerLinear(this));
    m_ndBrainLayerReluActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerReluActivation(this));
    m_ndBrainLayerTanhActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerTanhActivation(this));
    m_ndBrainLayerSoftmaxActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerSoftmaxActivation(this));
    m_ndBrainLayerLinearDropOutActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerLinearDropOutActivation(this));

    // create all backpropagate shaders
    m_ndBrainCopyOutputGradients = ndSharedPtr<ndBrainGpuShader>(new brainCopyOutputGradients(this));
    m_ndBrainLayerReluBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainReluBackPropagate(this));
    m_ndBrainLayerTanhBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainTanhBackPropagate(this));
    m_ndBrainLayerLinearBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainLinearBackPropagate(this));
    m_ndBrainLayerLinearDropOutBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainLinearDropOutBackPropagate(this));
    m_ndBrainLayerCathegoricalSoftmaxBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainCathegoricalSoftmaxBackPropagate(this));
}