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
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        UniformBufferObject* const parameters = &buffer0->m_data;
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const outputBuffer = &buffer2->m_buffer[0];

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
                ndBrainFloat a = outputBuffer[srcBase + i + itemId];
                inputOutputData[dstBase + i + itemId] = a;
            }
        }

        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = outputBuffer[srcBase + modWorkGroupSize + itemId];
            inputOutputData[dstBase + modWorkGroupSize + itemId] = a;
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
            
        //ndInt32 workGroupCount = inputSize / workGroupSize;
        //ndInt32 modWorkGroupSize = workGroupCount * workGroupSize;
        //ndInt32 workGroupSizeReminder = inputSize - modWorkGroupSize;
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
            }
        }
        //if (itemId < workGroupSizeReminder)
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            cachedInput[modWorkGroupSize + itemId] = inputOutputData[inputOffset + modWorkGroupSize + itemId];
        }
            
        //ndInt32 roundRowCount = outputSize / workGroupSize;
        //ndInt32 modRowCount = roundRowCount * workGroupSize;
        //ndInt32 rowCountReminder = outputSize - modRowCount;
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
}