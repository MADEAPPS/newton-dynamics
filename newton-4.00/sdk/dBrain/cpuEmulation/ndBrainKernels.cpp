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

class brainCopyInput : public ndBrainGpuContext::ndBrainGpuShader
{
    public:
    brainCopyInput(ndBrainGpuContext* const context)
        :ndBrainGpuContext::ndBrainGpuShader(context)
    {
    }

    void Execute(const UniformBufferObject* const parameters, float* const inputOutputData, float* const inputBuffer)
    {
        ndInt32 groupId = m_groupId;
        ndInt32 workGroupSize = m_workGroupSize;
        
        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);
        
        ndInt32 srcBase = groupId * inputSize;
        ndInt32 dstBase = groupId * inputOutputSize + inputOutputStartOffset;
        
        ndInt32 iterations = inputSize / workGroupSize;
        ndInt32 modWorkGroupSize = iterations * workGroupSize;
        ndInt32 reminderworkGroupSize = inputSize - modWorkGroupSize;
            
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                float a = inputBuffer[srcBase + i + itemId];
                inputOutputData[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < reminderworkGroupSize; ++itemId)
        {
            float a = inputBuffer[srcBase + modWorkGroupSize + itemId];
            inputOutputData[dstBase + modWorkGroupSize + itemId] = a;
        }
    }
};

class brainCopyOutput : public ndBrainGpuContext::ndBrainGpuShader
{
    public:
    brainCopyOutput(ndBrainGpuContext* const context)
        :ndBrainGpuContext::ndBrainGpuShader(context)
    {
    }

    void Execute(const UniformBufferObject* const parameters, float* const inputOutputData, float* const outputBuffer)
    {
        //ndInt32 itemId = get_local_id(0);
        //ndInt32 groupId = get_group_id(0);
        //ndInt32 workGroupSize = get_local_size(0);

        ndInt32 groupId = m_groupId;
        ndInt32 workGroupSize = m_workGroupSize;

        //ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);
            
        ndInt32 dstBase = groupId * outputSize;
        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
            
        ndInt32 iterations = outputSize / workGroupSize;
        ndInt32 modWorkGroupSize = iterations * workGroupSize;
        ndInt32 workGroupSizeReminder = outputSize - modWorkGroupSize;
            
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                float a = inputOutputData[srcBase + itemId];
                outputBuffer[dstBase + i + itemId] = a;
            }
        }
            
        //if (itemId < workGroupSizeReminder)
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            float a = inputOutputData[srcBase + modWorkGroupSize + itemId];
            outputBuffer[dstBase + modWorkGroupSize + itemId] = a;
            //if (itemId == 0) printf ("%d %d %d", groupId, workGroupSize, get_num_groups(0));
            //outputBuffer[dstBase + modWorkGroupSize + itemId] = groupId * 100 + itemId;
        }
    }
};

class brainLayerLinear : public ndBrainGpuContext::ndBrainGpuShader
{
    public:
    brainLayerLinear(ndBrainGpuContext* const context)
        :ndBrainGpuContext::ndBrainGpuShader(context)
    {
    }

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    void Execute(const UniformBufferObject* const parameters, float* const inputOutputData, float* const weightsAndBias)
    {
        float cachedInput [1024 * 2];
        float cachedOutput [1024 * 2];
        float reductionBuffer [1024];
        
        ndInt32 groupId = m_groupId;
        ndInt32 workGroupSize = m_workGroupSize;
        
        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);
        
        ndInt32 biasOffset = outputSize * inputSize + parametersStartOffset;
        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + inputSize;
            
        ndInt32 workGroupCount = inputSize / workGroupSize;
        ndInt32 modWorkGroupSize = workGroupCount * workGroupSize;
        ndInt32 workGroupSizeReminder = inputSize - modWorkGroupSize;
        
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
            
        ndInt32 roundRowCount = outputSize / workGroupSize;
        ndInt32 modRowCount = roundRowCount * workGroupSize;
        ndInt32 rowCountReminder = outputSize - modRowCount;
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
            float partialSum[1024 * 2];
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                partialSum[itemId] = 0.0f;
            }
            ndInt32 rowStartOffset = i * inputSize + parametersStartOffset;
            for (ndInt32 j = 0; j < modWorkGroupSize; j += workGroupSize)
            {
                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    float b = cachedInput[itemId + j];
                    float a = weightsAndBias[rowStartOffset + itemId + j];
                    partialSum[itemId] += a * b;
                }
            }
            //if (itemId < workGroupSizeReminder)
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                float b = cachedInput[modWorkGroupSize + itemId];
                float a = weightsAndBias[rowStartOffset + modWorkGroupSize + itemId];
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
        //if (itemId < rowCountReminder)
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            inputOutputData[outputOffset + modRowCount + itemId] = cachedOutput[modRowCount + itemId];
        }
    }
};

void ndBrainGpuContext::CreateKerners()
{
    m_ndBrainCopyInput = ndSharedPtr<ndBrainGpuShader> (new brainCopyInput(this));
    m_ndBrainCopyOutput = ndSharedPtr<ndBrainGpuShader> (new brainCopyOutput(this));
    m_ndBrainLayerLinear = ndSharedPtr<ndBrainGpuShader> (new brainLayerLinear(this));
}