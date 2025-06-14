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

class brainLayerMatrixTimeVector : public ndBrainGpuShader
{
    public:
    brainLayerMatrixTimeVector(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        // these local buffers have to be largest that the max hidden buffer size
        ndBrainFloat cachedInput[ND_GPU_LOCAL_BUFFER_SIZE * 2];
        ndBrainFloat accumulator[ND_GPU_LOCAL_BUFFER_SIZE * 2];
        ndAssert(ND_GPU_LOCAL_BUFFER_SIZE * 2 >= 1024);

        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const inputOutputData = &buffer1->m_buffer[0];
        ndBrainFloat* const weightsAndBias = &buffer2->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = ndInt32(parameters->m_inputOutputSize);
        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);
        ndInt32 inputOutputStartOffset = ndInt32(parameters->m_inputOutputStartOffset);

        ndInt32 biasOffset = outputSize * inputSize + parametersStartOffset;
        ndInt32 inputOffset = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        // cache the input
        ndInt32 workGroupInputSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupInputSize = inputSize - workGroupInputSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupInputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = inputOutputData[inputOffset + itemId + i];
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupInputSizeReminder; ++itemId)
        {
            cachedInput[modWorkGroupInputSize + itemId] = inputOutputData[inputOffset + itemId + modWorkGroupInputSize];
        }

        // copy the matrix bias
        //for (ndInt32 i = 0; i < outputSize; i += workGroupSize)
        //{
        //    for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        //    {
        //        accumulator[i + itemId] = 0.0f;
        //    }
        //}
        ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                accumulator[i + itemId] = weightsAndBias[biasOffset + i + itemId];
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            accumulator[modWorkGroupSize + itemId] = weightsAndBias[biasOffset + modWorkGroupSize + itemId];
        }

        for (ndInt32 i = 0; i < inputSize; ++i)
        {
            float scaleScale = cachedInput[i];
            ndInt32 rowStartOffset = i * outputSize + parametersStartOffset;

            for (ndInt32 j = 0; j < modWorkGroupSize; j += workGroupSize)
            {
                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    float matrixElement = weightsAndBias[rowStartOffset + j + itemId];
                    accumulator[j + itemId] += matrixElement * scaleScale;
                }
            }
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                float matrixElement = weightsAndBias[rowStartOffset + modWorkGroupSize + itemId];
                accumulator[modWorkGroupSize + itemId] += matrixElement * scaleScale;
            }
        }

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                float value = accumulator[i + itemId];
                inputOutputData[outputOffset + i + itemId] = value;
            }
        }

        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            float value = accumulator[modWorkGroupSize + itemId];
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = value;
        }
    }
};

class brainLayerTransposeMatrixBias : public ndBrainGpuShader
{
    public:
    brainLayerTransposeMatrixBias(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }

    // copy the bias from to the transpose matrix, whis is not transposed
    void Execute(ndInt32, ndInt32)
    {
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];

        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const weightMatrixBuffer = &buffer1->m_buffer[0];
        ndBrainFloat* const transposeWeightMatrixBuffer = &buffer2->m_buffer[0];

        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);
        ndInt32 biasOffset = outputSize * inputSize + parametersStartOffset;

        ndBrainMemVector xxx0(&weightMatrixBuffer[biasOffset], outputSize);
        ndBrainMemVector xxx1(&transposeWeightMatrixBuffer[biasOffset], outputSize);

        // copy the matrix bias
        //for (ndInt32 i = 0; i < outputSize; i += workGroupSize)
        //{
        //    for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        //    {
        //        float a = weightMatrixBuffer[biasOffset + itemId];
        //        transposeWeightMatrixBuffer[biasOffset + itemId] = a;
        //    }
        //}
        for (ndInt32 itemId = 0; itemId < outputSize; ++itemId)
        {
            float a = weightMatrixBuffer[biasOffset + itemId];
            transposeWeightMatrixBuffer[biasOffset + itemId] = a;
        }
    }
};

class brainLayerTransposeMatrix : public ndBrainGpuShader
{
    public:
    brainLayerTransposeMatrix(ndBrainGpuContext* const context)
        :ndBrainGpuShader(context)
    {
    }


    // transpose one tile of a (16 x 16) block matrix.
    void Execute(ndInt32 groupId, ndInt32)
    {
        ndBrainFloat tileBlock[16][16];
        
        ndBrainGpuUniformBuffer* const buffer0 = (ndBrainGpuUniformBuffer*)m_parameters[0];
        ndBrainGpuFloatBuffer* const buffer1 = (ndBrainGpuFloatBuffer*)m_parameters[1];
        ndBrainGpuFloatBuffer* const buffer2 = (ndBrainGpuFloatBuffer*)m_parameters[2];
        
        ndBrainLayer::ndCommandShareInfo* const parameters = (ndBrainLayer::ndCommandShareInfo*)&buffer0->m_data[0];
        ndBrainFloat* const weightMatrixBuffer = &buffer1->m_buffer[0];
        ndBrainFloat* const transposeWeightMatrixBuffer = &buffer2->m_buffer[0];
        
        ndInt32 inputSize = ndInt32(parameters->m_inputSize);
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 numberOfTile_x = ((inputSize + 15) & -16) / 16;
        //ndInt32 numberOfTile_y = ((outputSize + 15) & -16) / 16;

        ndInt32 blockId_x = groupId % numberOfTile_x;
        ndInt32 blockId_y = (groupId - blockId_x) / numberOfTile_x;

        ndInt32 tile_offset_x = blockId_x * 16;
        ndInt32 tile_count_x = ((tile_offset_x + 16) < inputSize) ? 16 : inputSize - tile_offset_x;
        ndAssert(tile_count_x > 0);

        ndInt32 tile_offset_y = blockId_y * 16;
        ndInt32 tile_count_y = ((tile_offset_y + 16) < outputSize) ? 16 : outputSize - tile_offset_y;
        ndAssert(tile_count_y > 0);

        if ((tile_count_x < 16) || (tile_count_y < 16))
        {
            for (ndInt32 threadId_y = 0; threadId_y < 16; ++threadId_y)
            {
                for (ndInt32 threadId_x = 0; threadId_x < 16; ++threadId_x)
                {
                    tileBlock[threadId_y][threadId_x] = 0.0f;
                }
            }
        }

        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);
        ndInt32 srcStartOffset = 16 * blockId_y * inputSize + tile_offset_x + parametersStartOffset;
        for (ndInt32 threadId_y = 0; threadId_y < tile_count_y; ++threadId_y)
        {
            for (ndInt32 threadId_x = 0; threadId_x < tile_count_x; ++threadId_x)
            {
                // read and transpose source tile.
                float element = weightMatrixBuffer[srcStartOffset + threadId_x];
                tileBlock[threadId_x][threadId_y] = element;
            }
            srcStartOffset += inputSize;
        }
        // block all thread here

        ndInt32 dstStartOffset = 16 * blockId_x * outputSize + tile_offset_y + parametersStartOffset;
        for (ndInt32 threadId_y = 0; threadId_y < tile_count_x; ++threadId_y)
        {
            ndBrainMemVector xxx1(&tileBlock[threadId_y][0], tile_count_y);
            ndBrainMemVector xxx0(&transposeWeightMatrixBuffer[dstStartOffset], tile_count_y);
            for (ndInt32 threadId_x = 0; threadId_x < tile_count_y; ++threadId_x)
            {
                float element = tileBlock[threadId_y][threadId_x];
                ndAssert(element == transposeWeightMatrixBuffer[dstStartOffset + threadId_x]);
                transposeWeightMatrixBuffer[dstStartOffset + threadId_x] = element;
            }
            dstStartOffset += outputSize;
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

        //ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        //ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputOutputData[dstBase + i + itemId];
                a -= inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = a;
            }
        }

        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    ndBrainFloat a = inputOutputData[dstBase + modWorkGroupSize + itemId];
        //    a -= inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a;
        //}
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

        //ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        //ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inpuData = inputOutputData[srcBase + i + itemId];
                ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
            }
        }

        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    ndBrainFloat inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
        //    ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
        //    ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        //}
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

        //ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        //ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = outputGrad;
            }
        }

        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = outputGrad;
        //}
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

        //ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        //ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat outputData = inputOutputData[dstBase + i + itemId];
                ndBrainFloat a = ndBrainFloat(1.0f) - outputData * outputData;
                ndBrainFloat b = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = a * b;
            }
        }

        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    ndBrainFloat outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
        //    ndBrainFloat a = ndBrainFloat(1.0f) - outputData * outputData;
        //    ndBrainFloat b = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a * b;
        //}
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

        
        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = ndBrainFloat (0.0f);
            }
        }
        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    cachedInput[modWorkGroupSize + itemId] = ndBrainFloat(0.0f);
        //}

        ndInt32 srcBase = groupId * inputOutputSize + inputOutputStartOffset;
        ndInt32 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndInt32 parametersStartOffset = ndInt32(parameters->m_parametersStartOffset);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        // calculate input gradients
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt32 weightOffset = j * inputSize + parametersStartOffset;
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

        //for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                inputOutputGradients[srcBase + i + itemId] = cachedInput[i + itemId];
                cachedInput[i + itemId] = inputOutputData[srcBase + i + itemId];
            }
        }
        //for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        //{
        //    inputOutputGradients[srcBase + modWorkGroupSize + itemId] = cachedInput[modWorkGroupSize + itemId];
        //    cachedInput[modWorkGroupSize + itemId] = inputOutputData[srcBase + modWorkGroupSize + itemId];
        //}

        // calculate weights and bias gradients
        ndInt32 matrixSize = inputSize * outputSize;
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
            ndInt32 weightRowOffset = j * inputSize + weightAndBiasGradientOffset;

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

void ndBrainGpuContext::CreateKerners()
{   
    // create all feed foward shaders
    m_brainCopyInput = ndSharedPtr<ndBrainGpuShader> (new brainCopyInput(this));
    m_brainCopyOutput = ndSharedPtr<ndBrainGpuShader> (new brainCopyOutput(this));
    m_brainLayerLinear = ndSharedPtr<ndBrainGpuShader> (new brainLayerMatrixTimeVector(this));
    m_brainLayerReluActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerReluActivation(this));
    m_brainLayerTanhActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerTanhActivation(this));
    m_brainLayerSoftmaxActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerSoftmaxActivation(this));
    m_brainLayerDropOutActivation = ndSharedPtr<ndBrainGpuShader>(new brainLayerLinearDropOutActivation(this));

    // create all backpropagate shaders
    m_brainCopyInputGradients = ndSharedPtr<ndBrainGpuShader>(new brainCopyInputGradients(this));
    m_brainCopyOutputGradients = ndSharedPtr<ndBrainGpuShader>(new brainCopyOutputGradients(this));
    m_brainLayerReluBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainReluBackPropagate(this));
    m_brainLayerTanhBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainTanhBackPropagate(this));
    m_brainLayerLinearBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainLinearBackPropagate(this));
    m_brainLayerDropOutBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainLinearDropOutBackPropagate(this));
    m_brainLayerCathegoricalSoftmaxBackPropagate = ndSharedPtr<ndBrainGpuShader>(new brainLayerBrainCathegoricalSoftmaxBackPropagate(this));

    // miscellaneous
    m_brainLayerTransposeMatrix = ndSharedPtr<ndBrainGpuShader>(new brainLayerTransposeMatrix(this));
    m_brainLayerTransposeMatrixBias = ndSharedPtr<ndBrainGpuShader>(new brainLayerTransposeMatrixBias(this));

    // accumulate gradient kernels
    m_brainAccumulateGradients = ndSharedPtr<ndBrainGpuShader>(new brainAccumulateGradients(this));

    // optimizer kernels
    m_brainAdamMomentumUpdate = ndSharedPtr<ndBrainGpuShader>(new brainAdamMomentumUpdate(this));
    m_brainAdamRidgeOptimizerUpdate = ndSharedPtr<ndBrainGpuShader>(new brainAdamUpdateRidgeRegularizer(this));
    m_brainAdamLassoOptimizerUpdate = ndSharedPtr<ndBrainGpuShader>(new brainAdamUpdateLassoRegularizer(this));
}