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
#include "ndBrainKernel.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainOptimizerAdam.h"

#define ND_GPU_USE_SOFT_SUBGROUPS
#define ND_GPU_SOFT_SUBGROUPS_WORD_SIZE 32
#define ND_GPU_LOCAL_BUFFER_SIZE	    1024 * 6

inline ndInt32 __cpuKernelRoundoff(ndInt32 value, ndInt32 workgroupSize)
{
    return (value + workgroupSize - 1) & -workgroupSize;
}

class brainCopyInput : public ndBrainKernel
{
    public:
    brainCopyInput(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
       ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
       ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
       ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

       ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
       ndBrainFloat* const inputBuffer = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
       ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputSize);
        ndInt64 dstBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
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

class brainCopyOutput : public ndBrainKernel
{
    public:
    brainCopyOutput(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const outputBuffer = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
            
        ndInt64 dstBase = groupId * (ndInt64)outputSize;
        ndInt64 srcBase = groupId * (ndInt64)inputOutputSize + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
            
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

class brainLayerReluActivation : public ndBrainKernel
{
    public:
    brainLayerReluActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ndBrainFloat(0.0f);
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ndBrainFloat(0.0f);
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
};

class brainLayerTanhActivation : public ndBrainKernel
{
    public:
    brainLayerTanhActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

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

class brainLayerLinearDropOutActivation : public ndBrainKernel
{
    public:
    brainLayerLinearDropOutActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
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

class brainLayerSoftmaxActivation : public ndBrainKernel
{
    public:
    brainLayerSoftmaxActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndFixSizeArray<ndBrainFloat, 1024> reductionBuffer(1024);
        ndFixSizeArray<ndBrainFloat, ND_GPU_LOCAL_BUFFER_SIZE> tmpInputBuffer(ND_GPU_LOCAL_BUFFER_SIZE);

        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        ndAssert(inputSize <= tmpInputBuffer.GetCount());
        
        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndFixSizeArray<ndBrainFloat, 1024> maxArgReg(1024);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            maxArgReg[itemId] = ndBrainFloat(-1.0e30f);
        }

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                tmpInputBuffer[i + itemId] = inputValue;
                maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            tmpInputBuffer[modWorkGroupSize + itemId] = inputValue;
            maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
        }

#ifdef ND_GPU_USE_SOFT_SUBGROUPS
        // using the trick that the width of a compute using is larger of equal than 32 
        // therefere we can about barries.

        // barrier reduction loop
        for (ndInt32 j = workGroupSize / 2; j > ND_GPU_SOFT_SUBGROUPS_WORD_SIZE; j = j >> 1)
        {
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                reductionBuffer[itemId] = maxArgReg[itemId + j];
            }
            // barrier
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
            }
            //barrier
            //j *= 1;
        }

        // no barrier reduction loop
        for (ndInt32 j = ND_GPU_SOFT_SUBGROUPS_WORD_SIZE; j > 0; j = j >> 1)
        {
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                reductionBuffer[itemId] = maxArgReg[itemId + j];
            }
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
            }
        }
#else
        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            //for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                reductionBuffer[itemId] = maxArgReg[itemId + j];
            }

            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
            }
        }
#endif
        reductionBuffer[0] = maxArgReg[0];
        
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            maxArgReg[itemId] = reductionBuffer[0];
        }

        ndFixSizeArray<ndBrainFloat, 1024> sumArgReg(1024);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            sumArgReg[itemId] = ndBrainFloat(0.0f);
        }
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = tmpInputBuffer[i + itemId] - maxArgReg[itemId];
                ndBrainFloat outputValue = exp(inputValue);
                sumArgReg[itemId] += outputValue;
                tmpInputBuffer[i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId] - maxArgReg[itemId];
            ndBrainFloat outputValue = exp(inputValue);
            sumArgReg[itemId] += outputValue;
            tmpInputBuffer[modWorkGroupSize + itemId] = outputValue;
        }
        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
            {
                reductionBuffer[itemId - j] = sumArgReg[itemId];
            }
        
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                sumArgReg[itemId] += inputValue;
            }
        }
        reductionBuffer[0] = ndBrainFloat(1.0f) / sumArgReg[0];
        
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
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId];
            ndBrainFloat outputValue = invDen * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
};

// back propagation kernels
class brainCopyInputGradients : public ndBrainKernel
{
    public:
    brainCopyInputGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const miniBatchGradients = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 dstBase = groupId * ndInt64(inputSize);
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
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

class brainCopyOutputGradients : public ndBrainKernel
{
    public:
    brainCopyOutputGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const miniBatchGradients = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(outputSize);
        ndInt64 dstBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
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

class brainLayerBrainReluBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainReluBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
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

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainTanhBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainTanhBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId  * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);

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

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainCathegoricalSoftmaxBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainCathegoricalSoftmaxBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
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

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainLinearDropOutBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainLinearDropOutBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);

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

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainAdamMomentumUpdate : public ndBrainKernel
{
public:
    brainAdamMomentumUpdate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    //void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    void Execute(ndInt32, ndInt32)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

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

// accumulate kernel gradienst
class brainAccumulateGradientsAndAverage : public ndBrainKernel
{
    public:
    brainAccumulateGradientsAndAverage(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];

        ndBrainFloat* const gradientBuffer = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt64 inputSize = parameters->m_inputSize;
        ndInt32 miniBatchSize = parameters->m_inputOutputSize;
        
        ndInt64 start = groupId * ndInt64(workGroupSize);
        ndFixSizeArray<ndBrainFloat, 1024> accRegister(1024);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            accRegister[itemId] = ndBrainFloat(0.0f);
        }
        for (ndInt32 j = 0; j < miniBatchSize; ++j)
        {
            ndInt64 base = start + j * inputSize;
            ndAssert(base >= 0);
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = gradientBuffer[base + itemId];
                accRegister[itemId] += a;
            }
        }
        // sync here.
        ndBrainFloat weightFactor = ndBrainFloat(1.0f) / ndBrainFloat(miniBatchSize);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndBrainFloat sum = accRegister[itemId];
            gradientBuffer[start + itemId] = sum * weightFactor;
        }
    }
};

class brainAdamUpdateLassoRegularizer : public ndBrainKernel
{
    public:
    brainAdamUpdateLassoRegularizer(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    //void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    void Execute(ndInt32, ndInt32)
    {
        ndAssert(0);
        //ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        //ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        //ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        //ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        //ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        //
        //ndBrainOptimizerAdamGpu::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdamGpu::ndCommandSharedInfo*)buffer0->GetData();
        //ndBrainFloat* const weightAndBiasBuffer = buffer1->GetData();
        //ndBrainFloat* const weightAndBiasGradientBuffer = buffer2->GetData();
        //ndBrainFloat* const vdw = buffer3->GetData();
        //ndBrainFloat* const vdw2 = buffer4->GetData();
        //
        //ndBrainFloat descendRate = -parameters->m_learnRate;
        //ndBrainFloat regularizer = -parameters->m_decayRegularizer;
        //
        //ndInt32 start = groupId * workGroupSize;
        //
        //const ndBrainMemVector vdw__(&vdw[start], workGroupSize);
        //const ndBrainMemVector vdw2__(&vdw[start], workGroupSize);
        //const ndBrainMemVector weight___(&weightAndBiasBuffer[start], workGroupSize);
        //for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        //{
        //    ndBrainFloat temp = weightAndBiasGradientBuffer[start + itemId];
        //    ndBrainFloat a = vdw[start + itemId] * parameters->m_alpha + temp * (ndBrainFloat(1.0f) - parameters->m_alpha);
        //    vdw[start + itemId] = a;
        //
        //    ndBrainFloat b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (ndBrainFloat(1.0f) - parameters->m_beta);
        //    vdw2[start + itemId] = b;
        //
        //    ndBrainFloat vdwCorrected = a * parameters->m_invAlpha;
        //    ndBrainFloat vdw2Corrected = b * parameters->m_invBeta;
        //
        //    ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected)) + parameters->m_epsilon);
        //    ndBrainFloat gradient = vdwCorrected * bias_den;
        //
        //    ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
        //    ndBrainFloat lassoRegularizer = (weight >= ndBrainFloat(0.0f)) ? regularizer : -regularizer;
        //    gradient += weight * lassoRegularizer;
        //    weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
        //}
    }
};

class brainAdamUpdateRidgeRegularizer : public ndBrainKernel
{
    public:
    brainAdamUpdateRidgeRegularizer(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const vdw2 = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const vdw = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const weightAndBiasGradientBuffer = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const weightAndBiasBuffer = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndBrainFloat descendRate = -parameters->m_learnRate;
        ndBrainFloat regularizer = -parameters->m_decayRegularizer;
        
        ndInt64 start = groupId * ndInt64(workGroupSize);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndBrainFloat temp = weightAndBiasGradientBuffer[start + itemId];
            
            // calculate moving average
            ndBrainFloat a = vdw[start + itemId] * parameters->m_alpha + temp * (ndBrainFloat(1.0f) - parameters->m_alpha);
            vdw[start + itemId] = a;
            
            // caluate RMS
            ndBrainFloat b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (ndBrainFloat(1.0f) - parameters->m_beta);
            vdw2[start + itemId] = b;
            
            ndBrainFloat vdwCorrected = a * parameters->m_invAlpha;
            ndBrainFloat vdw2Corrected = b * parameters->m_invBeta;
            
            ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected)) + parameters->m_epsilon);
            ndBrainFloat gradient = vdwCorrected * bias_den;
             
            ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
            gradient += weight * regularizer;
            weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
        }
    }
};

// matrix vector operation kernels.
class brainLayerMatrixMatrixMultiply : public ndBrainKernel
{
    public:
    brainLayerMatrixMatrixMultiply(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    // a matrix time a vector by iterating over each row of the matrix 
    // calculating the dot product of that row time the vector and adding the bias value.
    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        // not need for bank odd row tricks for PC emulation
        ndBrainFloat tile_acc[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];
        ndBrainFloat tile_inputs[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS];
        ndBrainFloat tile_weights[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS];

        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        
        ndBrainFloat* const weightsAndBias = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        const ndInt32 inputSize = parameters->m_inputSize;
        const ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        const ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
        
        const ndInt32 groupId_x = groupId % parameters->m_tiledStride;
        const ndInt32 groupId_y = (groupId - groupId_x) / parameters->m_tiledStride;
        
        //Initialise the accumulation register
        const ndInt64 blockBase = groupId_x * ND_GPU_TILED_MATRIX_ROWS;
        const ndInt64 parametersStartOffset = blockBase * width + parameters->m_parametersStartOffset;
        const ndInt64 parametersBiasOffset = blockBase + width * height + parameters->m_parametersStartOffset;
        ndAssert(parametersBiasOffset >= 0);
        
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
        
        const ndInt32 inputOutputStride = parameters->m_inputOutputSize;
        const ndInt64 inputOffset = groupId_y * ndInt64(inputOutputStride) * ND_GPU_TILED_MATRIX_ROWS + parameters->m_inputOutputStartOffset;
        ndAssert(inputOffset >= 0);
        
        // Loop over all tiles
        for (ndInt32 tile = 0; tile < width; tile += ND_GPU_TILED_MATRIX_COLUMNS)
        {
            // Load one tile of A and B into local memory
            ndInt64 inputStartOffset = tile + inputOffset;
            ndInt64 weightOffsetStart = tile + parametersStartOffset;
            for (ndInt32 itemId_y = 0; itemId_y < ND_GPU_TILED_MATRIX_ROWS; ++itemId_y)
            {
                for (ndInt32 itemId_x = 0; itemId_x < ND_GPU_TILED_MATRIX_COLUMNS; ++itemId_x)
                {
                    tile_inputs[itemId_y][itemId_x] = inputOutputData[inputStartOffset + itemId_x];
                    tile_weights[itemId_y][itemId_x] = weightsAndBias[weightOffsetStart + itemId_x];
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
        ndInt64 outputOffset = groupId_x * ND_GPU_TILED_MATRIX_ROWS + ndInt64(inputOffset) + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);
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


class brainLayerBrainLinearBackPropagateBiasGradients : public ndBrainKernel
{
public:
    brainLayerBrainLinearBackPropagateBiasGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 outputSize = parameters->m_outputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndInt64 parametersStartOffset = ndInt64(parameters->m_parametersStartOffset);

        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);

        // calculate weights and bias gradients
        ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
        ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        ndInt32 matrixSize = width * height;

        ndInt64 weightAndBiasGradientOffset = ndInt64(groupId) * parameters->m_parametersBatchSize + parametersStartOffset;
        ndAssert(weightAndBiasGradientOffset >= 0);

        // calculate bias Gradient
        ndInt32 workGroupOutputSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupOutputSize = outputSize - workGroupOutputSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupOutputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                float a = inputOutputGradients[dstBase + i + itemId];
                weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupOutputSizeReminder; ++itemId)
        {
            float a = inputOutputGradients[dstBase + modWorkGroupOutputSize + itemId];
            weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + modWorkGroupOutputSize + itemId] = a;
        }
        ndBrainMemVector xxx(&weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize], outputSize);
        height *= 1;
    }
};

class brainLayerBrainLinearBackPropagateWeightsOld : public ndBrainKernel
{
    public:
    brainLayerBrainLinearBackPropagateWeightsOld(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndFixSizeArray<ndBrainFloat, ND_GPU_LOCAL_BUFFER_SIZE> cachedInput(ND_GPU_LOCAL_BUFFER_SIZE);

        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const weightAndBias = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt64 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        ndAssert(inputSize <= cachedInput.GetCount());
        
        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = ndBrainFloat(0.0f);
            }
        }
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndInt64 parametersStartOffset = ndInt64(parameters->m_parametersStartOffset);

        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        ndAssert(parametersStartOffset >= 0);
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        
        // calculate input gradients
        ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt64 weightOffset = j * width + parametersStartOffset;
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

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif

        // calculate weights and bias gradients
        ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        ndInt32 matrixSize = width * height;

        ndInt64 weightAndBiasGradientOffset = ndInt64(groupId) * parameters->m_parametersBatchSize + parametersStartOffset;
        ndAssert(weightAndBiasGradientOffset >= 0);
        
        // calculate matrix weight Gradient
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt64 weightRowOffset = weightAndBiasGradientOffset + j * width;
        
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

        #ifdef _DEBUG
        {
            ndInt32 padded = (matrixSize + outputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = matrixSize + outputSize; i < padded; ++i)
            {
                ndBrainFloat a = weightAndBiasGradients[weightAndBiasGradientOffset + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainLinearBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainLinearBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    //void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    void Execute(ndInt32, ndInt32)
    {
#if 0
        //ndFixSizeArray<ndBrainFloat, ND_GPU_LOCAL_BUFFER_SIZE> cachedInput(ND_GPU_LOCAL_BUFFER_SIZE);
        ndBrainFloat tile_acc[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];
        ndBrainFloat tile_inputs[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS];
        ndBrainFloat tile_weights[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_COLUMNS];

        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const weightAndBias = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 outputSize = ndInt32(parameters->m_outputSize);
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt64 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        //ndAssert(inputSize <= cachedInput.GetCount());

        for (ndInt32 i = 0; i < inputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedInput[i + itemId] = ndBrainFloat(0.0f);
            }
        }

        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndInt64 parametersStartOffset = ndInt64(parameters->m_parametersStartOffset);

        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        ndAssert(parametersStartOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        // calculate input gradients
        ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt64 weightOffset = j * width + parametersStartOffset;
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

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif

        // calculate weights and bias gradients
        ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        ndInt32 matrixSize = width * height;

        ndInt64 weightAndBiasGradientOffset = ndInt64(groupId) * parameters->m_parametersBatchSize + parametersStartOffset;
        ndAssert(weightAndBiasGradientOffset >= 0);

        // calculate bias Gradient
        ndInt32 workGroupOutputSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupOutputSize = outputSize - workGroupOutputSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupOutputSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                float a = inputOutputGradients[dstBase + i + itemId];
                weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupOutputSizeReminder; ++itemId)
        {
            float a = inputOutputGradients[dstBase + modWorkGroupOutputSize + itemId];
            weightAndBiasGradients[weightAndBiasGradientOffset + matrixSize + modWorkGroupOutputSize + itemId] = a;
        }

        // calculate matrix weight Gradient
        for (ndInt32 j = 0; j < outputSize; ++j)
        {
            ndBrainFloat scale = inputOutputGradients[dstBase + j];
            ndInt64 weightRowOffset = weightAndBiasGradientOffset + j * width;

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

        #ifdef _DEBUG
        {
            ndInt32 padded = (matrixSize + outputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = matrixSize + outputSize; i < padded; ++i)
            {
                ndBrainFloat a = weightAndBiasGradients[weightAndBiasGradientOffset + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
#endif
    }
};

class brainCopyBuffer : public ndBrainKernel
{
    public:
    brainCopyBuffer(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        ndUnsigned32* const srcBuffer = (ndUnsigned32*)buffer2->GetGpuBuffer()->GetPtr();
        ndUnsigned32* const dstBuffer = (ndUnsigned32*)buffer1->GetGpuBuffer()->GetPtr();
        ndCopyBufferCommandInfo* const parameters = (ndCopyBufferCommandInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndAssert((parameters->m_strideInByte & (sizeof(ndInt32) - 1)) == 0);

        ndInt32 stride = parameters->m_strideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 srcStride = parameters->m_srcStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 srcOffset = parameters->m_srcOffsetInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstStride = parameters->m_dstStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstOffset = parameters->m_dstOffsetInByte / ndInt32(sizeof(ndInt32));

        ndInt32 workGroupSizeReminder = stride % workGroupSize;
        ndInt32 modWorkGroupSize = stride - workGroupSizeReminder;

        ndInt64 dstBase = ndInt64(dstOffset) + groupId * ndInt64(dstStride);
        ndInt64 srcBase = ndInt64(srcOffset) + groupId * ndInt64(srcStride);
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndUnsigned32 val = srcBuffer[srcBase + i + itemId];
                dstBuffer[dstBase + i + itemId] = val;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndUnsigned32 val = srcBuffer[srcBase + modWorkGroupSize + itemId];
            dstBuffer[dstBase + modWorkGroupSize + itemId] = val;
        }
    }
};

class brainCopyBufferIndirect : public ndBrainKernel
{
    public:
    brainCopyBufferIndirect(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainIntegerBuffer* const buffer3 = (ndBrainIntegerBuffer*)m_parameters[3];

        ndUnsigned32* indexArray = (ndUnsigned32*)buffer3->GetGpuBuffer()->GetPtr();
        ndUnsigned32* const srcBuffer = (ndUnsigned32*)buffer2->GetGpuBuffer()->GetPtr();
        ndUnsigned32* const dstBuffer = (ndUnsigned32*)buffer1->GetGpuBuffer()->GetPtr();
        ndCopyBufferCommandInfo* const parameters = (ndCopyBufferCommandInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndAssert((parameters->m_strideInByte & (sizeof(ndInt32) - 1)) == 0);

        ndInt32 stride = parameters->m_strideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 srcStride = parameters->m_srcStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 srcOffset = parameters->m_srcOffsetInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstStride = parameters->m_dstStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstOffset = parameters->m_dstOffsetInByte / ndInt32(sizeof(ndInt32));

        ndInt32 workGroupSizeReminder = stride % workGroupSize;
        ndInt32 modWorkGroupSize = stride - workGroupSizeReminder;

        ndInt64 dstBase = ndInt64(dstOffset) + groupId * ndInt64(dstStride);
        ndInt64 srcBase = ndInt64(srcOffset) + indexArray[groupId] * ndInt64(srcStride);
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndUnsigned32 val = srcBuffer[srcBase + i + itemId];
                dstBuffer[dstBase + i + itemId] = val;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndUnsigned32 val = srcBuffer[srcBase + modWorkGroupSize + itemId];
            dstBuffer[dstBase + modWorkGroupSize + itemId] = val;
        }
    }
};

void ndBrainGpuContext::CreateKerners()
{   
    // create all feed foward shaders
    m_brainCopyInput = ndSharedPtr<ndBrainKernel> (new brainCopyInput(this));
    m_brainCopyOutput = ndSharedPtr<ndBrainKernel> (new brainCopyOutput(this));
    m_brainLayerReluActivation = ndSharedPtr<ndBrainKernel>(new brainLayerReluActivation(this));
    m_brainLayerTanhActivation = ndSharedPtr<ndBrainKernel>(new brainLayerTanhActivation(this));
    m_brainLayerSoftmaxActivation = ndSharedPtr<ndBrainKernel>(new brainLayerSoftmaxActivation(this));
    m_brainLayerDropOutActivation = ndSharedPtr<ndBrainKernel>(new brainLayerLinearDropOutActivation(this));
    m_brainLayerMatrixMatrixMultiply = ndSharedPtr<ndBrainKernel>(new brainLayerMatrixMatrixMultiply(this));

    // create all backpropagate shaders
    m_brainCopyInputGradients = ndSharedPtr<ndBrainKernel>(new brainCopyInputGradients(this));
    m_brainCopyOutputGradients = ndSharedPtr<ndBrainKernel>(new brainCopyOutputGradients(this));
    m_brainLayerReluBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainReluBackPropagate(this));
    m_brainLayerTanhBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainTanhBackPropagate(this));
    m_brainLayerDropOutBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLinearDropOutBackPropagate(this));
    m_brainLayerCathegoricalSoftmaxBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainCathegoricalSoftmaxBackPropagate(this));
    m_brainLayerMatrixBackPropagateWeightGradeints = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLinearBackPropagateWeightsOld(this));
    m_brainLayerMatrixBackPropagateBiasGradients = ndSharedPtr<ndBrainKernel>(new   brainLayerBrainLinearBackPropagateBiasGradients(this));

    // optimizer kernels
    m_brainAdamMomentumUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamMomentumUpdate(this));
    m_brainAdamRidgeOptimizerUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamUpdateRidgeRegularizer(this));
    m_brainAdamLassoOptimizerUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamUpdateLassoRegularizer(this));
    m_brainAccumulateGradientsAndAverage = ndSharedPtr<ndBrainKernel>(new brainAccumulateGradientsAndAverage(this));

    // optimizer kernels
    m_brainCopyBuffer = ndSharedPtr<ndBrainKernel>(new brainCopyBuffer(this));
    m_brainCopyBufferIndirect = ndSharedPtr<ndBrainKernel>(new brainCopyBufferIndirect(this));
}