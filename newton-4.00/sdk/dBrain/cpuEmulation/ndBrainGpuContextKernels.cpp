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

#define ND_GPU_LOCAL_BUFFER_SIZE	    1024 * 4

#define ND_GPU_LEAKY_LRU_GRADIENT		ndBrainFloat(0.01f)

inline ndInt32 __cpuKernelRoundoff(ndInt32 value, ndInt32 workgroupSize)
{
    return (value + workgroupSize - 1) & -workgroupSize;
}

inline ndInt32 __twoPower(ndInt32 x)
{
    ndAssert(x <= 1 << 16);
    ndAssert(((x - 1) & -x) == 0);
    ndInt32 bits = 1;
    bits += (0xff00 & x) & 8;
    bits += (0xf0f0 & x) & 4;
    bits += (0xcccc & x) & 2;
    bits += (0x8888 & x) & 2;
    return bits;
}

inline bool ndCheckValidFloat(ndFloat32 x)
{
    x = 0;
    //bool test = _finite(x) && !_isnan(x);
    //return test;
    return true;
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
        
        ndInt32 outputSize = parameters->m_outputSize;
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

class brainLayerLeakyReluActivation : public ndBrainKernel
{
    public:
    brainLayerLeakyReluActivation(ndBrainContext* const context)
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
                ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ND_GPU_LEAKY_LRU_GRADIENT * inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ND_GPU_LEAKY_LRU_GRADIENT * inputValue;
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

class brainLayerLinearActivation : public ndBrainKernel
{
    public:
    brainLayerLinearActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainUniformBuffer* const buffer3 = (ndBrainUniformBuffer*)m_parameters[3];
        ndBrainUniformBuffer* const buffer4 = (ndBrainUniformBuffer*)m_parameters[4];

        ndBrainFloat* const biasPtr = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const slopesPtr = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();

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
                float  bias = biasPtr[i + itemId];
                float  slope = slopesPtr[i + itemId];
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = bias + slope * inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            float  bias = biasPtr[modWorkGroupSize + itemId];
            float  slope = slopesPtr[modWorkGroupSize + itemId];
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = bias + slope * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
};

class brainLayerDeterministicPolicyActivation : public ndBrainKernel
{
    public:
    brainLayerDeterministicPolicyActivation(ndBrainContext* const context)
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

        ndInt32 halfSize = inputSize / 2;
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat blend1 = ((i + itemId) >= halfSize) ? 1.0f : 0.0f;
                ndBrainFloat blend0 = 1.0f - blend1;

                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat expenential = ndExp_VSFix(inputValue);

                ndBrainFloat outputValue = inputValue * blend0 + expenential * blend1;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat blend1 = ((modWorkGroupSize + itemId) >= halfSize) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
            ndBrainFloat blend0 = ndBrainFloat(1.0f) - blend1;

            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat expenential = ndExp_VSFix(inputValue);

            ndBrainFloat outputValue = inputValue * blend0 + expenential * blend1;
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

        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
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
            // barrier
        }
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
                //ndBrainFloat outputValue = exp(inputValue);
                ndBrainFloat outputValue = ndExp_VSFix(inputValue);
                sumArgReg[itemId] += outputValue;
                tmpInputBuffer[i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId] - maxArgReg[itemId];
            //ndBrainFloat outputValue = exp(inputValue);
            ndBrainFloat outputValue = ndExp_VSFix(inputValue);
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
        
        ndInt32 outputSize = parameters->m_outputSize;
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

class brainLayerBrainLeakyReluBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainLeakyReluBackPropagate(ndBrainContext* const context)
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
                ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ND_GPU_LEAKY_LRU_GRADIENT;
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
            }
        }

        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
            ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ND_GPU_LEAKY_LRU_GRADIENT;
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

class brainLayerBrainLinearBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainLinearBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainUniformBuffer* const buffer5 = (ndBrainUniformBuffer*)m_parameters[5];

        ndBrainFloat* const slopesPtr = (ndBrainFloat*)buffer5->GetGpuBuffer()->GetPtr();
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
                ndBrainFloat slope = slopesPtr[i + itemId];
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = slope * outputGrad;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat slope = slopesPtr[modWorkGroupSize + itemId];
            ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = slope * outputGrad;
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

class brainLayerBrainDeterministicPolicyBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainDeterministicPolicyBackPropagate(ndBrainContext* const context)
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

        ndInt32 halfSize = inputSize / 2;
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat blend1 = ((i + itemId) >= halfSize) ? 1.0f : 0.0f;
                ndBrainFloat blend0 = 1.0f - blend1;

                ndBrainFloat gradient = ndBrainFloat(1.0);
                ndBrainFloat outputData = inputOutputData[dstBase + i + itemId];
                ndBrainFloat gradSelect = gradient * blend0 + outputData * blend1;
                ndBrainFloat inputGradient = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = gradSelect * inputGradient;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat blend1 = ((modWorkGroupSize + itemId) >= halfSize) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
            ndBrainFloat blend0 = ndBrainFloat(1.0f) - blend1;

            ndBrainFloat gradient = ndBrainFloat(1.0);
            ndBrainFloat outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
            ndBrainFloat gradSelect = gradient * blend0 + outputData * blend1;
            ndBrainFloat inputGradient = inputOutputGradients[dstBase + modWorkGroupSize + itemId];

            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradSelect * inputGradient;
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
        ndBrainFloat miniBatchWeight = parameters->m_minibathScale;
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndBrainFloat weightAndBiasGradient = miniBatchWeight * weightAndBiasGradientBuffer[start + itemId];
            
            // calculate moving average
            ndBrainFloat a = vdw[start + itemId] * parameters->m_alpha + weightAndBiasGradient * (ndBrainFloat(1.0f) - parameters->m_alpha);
            vdw[start + itemId] = a;
            
            // caluate RMS
            ndBrainFloat b = vdw2[start + itemId] * parameters->m_beta + weightAndBiasGradient * weightAndBiasGradient * (ndBrainFloat(1.0f) - parameters->m_beta);
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

    // tile based matrix x matrix multiplication.
    // partion that matrices A into a (M x K) and matrix B into a (K x N) blocks 
    // them doing the block matrix multiplication. 
    // if tune propertlly, each block can be load into shared memory or lavel ine cache 
    // then do a sub tileA by TileB matrix multiplication in cache memory.
    // this is the trick that specialized, so call AI instruction do in hardware,
    // inpement in speciallized instartion to speed up the opertioan 100s of times, 
    // Unfortunatly OpenCl and my intel cpu do not exposes these instrutions to the user, 
    // maybe one day. for now this will do using avx and cumpute units.
    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        // not need for bank odd row tricks for PC emulation
        const ndInt32 tileSize = ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 tileSizeBits = ND_GPU_TILED_MATRIX_ROWS_BITS;

        ndBrainFloat tile_inputs[tileSize * 2][tileSize];
        ndBrainFloat tile_weights[tileSize * 2][tileSize];

        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const weightsAndBias = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        const ndInt32 inputSize = parameters->m_inputSize;
        const ndInt32 outputSize = parameters->m_outputSize;
        const ndInt32 width = (inputSize + tileSize * 2 - 1) & -tileSize * 2;

        const ndInt32 minibatchBlock = (outputSize + tileSize - 1) >> tileSizeBits;
        const ndInt32 groupId_y = groupId / minibatchBlock;
        const ndInt32 groupId_x = groupId - groupId_y * minibatchBlock;

        //Initialise the accumulation register
        const ndInt64 parameterBlockBase = groupId_x * tileSize;
        const ndInt64 parametersStartOffset = parameterBlockBase * width + parameters->m_parametersStartOffset;

        // load matrix bias to local memory
        const ndInt32 height = (outputSize + tileSize - 1) & -tileSize;
        const ndInt64 parametersBiasOffset = parameterBlockBase + width * height + parameters->m_parametersStartOffset;
        ndAssert(parametersBiasOffset >= 0);
        for (ndInt32 itemId = 0; itemId < tileSize; ++itemId)
        {
            tile_inputs[0][itemId] = weightsAndBias[parametersBiasOffset + itemId];
        }
        // barries?

        ndBrainFloat tile_accReg[tileSize][tileSize];
        for (ndInt32 itemId_y = 0; itemId_y < tileSize; ++itemId_y)
        {
            for (ndInt32 itemId_x = 0; itemId_x < tileSize; ++itemId_x)
            {
                tile_accReg[itemId_y][itemId_x] = ndBrainFloat(0.0f);
            }
        }

        const ndInt32 inputOutputStride = parameters->m_inputOutputSize;
        const ndInt64 inputOffset = groupId_y * ndInt64(inputOutputStride) * tileSize + parameters->m_inputOutputStartOffset;
        ndAssert(inputOffset >= 0);

        // load matrix bias to registers and transpose it
        for (ndInt32 itemId = 0; itemId < tileSize; ++itemId)
        {
            tile_inputs[0][itemId] = weightsAndBias[parametersBiasOffset + itemId];
        }
        ndBrainFloat biasValueReg[1024];
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            //ndInt32 itemId_x = itemId & (tileSize - 1);
            ndInt32 itemId_y = itemId >> tileSizeBits;
            ndAssert(itemId_x < (itemId & (tileSize - 1)));
            ndAssert(itemId_y < (itemId >> tileSizeBits));
            biasValueReg[itemId] = tile_inputs[0][itemId_y];
        }

        // Loop over all tiles
        ndInt32 halfTileStart = tileSize / 2;
        for (ndInt32 tile = 0; tile < width; tile += tileSize * 2)
        {
            // read the transpose of the tiles (GPU style, but too slow for CPU)
            ndInt64 inputStartOffset = tile + inputOffset;
            ndInt64 weightOffsetStart = tile + parametersStartOffset;
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndInt32 itemId_x = itemId & (tileSize * 2 - 1);
                ndInt32 itemId_y = itemId >> (tileSizeBits + 1);
                ndAssert(itemId_y < tileSize / 2);
                ndAssert(itemId_x < tileSize * 2);

                ndBrainFloat weight0 = weightsAndBias[weightOffsetStart + itemId_y * width + itemId_x];
                ndBrainFloat inputData0 = inputOutputData[inputStartOffset + itemId_y * inputOutputStride + itemId_x];
                tile_weights[itemId_x][itemId_y] = weight0;
                tile_inputs[itemId_x][itemId_y] = inputData0;

                ndBrainFloat weight1 = weightsAndBias[weightOffsetStart + (itemId_y + halfTileStart) * width + itemId_x];
                ndBrainFloat inputData1 = inputOutputData[inputStartOffset + (itemId_y + halfTileStart) * inputOutputStride + itemId_x];
                tile_weights[itemId_x][itemId_y + halfTileStart] = weight1;
                tile_inputs[itemId_x][itemId_y + halfTileStart] = inputData1;
            }

            // Perform the computation for a single tile
            for (ndInt32 i = 0; i < tileSize * 2; ++i)
            {
                for (ndInt32 itemId_y = 0; itemId_y < tileSize; itemId_y++)
                {
                    ndBrainFloat a = tile_weights[i][itemId_y];
                    for (ndInt32 itemId_x = 0; itemId_x < tileSize; itemId_x++)
                    {
                        tile_accReg[itemId_y][itemId_x] += a * tile_inputs[i][itemId_x];
                    }
                }
            }
            // barrier
        }
        // barrier ?

        // add matrix bias, and transposed the results
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndInt32 itemId_x = itemId & (tileSize - 1);
            ndInt32 itemId_y = itemId >> tileSizeBits;
            ndAssert(itemId_x < tileSize);
            ndAssert(itemId_y < tileSize);
            tile_inputs[itemId_x][itemId_y] = tile_accReg[itemId_y][itemId_x] + biasValueReg[itemId];
        }
        // barrier

        const ndInt32 numberOutput = ((groupId_x + 1) * tileSize < outputSize) ? tileSize : outputSize - groupId_x * tileSize;
        ndInt64 outputOffset = groupId_x * tileSize + ndInt64(inputOffset) + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndInt32 itemId_x = itemId & (tileSize - 1);
            ndInt32 itemId_y = itemId >> tileSizeBits;
            ndAssert(itemId_x < tileSize);
            ndAssert(itemId_y < tileSize);
            ndBrainFloat value = tile_inputs[itemId_y][itemId_x];
            if (itemId_x < numberOutput)
            {
                inputOutputData[outputOffset + itemId_y * inputOutputStride + itemId_x] = value;
            }
        }
    }
};

class brainLayerBrainBackPropagateMatrixInputGradients : public ndBrainKernel
{
    public:
    brainLayerBrainBackPropagateMatrixInputGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        // not need for bank odd row tricks for PC emulation
        const ndInt32 tileSize = ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 tileSizeBits = ND_GPU_TILED_MATRIX_ROWS_BITS;

        ndBrainFloat tile_weights[tileSize][tileSize];
        ndBrainFloat tile_outputGradients[tileSize][tileSize];

        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const weightAndBias = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndAssert((parameters->m_matrixDimensionK) >= (ND_GPU_TILED_MATRIX_ROWS_BITS + 1));

        ndBrainFloat tile_accReg[tileSize][tileSize];
        for (ndInt32 itemId_y = 0; itemId_y < tileSize; ++itemId_y)
        {
            for (ndInt32 itemId_x = 0; itemId_x < tileSize; ++itemId_x)
            {
                tile_accReg[itemId_y][itemId_x] = ndBrainFloat(0.0f);
            }
        }

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 outputSize = parameters->m_outputSize;
        ndInt64 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        const ndInt32 minibatchBlock = parameters->m_matrixDimensionK >> tileSizeBits;
        const ndInt32 groupId_y = groupId / minibatchBlock;
        const ndInt32 groupId_x = groupId - groupId_y * minibatchBlock;

        const ndInt32 inputOutputStride = parameters->m_inputOutputSize;
        const ndInt32 width = (inputSize + (tileSize * 2) - 1) & -(tileSize * 2);
        const ndInt64 inputOffset = groupId_x * ndInt64(tileSize) * inputOutputStride + inputOutputStartOffset;
        const ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        const ndInt64 parametersStartOffset = groupId_y * ndInt64(tileSize) + parameters->m_parametersStartOffset;

        // Loop over all tiles
        const ndInt32 dimensionK = ((outputSize + tileSize - 1) & -tileSize);
        for (ndInt32 tile = 0; tile < dimensionK; tile += tileSize)
        {
            // Load one transposed tile A and B into local memory (cpu style)
            ndInt64 outputStartOffset = tile + outputOffset;
            ndInt64 weightOffsetStart = tile * width + parametersStartOffset;
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            { 
                ndInt32 itemId_x = itemId & (tileSize - 1);
                ndInt32 itemId_y = itemId >> tileSizeBits;
                ndAssert(itemId_y < tileSize);
                ndAssert(itemId_x < tileSize);

                ndBrainFloat weight0 = weightAndBias[weightOffsetStart + itemId_y * width + itemId_x];
                ndBrainFloat outputGradient0 = inputOutputGradients[outputStartOffset + itemId_y * inputOutputStride + itemId_x];
                tile_weights[itemId_y][itemId_x] = weight0;
                tile_outputGradients[itemId_x][itemId_y] = outputGradient0;
            }
            // barrier

            // Perform the computation for a single tile
            // this loop can be unrolled and get faster but the compiler fail to do it,
            for (ndInt32 i = 0; i < tileSize; ++i)
            {
                for (ndInt32 itemId_y = 0; itemId_y < tileSize; itemId_y++)
                {
                    ndBrainFloat outGradient = tile_outputGradients[i][itemId_y];
                    for (ndInt32 itemId_x = 0; itemId_x < tileSize; itemId_x++)
                    {
                        ndBrainFloat weight = tile_weights[i][itemId_x];
                        tile_accReg[itemId_y][itemId_x] += weight * outGradient;
                    }
                }
            }
            // barrier
        }

        ndInt64 dstOffset = inputOffset + groupId_y * tileSize; 
        for (ndInt32 itemId_y = 0; itemId_y < tileSize; ++itemId_y)
        {
            for (ndInt32 itemId_x = 0; itemId_x < tileSize; ++itemId_x)
            {
                ndBrainFloat value = tile_accReg[itemId_y][itemId_x];
                inputOutputGradients[dstOffset + itemId_x] = value;
            }
            dstOffset += inputOutputStride;
        }
    }
};

class brainLayerBrainBackPropagateMatrixClearBiasGradients : public ndBrainKernel
{
    public:
        brainLayerBrainBackPropagateMatrixClearBiasGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        
        ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 outputSize = parameters->m_outputSize;
        ndInt32 inpuOutputStride = parameters->m_inputOutputSize;

        //ndInt32 alignedOffset = (outputSize + 255) & -256;
        ndInt32 alignedOffset = (outputSize + workGroupSize - 1) & -workGroupSize;
        const ndInt32 dstOffset = groupId * alignedOffset;
        const ndInt32 srcOffset = parameters->m_inputOutputStartOffset + inputSize + groupId * inpuOutputStride;

        const ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        const ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat outputDerivative = inputOutputGradientsBuffer[srcOffset + i + itemId];
                partialBiasSumBuffer[dstOffset + i + itemId] = outputDerivative;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat outputDerivative = inputOutputGradientsBuffer[srcOffset + modWorkGroupSize + itemId];
            partialBiasSumBuffer[dstOffset + modWorkGroupSize + itemId] = outputDerivative;
        }
    }
};

class brainLayerBrainBackPropagateMatrixPartialSumBiasGradients : public ndBrainKernel
{
    public:
    brainLayerBrainBackPropagateMatrixPartialSumBiasGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        const ndInt32 numberOfIndex = parameters->m_matrixDimensionK;
        const ndInt32 srcIndex = groupId + (numberOfIndex + 1) /2;
        if (srcIndex < numberOfIndex)
        {
            ndInt32 outputSize = parameters->m_outputSize;
            ndInt32 alignedOffset = (outputSize + workGroupSize - 1) & -workGroupSize;

            ndInt32 dstOffset = groupId * alignedOffset;
            ndInt32 srcOffset = srcIndex * alignedOffset;

            const ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
            const ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;
            for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
            {
                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    ndBrainFloat biasGradient = partialBiasSumBuffer[srcOffset + i + itemId];
                    partialBiasSumBuffer[dstOffset + i + itemId] += biasGradient;
                }
            }
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                ndBrainFloat biasGradient = partialBiasSumBuffer[srcOffset + modWorkGroupSize + itemId];
                partialBiasSumBuffer[dstOffset + modWorkGroupSize + itemId] += biasGradient;
            }
        }
    }
};

class brainLayerBrainBackPropagateMatrixBiasGradients : public ndBrainKernel
{
    public:
    brainLayerBrainBackPropagateMatrixBiasGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    //#pragma optimize( "", off )
    void Execute(ndInt32, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 outputSize = parameters->m_outputSize;
        ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_ROWS * 2 - 1) & -ND_GPU_TILED_MATRIX_ROWS * 2;
        ndInt32 matrixSize = __cpuKernelRoundoff(width * height, ND_DEFAULT_WORKGROUP_SIZE);;
        
        ndInt64 parametersStartOffset = ndInt64(parameters->m_parametersStartOffset) + matrixSize;

        const ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        const ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;
        for (ndInt32 rowBlock = 0; rowBlock < modWorkGroupSize; rowBlock += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat biasDerivative = partialBiasSumBuffer[rowBlock + itemId];
                //ndAssert(ndCheckValidFloat(biasDerivative));
                weightAndBiasGradients[parametersStartOffset + rowBlock + itemId] = biasDerivative;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat biasDerivative = partialBiasSumBuffer[modWorkGroupSize + itemId];
            //ndAssert(ndCheckValidFloat(biasDerivative));
            weightAndBiasGradients[parametersStartOffset + modWorkGroupSize + itemId] = biasDerivative;
        }
    }
};

class brainLayerBrainBackPropagateMatrixWeightsGradients : public ndBrainKernel
{
    public:
    brainLayerBrainBackPropagateMatrixWeightsGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndFixSizeArray<ndBrainFloat, 1024> cachedRowGradient(1024);
        ndFixSizeArray<ndBrainFloat, ND_GPU_LOCAL_BUFFER_SIZE> cachedOutputGradients(ND_GPU_LOCAL_BUFFER_SIZE);
        ndAssert(workGroupSize <= cachedRowGradient.GetCount());

        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 numberOfRows = parameters->m_matrixDimensionK;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;

        ndInt64 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        ndInt64 dstBase = inputOutputStartOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        for (ndInt32 i = 0; i < numberOfRows; i += workGroupSize)
        {
            ndInt64 baseOffset = dstBase + i * inputOutputSize;
            ndInt32 count = ((i + workGroupSize) < numberOfRows) ? workGroupSize : numberOfRows - i;
            for (ndInt32 itemId = 0; itemId < count; ++itemId)
            {
                ndBrainFloat outputDerivative = inputOutputGradients[baseOffset + itemId * inputOutputSize + groupId];
                cachedOutputGradients[i + itemId] = outputDerivative;
            }
        }

        const ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        const ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        // barrier

        ndInt64 parametersStartOffset = ndInt64(parameters->m_parametersStartOffset);
        ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_ROWS * 2 - 1) & -ND_GPU_TILED_MATRIX_ROWS * 2;
        for (ndInt32 rowBlock = 0; rowBlock < modWorkGroupSize; rowBlock += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                cachedRowGradient[itemId] = ndBrainFloat(0.0f);
            }
            ndInt64 inputOffsetBase = inputOutputStartOffset + rowBlock;
            for (ndInt32 row = 0; row < numberOfRows; ++row)
            {
                ndInt64 inputOffset = inputOffsetBase + row * inputOutputSize;
                ndBrainFloat outputDerivative = cachedOutputGradients[row];

                for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
                {
                    ndBrainFloat inputValue = inputOutputData[inputOffset + itemId];
                    cachedRowGradient[itemId] += outputDerivative * inputValue;
                }
            }
            // store this weight gradient sum
            ndInt64 parametersOffset = parametersStartOffset + rowBlock + width * groupId;
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                weightAndBiasGradients[parametersOffset + itemId] = cachedRowGradient[itemId];
            }
        }

        if (workGroupSizeReminder != 0)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                cachedRowGradient[itemId] = ndBrainFloat(0.0f);
            }
            ndInt64 inputOffsetBase = inputOutputStartOffset + modWorkGroupSize;
            for (ndInt32 row = 0; row < numberOfRows; ++row)
            {
                ndInt64 inputOffset = inputOffsetBase + row * inputOutputSize;
                ndBrainFloat outputDerivative = cachedOutputGradients[row];

                for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
                {
                    ndBrainFloat inputValue = inputOutputData[inputOffset + itemId];
                    cachedRowGradient[itemId] += outputDerivative * inputValue;
                }
            }
            // store this weight gradient sum
            ndInt64 parametersOffset = parametersStartOffset + modWorkGroupSize + width * groupId;
            for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
            {
                ndBrainFloat weightGradeint = cachedRowGradient[itemId];
                weightAndBiasGradients[parametersOffset + itemId] = weightGradeint;
            }
        }
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
    m_brainLayerLinearActivation = ndSharedPtr<ndBrainKernel>(new brainLayerLinearActivation(this));
    m_brainLayerSoftmaxActivation = ndSharedPtr<ndBrainKernel>(new brainLayerSoftmaxActivation(this));
    m_brainLayerDropOutActivation = ndSharedPtr<ndBrainKernel>(new brainLayerLinearDropOutActivation(this));
    m_brainLayerLeakyReluActivation = ndSharedPtr<ndBrainKernel>(new brainLayerLeakyReluActivation(this));
    m_brainLayerMatrixMatrixMultiply = ndSharedPtr<ndBrainKernel>(new brainLayerMatrixMatrixMultiply(this));
    m_brainLayerDeterministicPolicyActivation = ndSharedPtr<ndBrainKernel>(new brainLayerDeterministicPolicyActivation(this));

    // create all backpropagate shaders
    m_brainCopyInputGradients = ndSharedPtr<ndBrainKernel>(new brainCopyInputGradients(this));
    m_brainCopyOutputGradients = ndSharedPtr<ndBrainKernel>(new brainCopyOutputGradients(this));
    m_brainLayerLinearPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLinearBackPropagate(this));
    m_brainLayerReluBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainReluBackPropagate(this));
    m_brainLayerTanhBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainTanhBackPropagate(this));
    m_brainLayerLeakyReluBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLeakyReluBackPropagate(this));
    m_brainLayerDropOutBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLinearDropOutBackPropagate(this));
    m_brainLayerDeterministicPolicyBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainDeterministicPolicyBackPropagate(this));
    m_brainLayerCathegoricalSoftmaxBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainCathegoricalSoftmaxBackPropagate(this));
    m_brainLayerMatrixBackPropagateBiasGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixBiasGradients(this));
    m_brainLayerMatrixBackPropagateInputGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixInputGradients(this));
    m_brainLayerMatrixBackPropagateWeightGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixWeightsGradients(this));
    m_brainLayerMatrixBackPropagateClearBiasGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixClearBiasGradients(this));
    m_brainLayerMatrixBackPropagateAddBiasGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixPartialSumBiasGradients(this));

    // optimizer kernels
    m_brainAdamMomentumUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamMomentumUpdate(this));
    m_brainAdamRidgeOptimizerUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamUpdateRidgeRegularizer(this));
    m_brainAdamLassoOptimizerUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamUpdateLassoRegularizer(this));

    // optimizer kernels
    m_brainCopyBuffer = ndSharedPtr<ndBrainKernel>(new brainCopyBuffer(this));
    m_brainCopyBufferIndirect = ndSharedPtr<ndBrainKernel>(new brainCopyBufferIndirect(this));
}