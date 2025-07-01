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
#ifndef __ND_BRAIN_BUFFER_COMMAND_H__
#define __ND_BRAIN_BUFFER_COMMAND_H__

#include "ndBrainStdafx.h"

class ndBrainLayer;
class ndBrainKernel;
class ndBrainBuffer;
class ndBrainContext;
class ndBrainUniformBuffer;
class ndBrainTrainerInference;

class ndCommandSharedInfo
{
	public:
	ndCommandSharedInfo()
		:m_layer(nullptr)
		,m_parametersStartOffset(0)
		,m_inputOutputStartOffset(0)
		,m_inputSize(0)
		,m_outputSize(0)
		,m_inputOutputSize(0)
		,m_parametersBatchSize(0)
		,m_tiledStride(0)
	{
	}

	ndCommandSharedInfo(const ndBrainLayer* const layer)
		:m_layer(layer)
		,m_parametersStartOffset(0)
		,m_inputOutputStartOffset(0)
		,m_inputSize(0)
		,m_outputSize(0)
		,m_inputOutputSize(0)
		,m_parametersBatchSize(0)
		,m_tiledStride(0)
	{
	}

	const ndBrainLayer* m_layer;
	ndInt32 m_parametersStartOffset;
	ndInt32 m_inputOutputStartOffset;
	ndInt32 m_inputSize;
	ndInt32 m_outputSize;
	ndInt32 m_inputOutputSize;
	ndInt32 m_parametersBatchSize;
	ndInt32	m_tiledStride;
};

class ndBrainBufferCommandDesc : public ndFixSizeArray<ndBrainBuffer*, 16>
{
	public:
	ndBrainBufferCommandDesc(ndInt32 minibatchSize);

	ndBrainContext* m_context;
	ndBrainTrainerInference* m_owner;
	ndCommandSharedInfo m_info;
	ndSharedPtr<ndBrainKernel> m_kernel;
	ndSharedPtr<ndBrainUniformBuffer> m_uniformBuffer;
	size_t m_id;
	ndInt32 m_miniBatchSize;
	ndInt32 m_workGroupSize;
};

class ndBrainBufferCommand : public ndContainersFreeListAlloc<ndBrainBufferCommand>
{
	public:
	ndBrainBufferCommand(const ndBrainBufferCommandDesc& desc);
	virtual ~ndBrainBufferCommand();
	
	ndBrainBufferCommandDesc& GetDescriptor();
	const ndBrainBufferCommandDesc& GetDescriptor() const;

	protected:
	ndBrainBufferCommandDesc m_desc;
};

class ndBrainBufferCommandCpu : public ndBrainBufferCommand
{
	public:
	ndBrainBufferCommandCpu(const ndBrainBufferCommandDesc& desc);
	virtual ~ndBrainBufferCommandCpu();

	virtual void Execute(ndInt32 miniBatchIndex) = 0;
};

#endif