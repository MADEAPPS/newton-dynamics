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
class ndBrainBuffer;
class ndBrainContext;
class ndBrainUniformBuffer;
class ndBrainTrainerInference;

class ndCommandShareInfo
{
	public:
	ndCommandShareInfo()
		:m_inputSize(0)
		,m_outputSize(0)
		,m_inputOutputSize(0)
		,m_inputOutputStartOffset(0)
		,m_parametersBatchSize(0)
		,m_parametersStartOffset(0)
		,m_tiledStride(0)
		,m_layer(nullptr)
	{
	}

	ndCommandShareInfo(const ndBrainLayer* const layer)
		:m_inputSize(0)
		,m_outputSize(0)
		,m_inputOutputSize(0)
		,m_inputOutputStartOffset(0)
		,m_parametersBatchSize(0)
		,m_parametersStartOffset(0)
		,m_tiledStride(0)
		,m_layer(layer)
	{
	}

	ndInt32 m_inputSize;
	ndInt32 m_outputSize;
	ndInt32 m_inputOutputSize;
	ndInt32 m_inputOutputStartOffset;
	ndInt32 m_parametersBatchSize;
	ndInt32 m_parametersStartOffset;
	ndInt32	m_tiledStride;
	const ndBrainLayer* m_layer;
};

class ndBrainBufferCommandDesc : public ndFixSizeArray<ndBrainBuffer*, 16>
{
	public:
	ndBrainBufferCommandDesc(ndInt32 minibatchSize)
		:m_context(nullptr)
		,m_owner(nullptr)
		,m_info()
		,m_uniformBuffer()
		,m_id(0)
		,m_miniBatchSize(minibatchSize)
	{
	}

	ndBrainContext* m_context;
	ndBrainTrainerInference* m_owner;
	ndCommandShareInfo m_info;
	ndSharedPtr<ndBrainUniformBuffer> m_uniformBuffer;
	size_t m_id;
	ndInt32 m_miniBatchSize;
};

class ndBrainBufferCommand : public ndContainersFreeListAlloc<ndBrainBufferCommand>
{
	public:
	ndBrainBufferCommand(const ndBrainBufferCommandDesc& desc);
	
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