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
#ifndef __ND_BRAIN_COMMAND_BUFFER_H__
#define __ND_BRAIN_COMMAND_BUFFER_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"

class ndBrainContext;
class ndBrainUniformBuffer;
class ndBrainTrainerInference;

class ndBrainCommandBufferDesc : public ndFixSizeArray<ndBrainBuffer*, 16>
{
	public:
	ndBrainCommandBufferDesc()
		:m_context(nullptr)
		,m_owner(nullptr)
		,m_info()
		,m_uniformBuffer()
		,m_id(0)
	{

	}

	ndBrainContext* m_context;
	ndBrainTrainerInference* m_owner;
	ndBrainLayer::ndCommandShareInfo m_info;
	ndSharedPtr<ndBrainUniformBuffer> m_uniformBuffer;
	size_t m_id;
};

class ndBrainCommandBuffer : public ndContainersFreeListAlloc<ndBrainCommandBuffer>
{
	public:
	ndBrainCommandBuffer(const ndBrainCommandBufferDesc& desc);
	//ndBrainCommandBuffer(
	//	ndBrainContext* const context, ndBrainTrainerInference* const owner, 
	//	size_t id, ndBrainLayer::ndCommandShareInfo& info, ndInt32 numberOfWorkGroups,
	//	const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer, ndFixSizeArray<ndBrainBuffer*, 16>& buffers)
	//void Assembly(const ndSharedPtr<ndBrainGpuShader>& shader, ndInt32 workGroupSize, ndInt32 buffersCount, ndBrainBuffer** buffer);

	protected:
	//ndBrainContext* m_context;
	//ndBrainTrainerInference* m_owner;
	//ndBrainLayer::ndCommandShareInfo m_info;
	//ndSharedPtr<ndBrainUniformBuffer> m_uniformBuffer;
	//size_t m_id;
	ndBrainCommandBufferDesc m_desc;
};

#endif