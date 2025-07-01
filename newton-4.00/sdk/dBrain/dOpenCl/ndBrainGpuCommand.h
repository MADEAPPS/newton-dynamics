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
#ifndef __ND_BRAIN_GPU_COMMAND_H__
#define __ND_BRAIN_GPU_COMMAND_H__

#include "ndBrainStdafx.h"
#include "ndBrainBufferCommand.h"

#if 0
#include "ndBrainLayer.h"
#include "ndBrainGpuContext.h"

class ndBrainGpuBuffer;
class ndBrainGpuFloatBuffer;
class ndBrainGpuUniformBuffer;

class ndBrainGpuCommand : public ndClassAlloc
{
	public:
	ndBrainGpuCommand(ndBrainGpuContext* const context, const ndBrainLayer::ndCommandShareInfo& info);
	virtual ~ndBrainGpuCommand();
	void Assembly(const ndSharedPtr<ndBrainGpuShader>& shader, ndInt32 workGroupSize, ndInt32 buffersCount, ndBrainBuffer** buffer);

	protected:
	ndBrainGpuContext* m_context;
	ndSharedPtr<ndBrainGpuShader> m_shader;
	ndBrainLayer::ndCommandShareInfo m_info;
	ndBrainLayer* m_layer;
	ndFixSizeArray<ndBrainBuffer*, 8> m_parameters;
	size_t m_workGroupSize;
	size_t m_numberOfWorkGroups;

	friend class ndBrainGpuContext;
	friend class ndBrainTrainerGpu;
	friend class ndBrainTrainerGpuInference;
};
#endif


class ndBrainGpuCommand : public ndBrainBufferCommand
{
	public:
	ndBrainGpuCommand(const ndBrainBufferCommandDesc& desc);
	virtual ~ndBrainGpuCommand();
};

#endif