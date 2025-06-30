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
#include "ndBrainLayer.h"
#include "ndBrainContext.h"
#include "ndBrainBufferCommand.h"

//class ndBrainKernel;
//class ndBrainGpuBuffer;
//class ndBrainFloatBuffer;
//class ndBrainUniformBuffer;


#if 0
class ndBrainGpuCommand : public ndContainersFreeListAlloc<ndBrainGpuCommand>
{
	public:
	ndBrainGpuCommand(ndBrainContext* const context, const ndCommandSharedInfo& info);
	virtual ~ndBrainGpuCommand();
	void Assembly(const ndSharedPtr<ndBrainKernel>& shader, ndInt32 workGroupSize, ndInt32 buffersCount, ndBrainBuffer** buffer);

	protected:
	ndBrainContext* m_context;
	ndSharedPtr<ndBrainKernel> m_shader;
	ndCommandSharedInfo m_info;
	ndBrainLayer* m_layer;
	ndFixSizeArray<ndBrainBuffer*, 8> m_parameters;
	size_t m_workGroupSize;
	size_t m_numberOfWorkGroups;

	friend class ndBrainContext;
	friend class ndBrainTrainerGpu;
	friend class ndBrainLayerLinear;
	friend class ndBrainTrainerGpuInference;
};

#endif

class ndBrainGpuCommand : public ndBrainBufferCommand
{
	public:
	ndBrainGpuCommand(const ndBrainBufferCommandDesc& desc);
	virtual ~ndBrainGpuCommand();

	virtual void Execute(ndInt32 groupId);
};


#endif