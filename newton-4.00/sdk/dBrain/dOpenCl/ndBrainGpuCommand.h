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

class ndBrainGpuContext;
class ndBrainGpuBuffer;
class ndBrainGpuFloatBuffer;
class ndBrainGpuUniformBuffer;

class ndBrainGpuCommand : public ndClassAlloc
{
	public:
	ndBrainGpuCommand(ndBrainGpuContext* const context)
		:m_context(context)
	{
	}
	virtual ~ndBrainGpuCommand(){}

	void Assembly(void*, ndInt32, ndInt32, ndBrainGpuBuffer**) {}

	protected:
	ndBrainGpuContext* m_context;
};

#endif