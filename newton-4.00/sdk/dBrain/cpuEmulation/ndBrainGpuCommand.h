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

class ndBrainGpuCommand : public ndBrainBufferCommand
{
	public:
	ndBrainGpuCommand(const ndBrainBufferCommandDesc& desc);
	virtual ~ndBrainGpuCommand();

	virtual void Execute(ndInt32 groupId);
};


#endif