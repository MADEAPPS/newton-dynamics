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
#ifndef __ND_BRAIN_CPU_CONTEXT_H__
#define __ND_BRAIN_CPU_CONTEXT_H__

#include "ndBrainStdafx.h"
#include "ndBrainContext.h"
#include "ndBrainThreadPool.h"

class ndBrainCpuContext : 
	public ndBrainContext, 
	public ndBrainThreadPool
{
	public:
	ndBrainCpuContext();
	virtual ~ndBrainCpuContext();

	virtual ndContextType GetType() const override;
	virtual ndBrainCpuContext* GetAsCpuContext() override;
};

#endif