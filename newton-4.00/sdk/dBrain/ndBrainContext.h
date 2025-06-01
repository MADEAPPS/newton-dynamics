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
#ifndef __ND_BRAIN_CONTEXT_H__
#define __ND_BRAIN_CONTEXT_H__
#include "ndBrainStdafx.h"

class ndBrainGpuContext;
class ndBrainCpuContext;

class ndBrainContext : public ndClassAlloc
{
	public:
	enum ndContextType
	{
		m_cpu,
		m_gpu,
	};
	ndBrainContext();
	virtual ~ndBrainContext();

	virtual ndContextType GetType() const = 0;
	virtual ndBrainCpuContext* GetAsCpuContext();
	virtual ndBrainGpuContext* GetAsGpuContext();
};

#endif