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
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"


ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
{
	ndMemSet(m_modules, (void*)nullptr, sizeof(m_modules) / sizeof(m_modules[0]));
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

ndBrainContext::ndContextType ndBrainGpuContext::GetType() const
{
	return ndBrainContext::m_cpu;
}
