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
#ifndef __ND_BRAIN_KERNEL_H__
#define __ND_BRAIN_KERNEL_H__

#include "ndBrainStdafx.h"

class ndBrainBuffer;
class ndBrainContext;

class ndBrainKernel : public ndContainersFreeListAlloc<ndBrainKernel>
{
	public:
	ndBrainKernel(ndBrainContext* const context);
	virtual ~ndBrainKernel();

	virtual void Execute(ndInt32 groupId, ndInt32 workGroupSize) = 0;

	ndBrainContext* m_context;
	ndFixSizeArray<ndBrainBuffer*, 8> m_parameters;
};

#endif