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
#ifndef __ND_BRAIN_GPU_BUFFER_H__
#define __ND_BRAIN_GPU_BUFFER_H__

#include "ndBrainStdafx.h"

class ndBrainBuffer;

class ndBrainGpuBuffer : public ndClassAlloc
{
	public:
	void* GetPtr();
	const void* GetPtr() const;

	private:
	ndBrainGpuBuffer(const ndBrainBuffer* const owner);

	const ndBrainBuffer* m_owner;
	ndArray<ndInt8> m_memory;

	friend class ndBrainBuffer;
	friend class ndBrainGpuContext;
};

#endif