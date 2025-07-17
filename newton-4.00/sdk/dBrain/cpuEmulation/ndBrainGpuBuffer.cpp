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
#include "ndBrainBuffer.h"
#include "ndBrainGpuBuffer.h"

ndBrainGpuBuffer::ndBrainGpuBuffer(const ndBrainBuffer* const owner)
	:ndClassAlloc()
	,m_owner(owner)
{
	ndInt64 size = ndInt64(m_owner->SizeInBytes());
	m_memory.Resize(size);
	m_memory.SetCount(size);
}

void* ndBrainGpuBuffer::GetPtr()
{
	return &m_memory[0];
}

const void* ndBrainGpuBuffer::GetPtr() const
{
	return &m_memory[0];
}
