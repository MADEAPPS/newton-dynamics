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

ndBrainBuffer::ndBrainBuffer(ndBrainContext* const context, ndInt64 sizeInByte, ndStorageBufferType)
	:m_context(context)
	,m_sizeInBytes(size_t(sizeInByte))
{
}

ndBrainBuffer::~ndBrainBuffer()
{
}
