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
#include "ndBrainKernel.h"
#include "ndBrainContext.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainBufferCommand.h"

ndBrainBufferCommandDesc::ndBrainBufferCommandDesc(ndInt32 minibatchSize)
	:m_context(nullptr)
	,m_owner(nullptr)
	,m_info()
	,m_uniformBuffer()
	,m_id(0)
	,m_miniBatchSize(minibatchSize)
	,m_workGroupSize(ND_DEFAULT_WORKGROUP_SIZE)
{
}

ndBrainBufferCommand::ndBrainBufferCommand(const ndBrainBufferCommandDesc& desc)
	:ndContainersFreeListAlloc<ndBrainBufferCommand>()
	,m_desc(desc)
{
}

ndBrainBufferCommand::~ndBrainBufferCommand()
{
}

ndBrainBufferCommandDesc& ndBrainBufferCommand::GetDescriptor()
{
	return m_desc;
}

const ndBrainBufferCommandDesc& ndBrainBufferCommand::GetDescriptor() const
{
	return m_desc;
}

ndBrainBufferCommandCpu::ndBrainBufferCommandCpu(const ndBrainBufferCommandDesc& desc)
	:ndBrainBufferCommand(desc)
{
}

ndBrainBufferCommandCpu::~ndBrainBufferCommandCpu()
{
}
