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

ndBrainGpuCommandTest::ndBrainGpuCommandTest(ndBrainGpuContext* const context,
	ndBrainGpuUniformBuffer& parammeters,
	ndBrainGpuFloatBuffer& input, ndBrainGpuFloatBuffer& output)
	:ndBrainGpuCommand(context)
{
	ndBrainGpuBuffer* params[3];
	params[0] = &parammeters;
	params[1] = &input;
	params[2] = &output;
	ndAssert(0);
	//Assembly(context->m_testShader, 1, 3, params);
};