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
#include "ndBrain.h"
//#include "ndBrainVector.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuInference.h"

#if defined (D_USE_VULKAN_SDK)

ndBrainGpuInference::ndBrainGpuInference(ndBrainGpuContext* const context, ndBrain* const brain)
	:ndClassAlloc()
	,m_brain(brain)
	,m_context(context)
{
	brain->GetParameterVector(m_parameters, m_offsets);
	m_gpuOffsets = new ndBrainGpuIntegerBuffer(m_context, m_offsets);
	m_gpuParameters = new ndBrainGpuFloatBuffer(m_context, m_parameters);

}

ndBrainGpuInference::~ndBrainGpuInference()
{
	delete m_gpuOffsets;
	delete m_gpuParameters;

}

#endif