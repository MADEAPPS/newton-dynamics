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
//#include "ndBrainGpuBuffer.h"
//#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"

#ifdef D_USE_VULKAN_SDK
#include <vulkan/vulkan.h>
#include "ndBrainVulkanContext.h"
#endif

ndBrainGpuContext::ndBrainGpuContext()
{
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

void ndBrainGpuContext::Sync()
{
}

void ndBrainGpuContext::SubmitQueue(const ndList<ndSharedPtr<ndBrainGpuCommand>>&)
{
}

//bool ndBrainGpuContext::SupportVulkanBackEnd()
//{
//	return false;
//}

ndBrainGpuContext* ndBrainGpuContext::CreateVulkanContext()
{
	#ifdef D_USE_VULKAN_SDK
	return new ndBrainVulkanContext();
	#else
	return nullptr;
	#endif
}