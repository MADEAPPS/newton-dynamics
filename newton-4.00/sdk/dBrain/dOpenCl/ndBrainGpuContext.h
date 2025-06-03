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
#ifndef __ND_BRAIN_GPU_CONTEXT_H__
#define __ND_BRAIN_GPU_CONTEXT_H__

#include "ndBrainStdafx.h"
#include "ndBrainContext.h"

class ndBrainGpuCommand;
class ndBrainGpuFloatBuffer;

typedef void* ndVulkanShader;

class ndBrainGpuContext : public ndBrainContext
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();
	virtual ndContextType GetType() const override;

	void SyncQueue() {}
	void AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>&) {}

	ndInt32 GetSubGroupSize() const { return 0; }
	static bool HasGpuSupport() { return false; }

	union
	{
		struct
		{
			ndVulkanShader m_ndBrainCopyInput;
			ndVulkanShader m_ndBrainCopyOutput;
			ndVulkanShader m_ndBrainLayerLinear;
			ndVulkanShader m_ndBrainCopyOutputGradients;
			ndVulkanShader m_ndBrainLayerReluActivation;
			ndVulkanShader m_ndBrainLayerTanhActivation;
			ndVulkanShader m_ndBrainLayerSoftmaxActivation;
			ndVulkanShader m_ndBrainLayerLinearDropOutActivation;
		};
		ndVulkanShader m_modules[128];
	};
};
#endif