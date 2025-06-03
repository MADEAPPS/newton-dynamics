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

typedef void* ndBrainGpuShader;

class ndBrainGpuContext : public ndBrainContext
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();
	virtual ndContextType GetType() const override;

	void SyncQueue() {}
	void AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>&) {}

	ndInt32 GetSubGroupSize() const { return 0; }
	static bool HasGpuSupport();

	union
	{
		struct
		{
			ndBrainGpuShader m_ndBrainCopyInput;
			ndBrainGpuShader m_ndBrainCopyOutput;
			ndBrainGpuShader m_ndBrainLayerLinear;
			ndBrainGpuShader m_ndBrainCopyOutputGradients;
			ndBrainGpuShader m_ndBrainLayerReluActivation;
			ndBrainGpuShader m_ndBrainLayerTanhActivation;
			ndBrainGpuShader m_ndBrainLayerSoftmaxActivation;
			ndBrainGpuShader m_ndBrainLayerLinearDropOutActivation;
		};
		ndBrainGpuShader m_modules[128];
	};

	cl::Device m_device;
	bool m_devicedInitialized;
};
#endif