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

typedef cl::Kernel ndBrainGpuShader;

class ndBrainGpuContext : public ndBrainContext
{
	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();
	virtual ndContextType GetType() const override;

	void SyncQueue();
	void AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>& command); 

	ndInt32 GetSubGroupSize() const { return 0; }
	static bool HasGpuSupport();

	ndBrainGpuContext* GetAsGpuContext() override;

	private:
	void CreateKerners();
	ndSharedPtr<ndBrainGpuShader> CreateKerner(const cl::Program& program, const char* const functionMame) const;
	static void CL_CALLBACK clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data);

	public:
	ndSharedPtr<ndBrainGpuShader> m_ndBrainCopyInput;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainCopyOutput;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerLinear;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainCopyOutputGradients;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerReluActivation;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerTanhActivation;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerSoftmaxActivation;
	ndSharedPtr<ndBrainGpuShader> m_ndBrainLayerLinearDropOutActivation;

	private:
	ndSharedPtr<cl::Device> m_device;
	ndSharedPtr<cl::Context> m_context;
	ndSharedPtr<cl::CommandQueue> m_queue;

	static const char* m_kernelSource0;
	static const char* m_kernelSource1;
	static const char* m_kernelSource2;
	static const char* m_kernelSource3;

	friend class ndBrainGpuBuffer;
	friend class ndBrainGpuCommand;
	friend class ndBrainGpuFloatBuffer;
	friend class ndBrainGpuUniformBuffer;
};
#endif