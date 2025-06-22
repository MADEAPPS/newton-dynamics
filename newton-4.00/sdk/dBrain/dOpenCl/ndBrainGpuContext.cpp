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
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"
#include "ndBrainGpuIntegerBuffer.h"

#include <string>
#include <vector>

//#define ND_DEBUG_KERNELS

#define D_OPENCL_SELECTION_TYPE		CL_DEVICE_TYPE_ALL
//#define D_OPENCL_SELECTION_TYPE	CL_DEVICE_TYPE_CPU
//#define D_OPENCL_SELECTION_TYPE	CL_DEVICE_TYPE_GPU

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
{
	// get all devices of all platforms
	std::vector<cl::Device> cl_devices; 
	{
		std::vector<cl::Platform> cl_platforms; 
		cl::Platform::get(&cl_platforms);
		ndExpandTraceMessage("\n");
		ndExpandTraceMessage("platforms found:\n");
		for (size_t i = 0; i < cl_platforms.size(); i++)
		{
			std::vector<cl::Device> cl_devices_available;
			cl_platforms[i].getDevices(D_OPENCL_SELECTION_TYPE, &cl_devices_available);
			for (size_t j = 0; j < cl_devices_available.size(); j++)
			{
				const std::string name(cl_platforms[i].getInfo<CL_PLATFORM_NAME>());
				ndExpandTraceMessage("opencl platform: %s\n", name.c_str());
				cl_devices.push_back(cl_devices_available[j]);
			}
		}
	}

	// select fastest available device
	if (cl_devices.size())
	{
		size_t bestDeviceIndex = 0;
		#ifdef ND_DEBUG_KERNELS
		for (size_t i = 0u; i < cl_devices.size(); i++)
		{
			const std::string name (cl_devices[i].getInfo<CL_DEVICE_NAME>()); 
			size_t index = name.find("Oclgrind");
			if (index != size_t (-1))
			{
				bestDeviceIndex = i;
			}
		}
		#endif

		m_device = ndSharedPtr<cl::Device>(new cl::Device(cl_devices[bestDeviceIndex]));
		const std::string name(m_device->getInfo<CL_DEVICE_NAME>());
		const std::string version(m_device->getInfo<CL_DEVICE_VERSION>());
		size_t localMemorySize = m_device->getInfo<CL_DEVICE_LOCAL_MEM_SIZE>();
		//size_t compute_units = m_device->getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
		size_t compute_units = m_device->getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
		ndExpandTraceMessage("\n");
		ndExpandTraceMessage("selecting:\n");
		ndExpandTraceMessage("opencl device name: %s\n", name.c_str());
		ndExpandTraceMessage("opencl device version: %s\n", version.c_str());
		
		ndExpandTraceMessage("opencl device compute units: %d\n", compute_units);
		ndExpandTraceMessage("opencl device local memory: %d\n", localMemorySize);

		ndAssert(localMemorySize >= 1024 * 24);
		
		cl_int error = 0;
		m_context = ndSharedPtr<cl::Context>(new cl::Context(**m_device, nullptr, clNotification, this, &error));
		ndAssert(error == CL_SUCCESS);

		cl_command_queue_properties properties = CL_QUEUE_PROFILING_ENABLE;
		m_queue = ndSharedPtr<cl::CommandQueue>(new cl::CommandQueue(**m_context , **m_device, properties, &error));
		ndAssert(error == CL_SUCCESS);

		m_emptyBuffer = cl::Buffer(**m_context, CL_MEM_READ_WRITE, 256);

		CreateKerners();
		CreateCopyIndirectCommand();
	}
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

bool ndBrainGpuContext::HasGpuSupport() 
{ 
	return true; 
}

//void CL_CALLBACK ndBrainGpuContext::clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data)
void CL_CALLBACK ndBrainGpuContext::clNotification(const char*, const void*, size_t, void*)
{
	ndAssert(0);
}

ndBrainContext::ndContextType ndBrainGpuContext::GetType() const
{
	return ndBrainContext::m_gpu;
}

ndBrainGpuContext* ndBrainGpuContext::GetAsGpuContext()
{
	return this;
}

void ndBrainGpuContext::CreateCopyIndirectCommand()
{
	ndBrainLayer::ndCommandShareInfo uniformParam;
	//uniformParam.m_inputSize = uniformData.m_inputSize;
	//uniformParam.m_outputSize = uniformData.m_outputSize;
	//uniformParam.m_parametersStartOffset = 0;
	//uniformParam.m_inputOutputSize = inputOutputBufferSize;
	//uniformParam.m_inputOutputStartOffset = 0;
	m_copyBufferIndirectCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(this, uniformParam));
	m_copyBufferIndirectCommandParamBuffer = ndSharedPtr<ndBrainGpuUniformBuffer>(new ndBrainGpuUniformBuffer(this, sizeof(ndBrainLayer::ndCommandShareInfo)));
}

void ndBrainGpuContext::CopyBufferIndirectSource(ndBrainGpuFloatBuffer& dstBuffer, ndBrainGpuIntegerBuffer& indexBuffer, ndBrainGpuFloatBuffer& srcDataBuffer, ndInt32 srcStrideInBytes)
{
	ndFixSizeArray<ndBrainBuffer*, 8> params;
	params.PushBack(*m_copyBufferIndirectCommandParamBuffer);
	params.PushBack(&indexBuffer);
	params.PushBack(&srcDataBuffer);
	params.PushBack(&dstBuffer);
	ndInt32 stride = srcStrideInBytes / sizeof(ndReal);
	ndInt32 minibatchSize = indexBuffer.SizeInBytes() / sizeof (ndUnsigned32);

	m_copyBufferIndirectCommand->Assembly(m_brainCopyBufferIndirect, minibatchSize, params.GetCount(), &params[0]);
	AddCommandQueue(m_copyBufferIndirectCommand);
}

void ndBrainGpuContext::SyncQueue()
{
	cl_int error = 0;
	error = m_queue->finish();
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::AddCommandQueue(const ndSharedPtr<ndBrainGpuCommand>& command)
{
	cl_int error = 0;
	cl_int numberOfParameters = 0;
	ndSharedPtr<ndBrainGpuShader>& shader = command->m_shader;
	
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(error == CL_SUCCESS);

	for (ndInt32 i = 0; i < numberOfParameters; ++i)
	{
		ndBrainGpuBuffer* const argBuffer = (ndBrainGpuBuffer*)command->m_parameters[i];
		error = shader->setArg(cl_uint(i), argBuffer ? argBuffer->m_buffer : m_emptyBuffer);
		ndAssert(error == CL_SUCCESS);
	}

	cl::NDRange offset(0);
	cl::NDRange local(command->m_workGroupSize);
	cl::NDRange global(command->m_workGroupSize * command->m_numberOfWorkGroups);
	error = m_queue->enqueueNDRangeKernel(**command->m_shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}