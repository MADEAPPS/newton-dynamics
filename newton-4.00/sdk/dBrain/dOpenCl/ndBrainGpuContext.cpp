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


#define D_OPENCL_SELECTION_TYPE		CL_DEVICE_TYPE_ALL
//#define D_OPENCL_SELECTION_TYPE	CL_DEVICE_TYPE_CPU
//#define D_OPENCL_SELECTION_TYPE	CL_DEVICE_TYPE_GPU

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
{
	m_supportsMappedMemory = false;

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
		size_t bestScore = 0;
		size_t bestDeviceIndex = 0;
		for (size_t i = 0; i < cl_devices.size(); ++i)
		{
			size_t score = GetDeviceScore(cl_devices[i]);
			if (score > bestScore)
			{
				bestScore = score;
				bestDeviceIndex = i;
			}
		}

		m_device = ndSharedPtr<cl::Device>(new cl::Device(cl_devices[bestDeviceIndex]));
		const std::string name(m_device->getInfo<CL_DEVICE_NAME>());
		const std::string version(m_device->getInfo<CL_DEVICE_VERSION>());
		size_t localMemorySize = m_device->getInfo<CL_DEVICE_LOCAL_MEM_SIZE>();
		size_t compute_units = m_device->getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();

		ndExpandTraceMessage("\n");
		ndExpandTraceMessage("selecting:\n");
		ndExpandTraceMessage("opencl device name: %s\n", name.c_str());
		ndExpandTraceMessage("opencl device version: %s\n", version.c_str());
		
		ndExpandTraceMessage("opencl device compute units: %d\n", compute_units);
		ndExpandTraceMessage("opencl device local memory: %d\n", localMemorySize);
		ndExpandTraceMessage("\n");

		ndAssert(localMemorySize >= 1024 * 24);
		
		cl_int error = 0;
		m_context = ndSharedPtr<cl::Context>(new cl::Context(**m_device, nullptr, clNotification, this, &error));
		ndAssert(error == CL_SUCCESS);

		cl_command_queue_properties properties = CL_QUEUE_PROFILING_ENABLE;
		m_queue = ndSharedPtr<cl::CommandQueue>(new cl::CommandQueue(**m_context , **m_device, properties, &error));
		ndAssert(error == CL_SUCCESS);

		m_emptyBuffer = cl::Buffer(**m_context, CL_MEM_READ_WRITE, 256);

		cl_device_svm_capabilities svm_caps(m_device->getInfo<CL_DEVICE_SVM_CAPABILITIES>(&error));

		if (error == CL_SUCCESS) 
		{
			if (svm_caps & (CL_DEVICE_SVM_COARSE_GRAIN_BUFFER | CL_DEVICE_SVM_FINE_GRAIN_BUFFER))
			{
				m_supportsMappedMemory = true;
			}
		}

		CreateKerners();
		CreateCopyCommands();
	}
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

bool ndBrainGpuContext::SupportsMappedMemory() const
{
	return m_supportsMappedMemory;
}

size_t ndBrainGpuContext::GetDeviceScore(cl::Device& device)
{
	std::string name = device.getInfo<CL_DEVICE_NAME>();
	
	std::string lowerName;
	for (size_t i = 0; i < name.size(); ++i)
	{
		int ch = tolower(name[i]);
		lowerName.push_back(char (ch));
	}

	// some heuristic to select a device. I am no using specific vendor gpu/cpu/fpga types.
	size_t computeUnits = device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
	size_t clockMegaHertz = device.getInfo<CL_DEVICE_MAX_CLOCK_FREQUENCY>();
	const bool is_gpu = device.getInfo<CL_DEVICE_TYPE>() == CL_DEVICE_TYPE_GPU;
	size_t wavefront = size_t(is_gpu ? 64 : 8);
	size_t amd = size_t((lowerName.find("amd") != std::string::npos) ? 1 : 0);
	size_t arm = size_t((lowerName.find("arm") != std::string::npos) ? 1 : 0);
	size_t intel = size_t((lowerName.find("intel") != std::string::npos) ? 1 : 0);
	size_t nvidia = size_t((lowerName.find ("nvidia") != std::string::npos) ? 1 : 0);

	size_t cores = computeUnits * (nvidia + amd + intel + arm);
	size_t megaFlops = cores * wavefront * clockMegaHertz;
	return megaFlops;
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

void ndBrainGpuContext::CreateCopyCommands()
{
	ndBrainLayer::ndCommandShareInfo uniformParam;
	m_copyBufferCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(this, uniformParam));
	m_copyBufferIndirectCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(this, uniformParam));
}

#if 0
void ndBrainGpuContext::CopyBuffer(
	size_t minibatchSize, size_t strideInBytes,
	ndBrainGpuFloatBuffer& dstBuffer, size_t dstOffsetInBytes, size_t dstStrideInBytes,
	ndBrainGpuFloatBuffer& srcData, size_t srcOffsetInBytes, size_t srcStrideInBytes)
{
	ndBrainLayer::ndCommandShareInfo uniformParam;
	uniformParam.m_outputSize = ndInt32(strideInBytes / sizeof(ndReal));
	uniformParam.m_inputSize = ndInt32(srcStrideInBytes / sizeof(ndReal));
	uniformParam.m_inputOutputStartOffset = ndInt32(srcOffsetInBytes / sizeof(ndReal));

	uniformParam.m_parametersBatchSize = ndInt32(dstStrideInBytes / sizeof(ndReal));
	uniformParam.m_parametersStartOffset = ndInt32(dstOffsetInBytes / sizeof(ndReal));

	m_copyBufferCommandParamBuffer->MemoryToDevice(0, sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam);

	ndFixSizeArray<ndBrainBuffer*, 8> params;
	params.PushBack(*m_copyBufferCommandParamBuffer);
	params.PushBack(&srcData);
	params.PushBack(&dstBuffer);

	m_copyBufferCommand->Assembly(m_brainCopyBuffer, ndInt32(minibatchSize), params.GetCount(), &params[0]);
	AddCommandQueue(m_copyBufferCommand);
}
#endif

void ndBrainGpuContext::CopyBuffer(ndBrainGpuUniformBuffer& parameterBuffer, ndInt32 workGroups, ndBrainGpuFloatBuffer& dstBuffer, ndBrainGpuFloatBuffer& srcBuffer)
{
	//ndCopyBufferCommandInfo uniformParam;
	//parameterBuffer.MemoryFromDevice(0, sizeof(ndCopyBufferCommandInfo), &uniformParam);

	ndFixSizeArray<ndBrainBuffer*, 8> params;
	params.PushBack(&parameterBuffer);
	params.PushBack(&srcBuffer);
	params.PushBack(&dstBuffer);

	m_copyBufferCommand->Assembly(m_brainCopyBuffer, workGroups, params.GetCount(), &params[0]);
	AddCommandQueue(m_copyBufferCommand);
}

void ndBrainGpuContext::CopyBufferIndirect(ndBrainGpuUniformBuffer& parameterBuffer, ndBrainGpuIntegerBuffer& indexBuffer, ndBrainGpuFloatBuffer& dstBuffer, ndBrainGpuFloatBuffer& srcBuffer)
{
	//ndCopyBufferCommandInfo uniformParam;
	//parameterBuffer.MemoryFromDevice(0, sizeof(ndCopyBufferCommandInfo), &uniformParam);

	ndFixSizeArray<ndBrainBuffer*, 8> params;
	params.PushBack(&parameterBuffer);
	params.PushBack(&indexBuffer);
	params.PushBack(&srcBuffer);
	params.PushBack(&dstBuffer);
	ndInt32 minibatchSize = ndInt32(indexBuffer.SizeInBytes() / sizeof(ndUnsigned32));

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
		error = shader->setArg(cl_uint(i), argBuffer ? **argBuffer->m_buffer : m_emptyBuffer);
		ndAssert(error == CL_SUCCESS);
	}

	cl::NDRange offset(0);
	cl::NDRange local(command->m_workGroupSize);
	cl::NDRange global(command->m_workGroupSize * command->m_numberOfWorkGroups);
	error = m_queue->enqueueNDRangeKernel(**command->m_shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}