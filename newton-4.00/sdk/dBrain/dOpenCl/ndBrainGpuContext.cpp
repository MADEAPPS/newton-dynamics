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
#include "ndBrainVector.h"
#include "ndBrainKernel.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainBufferCommand.h"

#define ND_OPENCL_SELECTION_TYPE	CL_DEVICE_TYPE_ALL
//#define ND_OPENCL_SELECTION_TYPE	CL_DEVICE_TYPE_CPU
//#define ND_OPENCL_SELECTION_TYPE	CL_DEVICE_TYPE_GPU

ndBrainGpuContext* ndBrainGpuContext::GetAsGpuContext()
{
	return this;
}

void ndBrainGpuContext::BrainVectorFromDevice(ndBrainFloatBuffer& src, ndBrainVector& dstVector)
{
	size_t sizeInBytes = ndMin(size_t(src.SizeInBytes()), size_t(dstVector.GetCount() * sizeof(ndReal)));
	MemoryFromDevice(src, 0, sizeInBytes, &dstVector[0]);
}

void ndBrainGpuContext::BrainVectorToDevice(ndBrainFloatBuffer& dst, const ndBrainVector& srcVector)
{
	size_t sizeInBytes = ndMin(size_t(dst.SizeInBytes()), size_t(srcVector.GetCount() * sizeof(ndReal)));
	MemoryToDevice(dst, 0, sizeInBytes, &srcVector[0]);
}

void ndBrainGpuContext::CopyBuffer(const ndCopyBufferCommandInfo& parameters, ndInt32 numberOfWorkGrups, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	m_copyBufferParams->MemoryToDevice(0, sizeof(ndCopyBufferCommandInfo), &parameters);

	ndBrainBufferCommandDesc& descriptor = m_copyBufferCommand->GetDescriptor();
	descriptor.SetCount(0);
	descriptor.PushBack(*m_copyBufferParams);
	descriptor.PushBack((ndBrainBuffer*)&dstData);
	descriptor.PushBack((ndBrainBuffer*)&srcData);

	descriptor.m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
	descriptor.m_miniBatchSize = numberOfWorkGrups;
	SubmitBufferCommand(*m_copyBufferCommand);
}

void ndBrainGpuContext::CopyBufferIndirect(const ndCopyBufferCommandInfo& parameters, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	m_copyBufferParams->MemoryToDevice(0, sizeof(ndCopyBufferCommandInfo), &parameters);

	ndBrainBufferCommandDesc& descriptor = m_copyBufferIndirectCommand->GetDescriptor();
	descriptor.SetCount(0);
	descriptor.PushBack(*m_copyBufferParams);
	descriptor.PushBack((ndBrainBuffer*)&dstData);
	descriptor.PushBack((ndBrainBuffer*)&srcData);
	descriptor.PushBack((ndBrainBuffer*)&indexBuffer);

	descriptor.m_workGroupSize = ND_DEFAULT_WORKGROUP_SIZE;
	descriptor.m_miniBatchSize = ndInt32(indexBuffer.SizeInBytes() / sizeof(ndUnsigned32));
	SubmitBufferCommand(*m_copyBufferIndirectCommand);
}

ndBrainGpuContext::ndBrainGpuContext()
	:ndBrainContext()
{
	m_supportMappedMemory = false;

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
			cl_platforms[i].getDevices(ND_OPENCL_SELECTION_TYPE, &cl_devices_available);
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

		cl_int error = 0;
		m_context = ndSharedPtr<cl::Context>(new cl::Context(**m_device, nullptr, clNotification, this, &error));
		ndAssert(error == CL_SUCCESS);

		cl_command_queue_properties properties = CL_QUEUE_PROFILING_ENABLE;
		m_queue = ndSharedPtr<cl::CommandQueue>(new cl::CommandQueue(**m_context, **m_device, properties, &error));
		ndAssert(error == CL_SUCCESS);

		m_emptyBuffer = cl::Buffer(**m_context, CL_MEM_READ_WRITE, 256);

		cl_device_svm_capabilities svm_caps(m_device->getInfo<CL_DEVICE_SVM_CAPABILITIES>(&error));

		if (error == CL_SUCCESS)
		{
			//if (svm_caps & (CL_DEVICE_SVM_COARSE_GRAIN_BUFFER | CL_DEVICE_SVM_FINE_GRAIN_BUFFER))
			if (svm_caps & CL_DEVICE_SVM_FINE_GRAIN_BUFFER)
			{
				m_supportMappedMemory = true;
			}
		}

		CreateKerners();
		CreateCopyCommands();
	}
}

ndBrainGpuContext::~ndBrainGpuContext()
{
}

//void CL_CALLBACK ndBrainGpuContext::clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data)
void CL_CALLBACK ndBrainGpuContext::clNotification(const char*, const void*, size_t, void*)
{
	ndAssert(0);
}

bool ndBrainGpuContext::SupportsMappedMemory() const
{
	return m_supportMappedMemory;
}

size_t ndBrainGpuContext::GetDeviceScore(cl::Device& device)
{
	//std::string name (device.getInfo<CL_DEVICE_VENDOR>());
	//std::string lowerName;
	//for (size_t i = 0; i < name.size(); ++i)
	//{
	//	int ch = tolower(name[i]);
	//	lowerName.push_back(char(ch));
	//}
	//
	//// some heuristic to select a device. I am no using specific vendor gpu/cpu/fpga types.
	//size_t computeUnits = device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
	//size_t clockMegaHertz = device.getInfo<CL_DEVICE_MAX_CLOCK_FREQUENCY>();
	//const bool is_gpu = device.getInfo<CL_DEVICE_TYPE>() == CL_DEVICE_TYPE_GPU;
	//size_t wavefront = size_t(is_gpu ? 64 : 8);
	//size_t amd = size_t((lowerName.find("amd") != std::string::npos) ? 1 : 0);
	//size_t arm = size_t((lowerName.find("arm") != std::string::npos) ? 1 : 0);
	//size_t intel = size_t((lowerName.find("intel") != std::string::npos) ? 1 : 0);
	//size_t nvidia = size_t((lowerName.find("nvidia") != std::string::npos) ? 1 : 0);
	//amd = amd | (size_t((lowerName.find("advanced micro devices, inc.") != std::string::npos) ? 1 : 0));
	//size_t cores = computeUnits * (nvidia + amd + intel + arm);
	//size_t megaFlops = cores * wavefront * clockMegaHertz;

	const bool is_gpu = device.getInfo<CL_DEVICE_TYPE>() == CL_DEVICE_TYPE_GPU;
	size_t computeUnits = device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
	size_t wavefront = size_t(is_gpu ? 64 : 8);
	size_t flopsPerClock = computeUnits * wavefront;

	size_t localMemorySize = device.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>();
	if (localMemorySize < 1024 * 32)
	{
		flopsPerClock = 0;
	}
	return flopsPerClock;
}

void ndBrainGpuContext::CreateCopyCommands()
{
	ndBrainBufferCommandDesc copyDescriptor(0);
	copyDescriptor.m_context = this;
	copyDescriptor.m_kernel = m_brainCopyStridedBuffer;
	m_copyBufferCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyDescriptor));

	ndBrainBufferCommandDesc copyIndirectDescriptor(0);
	copyIndirectDescriptor.m_context = this;
	copyIndirectDescriptor.m_kernel = m_brainCopyStridedBufferIndirect;
	m_copyBufferIndirectCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyIndirectDescriptor));

	ndCopyBufferCommandInfo copyBuffer;
	m_copyBufferParams = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(this, sizeof(ndCopyBufferCommandInfo), &copyBuffer, true));
}

void ndBrainGpuContext::MemoryToDevice(ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, const void* const srcMemory) const
{
	ndAssert((sizeInBytes & 3) == 0);
	ndAssert(sizeInBytes <= deviceBuffer.SizeInBytes());

	cl_int error = 0;
	const cl::CommandQueue* queue = *m_queue;
	ndBrainGpuBuffer* const buffer = deviceBuffer.GetGpuBuffer();
	if (buffer->m_memory)
	{
		// if the queue has to be flushed to get the memory,
		// then to me, this is not different that just calling enqueueWriteBuffer
		// but I use it in case ther is some different
		//error = m_queue->finish();
		//ndAssert(error == CL_SUCCESS);

		ndAssert(buffer->m_owner->m_isMemoryMapped);
		ndInt64 size = ndInt64(sizeInBytes / sizeof(ndUnsigned32));
		ndInt64 offset = ndInt64(offsetInBytes / sizeof(ndUnsigned32));
		ndMemCpy(&buffer->m_memory[offset], (ndUnsigned32*)srcMemory, size);
	}
	else
	{
		error = queue->enqueueWriteBuffer(**buffer->m_buffer, CL_TRUE, offsetInBytes, sizeInBytes, srcMemory);
		ndAssert(error == CL_SUCCESS);
	}
}

void ndBrainGpuContext::MemoryFromDevice(const ndBrainBuffer& deviceBuffer, size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const
{
	ndAssert((sizeInBytes & 3) == 0);
	ndAssert(sizeInBytes <= deviceBuffer.SizeInBytes());

	cl_int error = 0;
	const cl::CommandQueue* queue = *m_queue;
	const ndBrainGpuBuffer* const buffer = deviceBuffer.GetGpuBuffer();
	if (buffer->m_memory)
	{
		// if the queue has to be flushed to get the memory,
		// then to me, this is not different that just calling enqueueWriteBuffer
		// but I use it in case ther is some different
		//error = m_queue->finish();
		//ndAssert(error == CL_SUCCESS);

		ndAssert(buffer->m_owner->m_isMemoryMapped);
		ndInt64 size = ndInt64(sizeInBytes / sizeof(ndUnsigned32));
		ndInt64 offset = ndInt64(offsetInBytes / sizeof(ndUnsigned32));
		ndMemCpy((ndUnsigned32*)outputMemory, &buffer->m_memory[offset], size);
	}
	else
	{
		error = queue->enqueueReadBuffer(**buffer->m_buffer, CL_TRUE, offsetInBytes, sizeInBytes, outputMemory);
		ndAssert(error == CL_SUCCESS);
	}
}

void ndBrainGpuContext::SyncBufferCommandQueue()
{
	cl_int error = 0;
	error = m_queue->finish();
	//error = m_queue->flush();
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::SubmitBufferCommand(ndBrainBufferCommand* const command)
{
	cl_int error = 0;
	cl_int numberOfParameters = 0;
	ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	OpenclKernel* const kernel = (OpenclKernel*)*desc.m_kernel;
	cl::Kernel* const shader = *kernel->m_shader;

	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(error == CL_SUCCESS);
	
	for (ndInt32 i = 0; i < numberOfParameters; ++i)
	{
		ndBrainGpuBuffer* const arg = *desc[i]->m_gpuBuffer;
		error = shader->setArg(cl_uint(i), arg ? **arg->m_buffer : m_emptyBuffer);
		ndAssert(error == CL_SUCCESS);
	}
	
	cl::NDRange offset(0);
	cl::NDRange local(size_t(desc.m_workGroupSize));
	cl::NDRange global(size_t(desc.m_workGroupSize * desc.m_miniBatchSize));
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer)
{
	cl_int error = 0;
	OpenclKernel* const oclKernel = (OpenclKernel*)*kernel;
	cl::Kernel* const shader = *oclKernel->m_shader;

	//, ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer
	ndBrainGpuBuffer* const dst = buffer->GetGpuBuffer();
	const ndBrainGpuBuffer* const src = srcBuffer->GetGpuBuffer();

	ndInt32 numberOfElements = ndInt32(buffer->SizeInBytes() / sizeof(float));
	error = shader->setArg(0, numberOfElements);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, **src->m_buffer);
	ndAssert(error == CL_SUCCESS);

	ndInt32 numberOfGroups = (numberOfElements + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;

	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(numberOfGroups);
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	SubmitMathOperation(m_mathBufferAssigment, &dstData, &srcData);
}

void ndBrainGpuContext::Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale)
{
	ndAssert(0);
}

void ndBrainGpuContext::Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(0);
}

void ndBrainGpuContext::Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(0);
}

void ndBrainGpuContext::Sub(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(0);
}

void ndBrainGpuContext::Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	ndAssert(0);
}
