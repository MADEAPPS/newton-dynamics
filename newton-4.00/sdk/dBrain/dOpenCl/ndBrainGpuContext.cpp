/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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

ndBrainGpuContext* ndBrainGpuContext::GetAsGpuContext()
{
	return this;
}

//void CL_CALLBACK ndBrainGpuContext::clNotification(const char* errinfo, const void* private_info, size_t cb, void* user_data)
void CL_CALLBACK ndBrainGpuContext::clNotification(const char*, const void*, size_t, void*)
{
	ndAssert(0);
}

bool ndBrainGpuContext::SupportsMappedMemory() const
{
	//return m_supportMappedMemory;
	return false;
}

size_t ndBrainGpuContext::GetDeviceScore(cl::Device& device)
{
	const bool is_gpu = device.getInfo<CL_DEVICE_TYPE>() == CL_DEVICE_TYPE_GPU;
	size_t computeUnits = device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
	std::vector<::size_t> workItems(device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>());
	size_t wavefront = size_t(is_gpu ? workItems[0] : 8);
	size_t flopsPerClock = computeUnits * wavefront;
	return flopsPerClock;
}

void ndBrainGpuContext::CreateCopyCommands()
{
	ndBrainBufferCommandDesc copyDescriptor(0);
	copyDescriptor.m_context = this;
	copyDescriptor.m_kernel = m_brainCopyStridedBuffer;
	m_copyStridedBufferCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyDescriptor));

	ndBrainBufferCommandDesc copyIndirectDescriptor(0);
	copyIndirectDescriptor.m_context = this;
	copyIndirectDescriptor.m_kernel = m_brainCopyStridedBufferIndirect;
	m_copyStridedBufferIndirectCommand = ndSharedPtr<ndBrainGpuCommand>(new ndBrainGpuCommand(copyIndirectDescriptor));

	ndCopyBufferCommandInfo copyBuffer;
	m_copyStridedBufferParams = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(this, sizeof(ndCopyBufferCommandInfo), &copyBuffer, true));
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
		error = m_queue->finish();
		ndAssert(error == CL_SUCCESS);

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
		error = m_queue->finish();
		ndAssert(error == CL_SUCCESS);

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
	cl_int error = 0;
	OpenclKernel* const oclKernel = (OpenclKernel*)*m_brainCopyStridedBuffer;
	cl::Kernel* const shader = *oclKernel->m_shader;

	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 7);

	//uint strideInByte,
	//uint srcStrideInByte,
	//uint srcOffsetInByte,
	//uint dstStrideInByte,
	//uint dstOffsetInByte,
	//__global float* outputData,
	//__global float* inputData

	ndBrainGpuBuffer* const dst = dstData.GetGpuBuffer();
	const ndBrainGpuBuffer* const src = srcData.GetGpuBuffer();

	error = shader->setArg(0, cl_uint(parameters.m_strideInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, cl_uint(parameters.m_srcStrideInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, cl_uint(parameters.m_srcOffsetInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(3, cl_uint(parameters.m_dstStrideInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(4, cl_uint(parameters.m_dstOffsetInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(5, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(6, **src->m_buffer);
	ndAssert(error == CL_SUCCESS);

	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(size_t(numberOfWorkGrups * ND_DEFAULT_WORKGROUP_SIZE));
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::CopyBufferIndirect(const ndCopyBufferCommandInfo& parameters, const ndBrainIntegerBuffer& indexBuffer, ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	cl_int error = 0;
	OpenclKernel* const oclKernel = (OpenclKernel*)*m_brainCopyStridedBufferIndirect;
	cl::Kernel* const shader = *oclKernel->m_shader;

	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 8);

	//uint strideInByte,
	//uint srcStrideInByte,
	//uint srcOffsetInByte,
	//uint dstStrideInByte,
	//uint dstOffsetInByte,
	//__global float* outputData,
	//__global float* inputData

	ndBrainGpuBuffer* const dst = dstData.GetGpuBuffer();
	const ndBrainGpuBuffer* const src = srcData.GetGpuBuffer();
	const ndBrainGpuBuffer* const indirectBuffer = indexBuffer.GetGpuBuffer();

	error = shader->setArg(0, cl_uint(parameters.m_strideInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, cl_uint(parameters.m_srcStrideInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, cl_uint(parameters.m_srcOffsetInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(3, cl_uint(parameters.m_dstStrideInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(4, cl_uint(parameters.m_dstOffsetInByte));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(5, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(6, **src->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(7, **indirectBuffer->m_buffer);
	ndAssert(error == CL_SUCCESS);

	ndInt32 numberOfWorkGroups = ndInt32(indexBuffer.SizeInBytes() / sizeof(ndUnsigned32));
	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(size_t(numberOfWorkGroups * ND_DEFAULT_WORKGROUP_SIZE));
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::SyncBufferCommandQueue()
{
	cl_int error = 0;
	error = m_queue->finish();
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::SubmitBufferCommand(ndBrainBufferCommand* const command)
{
	cl_int error = 0;
	cl_int numberOfParameters = 0;
	ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	OpenclKernel* const kernel = (OpenclKernel*)*desc.m_kernel;
	ndAssert(kernel);
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

void ndBrainGpuContext::SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, float scalarValue)
{
	cl_int error = 0;
	OpenclKernel* const oclKernel = (OpenclKernel*)*kernel;
	cl::Kernel* const shader = *oclKernel->m_shader;

	ndBrainGpuBuffer* const dst = buffer->GetGpuBuffer();
	size_t numberOfElements = size_t(buffer->SizeInBytes() / sizeof(float));
	size_t numberOfGroups = (numberOfElements + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;

	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 3);

	error = shader->setArg(0, ndInt32(numberOfElements));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, scalarValue);
	ndAssert(error == CL_SUCCESS);

	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(numberOfGroups);
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer)
{
	cl_int error = 0;
	OpenclKernel* const oclKernel = (OpenclKernel*)*kernel;
	cl::Kernel* const shader = *oclKernel->m_shader;

	ndBrainGpuBuffer* const dst = buffer->GetGpuBuffer();
	const ndBrainGpuBuffer* const src = srcBuffer->GetGpuBuffer();

	size_t numberOfElements = size_t(buffer->SizeInBytes() / sizeof(float));
	size_t numberOfGroups = (numberOfElements + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;

	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 3);

	error = shader->setArg(0, ndInt32(numberOfElements));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, **src->m_buffer);
	ndAssert(error == CL_SUCCESS);

	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(numberOfGroups);
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer, float scale)
{
	cl_int error = 0;
	OpenclKernel* const oclKernel = (OpenclKernel*)*kernel;
	cl::Kernel* const shader = *oclKernel->m_shader;

	ndBrainGpuBuffer* const dst = buffer->GetGpuBuffer();
	const ndBrainGpuBuffer* const src = srcBuffer->GetGpuBuffer();

	size_t numberOfElements = size_t(buffer->SizeInBytes() / sizeof(float));
	size_t numberOfGroups = (numberOfElements + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;

	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 4);

	error = shader->setArg(0, ndInt32(numberOfElements));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, **src->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(3, scale);
	ndAssert(error == CL_SUCCESS);

	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(numberOfGroups);
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::SubmitMathOperation(const ndSharedPtr<ndBrainKernel>& kernel, ndBrainBuffer* const buffer, const ndBrainBuffer* const srcBuffer0, const ndBrainBuffer* const srcBuffer1)
{
	cl_int error = 0;
	OpenclKernel* const oclKernel = (OpenclKernel*)*kernel;
	cl::Kernel* const shader = *oclKernel->m_shader;

	ndBrainGpuBuffer* const dst = buffer->GetGpuBuffer();
	const ndBrainGpuBuffer* const src0 = srcBuffer0->GetGpuBuffer();
	const ndBrainGpuBuffer* const src1 = srcBuffer1->GetGpuBuffer();

	size_t numberOfElements = size_t(buffer->SizeInBytes() / sizeof(float));
	size_t numberOfGroups = (numberOfElements + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;

	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 4);

	error = shader->setArg(0, ndInt32(numberOfElements));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, **src0->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(3, **src1->m_buffer);
	ndAssert(error == CL_SUCCESS);

	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(numberOfGroups);
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::BroadcastScaler(ndBrainFloatBuffer& buffer, ndInt32 bufferStrideInFloats, const ndBrainFloatBuffer& srcScalar)
{
	cl_int error = 0;

	OpenclKernel* const oclKernel = (OpenclKernel*)*m_brainBroadcastScalar;
	cl::Kernel* const shader = *oclKernel->m_shader;
	
	ndBrainGpuBuffer* const dst = buffer.GetGpuBuffer();
	const ndBrainGpuBuffer* const src = srcScalar.GetGpuBuffer();
	
	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 3);
	
	error = shader->setArg(0, ndInt32(bufferStrideInFloats));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, **src->m_buffer);
	ndAssert(error == CL_SUCCESS);
	
	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(size_t(numberOfParameters * ND_DEFAULT_WORKGROUP_SIZE));
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::Select(ndBrainFloatBuffer& buffer, ndBrainFloatBuffer& maskBuffer, ndBrainFloat a, ndBrainFloat b)
{
	cl_int error = 0;

	OpenclKernel* const oclKernel = (OpenclKernel*)*m_brainSelect;
	cl::Kernel* const shader = *oclKernel->m_shader;
	
	ndBrainGpuBuffer* const dst = buffer.GetGpuBuffer();
	const ndBrainGpuBuffer* const mask = maskBuffer.GetGpuBuffer();
	
	cl_int numberOfParameters = 0;
	error = shader->getInfo(CL_KERNEL_NUM_ARGS, &numberOfParameters);
	ndAssert(numberOfParameters == 5);

	size_t numberOfElements = size_t(buffer.SizeInBytes() / sizeof(float));
	size_t numberOfGroups = (numberOfElements + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;
	
	error = shader->setArg(0, ndInt32(numberOfElements));
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(1, **dst->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(2, **mask->m_buffer);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(3, a);
	ndAssert(error == CL_SUCCESS);
	error = shader->setArg(4, b);
	ndAssert(error == CL_SUCCESS);

	cl::NDRange offset(0);
	cl::NDRange local(ND_DEFAULT_WORKGROUP_SIZE);
	cl::NDRange global(numberOfGroups);
	error = m_queue->enqueueNDRangeKernel(*shader, offset, global, local);
	ndAssert(error == CL_SUCCESS);
}

void ndBrainGpuContext::Set(ndBrainFloatBuffer& dstData, ndBrainFloat value)
{
	SubmitMathOperation(m_brainSet, &dstData, value);
}

void ndBrainGpuContext::Set(ndBrainFloatBuffer& dstData, const ndBrainFloatBuffer& srcData)
{
	SubmitMathOperation(m_brainAssigment, &dstData, &srcData);
}

void ndBrainGpuContext::CopyBuffer(ndBrainBuffer& dstData, const ndBrainBuffer& srcData)
{
	SubmitMathOperation(m_brainAssigment, &dstData, &srcData);
}

void ndBrainGpuContext::ScaleAdd(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat scale)
{
	SubmitMathOperation(m_brainScaleAdd, &buffer, &srcBuffer, scale);
}

void ndBrainGpuContext::Scale(ndBrainFloatBuffer& buffer, ndBrainFloat scale)
{
	SubmitMathOperation(m_brainScale, &buffer, scale);
}

void ndBrainGpuContext::Min(ndBrainFloatBuffer& buffer, ndBrainFloat value)
{
	SubmitMathOperation(m_brainMinScalar, &buffer, value);
}

void ndBrainGpuContext::Max(ndBrainFloatBuffer& buffer, ndBrainFloat value)
{
	SubmitMathOperation(m_brainMaxScalar, &buffer, value);
}

void ndBrainGpuContext::Min(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	SubmitMathOperation(m_brainMin, &buffer, &srcBuffer);
}

void ndBrainGpuContext::Max(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	SubmitMathOperation(m_brainMax, &buffer, &srcBuffer);
}

void ndBrainGpuContext::Add(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	SubmitMathOperation(m_brainAdd, &buffer, &srcBuffer);
}

void ndBrainGpuContext::Sub(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	SubmitMathOperation(m_brainSub, &buffer, &srcBuffer);
}

void ndBrainGpuContext::Mul(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	SubmitMathOperation(m_brainMul, &buffer, &srcBuffer);
}

void ndBrainGpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, ndBrainFloat blend)
{
	SubmitMathOperation(m_brainBlendScale, &buffer, &srcBuffer, blend);
}

void ndBrainGpuContext::Blend(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer, const ndBrainFloatBuffer& blend)
{
	SubmitMathOperation(m_brainBlendVector, &buffer, &srcBuffer, &blend);
}

void ndBrainGpuContext::LessEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	SubmitMathOperation(m_brainLessEqual, &buffer, &srcBuffer);
}

void ndBrainGpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, const ndBrainFloatBuffer& srcBuffer)
{
	SubmitMathOperation(m_brainGreaterEqual, &buffer, &srcBuffer);
}

void ndBrainGpuContext::LessEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	SubmitMathOperation(m_brainLessEqualScalar, &buffer, test);
}

void ndBrainGpuContext::GreaterEqual(ndBrainFloatBuffer& buffer, ndBrainFloat test)
{
	SubmitMathOperation(m_brainGreaterEqualScalar, &buffer, test);
}

void ndBrainGpuContext::StandardNormalDistribution(ndBrainFloatBuffer&)
{
	ndAssert(0);
}