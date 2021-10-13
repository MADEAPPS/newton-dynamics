/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include <CL/cl.h>
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdateOpencl.h"
#include "ndJointBilateralConstraint.h"

#if 0
template<class T>
class dOpenclBuffer: public dArray<T>
{
	public:
	dOpenclBuffer(cl_mem_flags flags)
		:dArray<T>(D_DEFAULT_BUFFER_SIZE)
		,m_flags(flags)
		,m_gpuBuffer(nullptr)
	{
		//SetCount(D_DEFAULT_BUFFER_SIZE);
		dArray<T>::SetCount(D_DEFAULT_BUFFER_SIZE * 10);
	}

	~dOpenclBuffer()
	{
		if (m_gpuBuffer)
		{
			cl_int err;
			err = clReleaseMemObject(m_gpuBuffer);
			dAssert(err == CL_SUCCESS);
		}
	}

	void SyncSize(cl_context context, dInt32 size)
	{
		cl_int err;

		if (m_gpuBuffer == nullptr)
		{
			if (m_flags & CL_MEM_USE_HOST_PTR)
			{
				void* const hostBuffer = &(*this)[0];
				m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * dArray<T>::GetCapacity(), hostBuffer, &err);
			}
			else
			{
				m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * dArray<T>::GetCapacity(), nullptr, &err);
			}
			dAssert(err == CL_SUCCESS);
		}

		if (dArray<T>::GetCapacity() < size)
		{
			dAssert(0);
		}
		dArray<T>::SetCount(size);
	}

	void ReadData(cl_command_queue commandQueue)
	{
		void* const source = &(*this)[0];
		cl_int err = clEnqueueReadBuffer(commandQueue, m_gpuBuffer,
			CL_FALSE, 0, sizeof(T) * dArray<T>::GetCount(), source,
			0, nullptr, nullptr);
		dAssert(err == CL_SUCCESS);
	}

	void WriteData(cl_command_queue commandQueue)
	{
		cl_int err;
		const void* const source = &(*this)[0];
		err = clEnqueueWriteBuffer(commandQueue, m_gpuBuffer,
			CL_FALSE, 0, sizeof(T) * dArray<T>::GetCount(), source,
			0, nullptr, nullptr);
		dAssert(err == CL_SUCCESS);
	}

	cl_mem_flags m_flags;
	cl_mem m_gpuBuffer;
};

class ndOpenclBodyProxy
{
	public:
	cl_float4 m_rotation;
	cl_float3 m_position;
	cl_float3 m_veloc;
	cl_float3 m_omega;
	cl_float4 m_invMass;
};

//class ndOpenclOutBodyProxy
//{
//	public:
//	cl_float3 m_matrix[3];
//	cl_float3 m_position;
//	cl_float4 m_rotation;
//	cl_float3 m_veloc;
//	cl_float3 m_omega;
//};

class ndOpenclMatrix3x3
{
	//cl_float3 m_matrix[3];
};


class ndOpenclBodyWorkingBuffer
{
	public:
	//ndOpenclMatrix3x3 m_matrix;
	cl_float4 m_rotation;
	cl_float3 m_position;
	cl_float3 m_veloc;
	cl_float3 m_omega;
};

#endif

class OpenclSystem: public dClassAlloc
{
	public:
	OpenclSystem(cl_context context, cl_platform_id platform)
		:m_context(context)
		////,m_bodyArray(CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR)
		//,m_bodyArray(CL_MEM_READ_ONLY)
		////,m_outBodyArray(CL_MEM_WRITE_ONLY)
		//,m_bodyWorkingArray(CL_MEM_READ_WRITE)
		,m_integrateUnconstrainedBodies(nullptr)
	{
		cl_int err;
		// get the device
		err = clGetContextInfo(m_context, CL_CONTEXT_DEVICES, sizeof(cl_device_id), &m_device, nullptr);
		dAssert(err == CL_SUCCESS);
		
		// get vendor driver support
		size_t stringLength = 0;
		err = clGetPlatformInfo(platform, CL_PLATFORM_NAME, 0, nullptr, &stringLength);
		dAssert(err == CL_SUCCESS);
		dAssert(stringLength < sizeof(m_platformName));
		err = clGetPlatformInfo(platform, CL_PLATFORM_NAME, stringLength, m_platformName, nullptr);
		dAssert(err == CL_SUCCESS);
		
		// get opencl version
		err = clGetDeviceInfo(m_device, CL_DEVICE_VERSION, 0, NULL, &stringLength);
		dAssert(err == CL_SUCCESS);
		dAssert(stringLength < sizeof(m_platformName));
		err = clGetDeviceInfo(m_device, CL_DEVICE_VERSION, stringLength, m_platformName, nullptr);
		dAssert(err == CL_SUCCESS);
		
		// create command queue
		cl_command_queue_properties properties = CL_QUEUE_PROFILING_ENABLE;
		m_commandQueue = clCreateCommandQueue(m_context, m_device, properties, &err);
		dAssert(err == CL_SUCCESS);
		
		//char programFile[256];
		//sprintf(programFile, "%s/CL/solver/solver.cl", CL_KERNEL_PATH);
		m_solverProgram = CompileProgram();
		
		m_integrateUnconstrainedBodies = clCreateKernel(m_solverProgram, "IntegrateUnconstrainedBodies", &err);
		dAssert(err == CL_SUCCESS);
	}

	~OpenclSystem()
	{
		cl_int err;
		
		err = clReleaseKernel(m_integrateUnconstrainedBodies);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseProgram(m_solverProgram);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseCommandQueue(m_commandQueue);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseDevice(m_device);
		dAssert(err == CL_SUCCESS);
		
		err = clReleaseContext(m_context);
		dAssert(err == CL_SUCCESS);
	}

	static OpenclSystem* Singleton()
	{
		cl_uint numPlatforms = 0;
		cl_int err = clGetPlatformIDs(0, nullptr, &numPlatforms);
		if ((err != CL_SUCCESS) || (numPlatforms == 0))
		{
			return nullptr;
		}

		dAssert(numPlatforms < 16);
		cl_platform_id platforms[16];
		err = clGetPlatformIDs(numPlatforms, &platforms[0], nullptr);
		if (err != CL_SUCCESS)
		{
			return nullptr;
		}

		cl_platform_id bestPlatform = 0;
		for (cl_uint i = 0; i < numPlatforms; i++)
		{
			cl_uint numDevices = 0;
			err = clGetDeviceIDs(platforms[i], CL_DEVICE_TYPE_GPU, 0, nullptr, &numDevices);
			if (!((err != CL_SUCCESS) || (numDevices == 0)))
			{
				bestPlatform = platforms[i];
			}
		}

		if (bestPlatform == nullptr)
		{
			return nullptr;
		}

		cl_context_properties contextProperties[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)bestPlatform, 0 };

		cl_context context = clCreateContextFromType(contextProperties, CL_DEVICE_TYPE_GPU, nullptr, nullptr, &err);
		if ((CL_SUCCESS != err) || (context == nullptr))
		{
			return nullptr;
		}

		return new OpenclSystem(context, bestPlatform);
	}

	cl_program CompileProgram()
	{
		int errorCode;
		size_t sourceSize = strlen (m_kernelSource);
		cl_program program = clCreateProgramWithSource(m_context, 1, (const char**)&m_kernelSource, &sourceSize, &errorCode);
		dAssert(errorCode == CL_SUCCESS);

		errorCode = clBuildProgram(program, 1, &m_device, "", nullptr, nullptr);
		if (errorCode == CL_BUILD_PROGRAM_FAILURE)
		{
			size_t log_size = 0;
			clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);

			char* const build_log = dAlloca(char, log_size + 4096);
			clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, log_size, build_log, nullptr);
			dTrace((build_log));
		}
		dAssert(errorCode == CL_SUCCESS);
		return program;
	}

	#if 0	
	//cl_kernel        kernel;				// hold the kernel handler
	

	dOpenclBuffer<ndOpenclBodyProxy> m_bodyArray;
	dOpenclBuffer<ndOpenclBodyWorkingBuffer> m_bodyWorkingArray;
//	dOpenclBuffer<ndOpenclOutBodyProxy> m_outBodyArray;


#endif

	char m_platformName[128];

	// Regular OpenCL objects:
	cl_context m_context;					// hold the context handler
	cl_device_id m_device;					// hold the selected device handler
	cl_program	m_solverProgram;			// hold the program handler
	cl_command_queue m_commandQueue;		// hold the commands-queue handler

	cl_kernel m_integrateUnconstrainedBodies;

	static char* m_kernelSource;
};


char* OpenclSystem::m_kernelSource = "\n\
struct ndOpenclMatrix3x3 \n\
{ \n\
	float3 m_element[3];\n\
}; \n\
\n\
struct ndOpenclBodyProxy \n\
{ \n\
	float4 m_rotation; \n\
	float3 m_position; \n\
	float3 m_veloc;	\n\
	float3 m_omega;	\n\
	float4 m_invMass; \n\
}; \n\
\n\
struct ndOpenclBodyWorkingBuffer \n\
{ \n\
	struct ndOpenclMatrix3x3 m_matrix; \n\
	float4 m_rotation; \n\
	float3 m_position; \n\
	float3 m_veloc;	\n\
	float3 m_omega;	\n\
}; \n\
\n\
struct ndOpenclMatrix3x3 QuatToMatrix(float4 rotation) \n\
{ \n\
	struct ndOpenclMatrix3x3 matrix; \n\
	return matrix; \n\
} \n\
\n\
__kernel void IntegrateUnconstrainedBodies(float timestep, int bodyCount, __global struct ndOpenclBodyProxy* inputArray, __global struct ndOpenclBodyWorkingBuffer* outputArray) \n\
{ \n\
	const int index = get_global_id(0); \n\
\n\
	struct ndOpenclBodyProxy body; \n\
\n\
	// load all variable into registers.\n\
	if (index < bodyCount) \n\
	{ \n\
		body = inputArray[index];\n\
	} \n\
	barrier(CLK_LOCAL_MEM_FENCE); \n\
\n\
	if (index < bodyCount) \n\
	{ \n\
		struct ndOpenclMatrix3x3 matrix; \n\
		matrix = QuatToMatrix(body.m_rotation); \n\
	} \n\
} \n\
\n\
";



ndDynamicsUpdateOpencl::ndDynamicsUpdateOpencl(ndWorld* const world)
	:ndDynamicsUpdate(world)
	,m_opencl(nullptr)
{
	m_opencl = OpenclSystem::Singleton();
}


ndDynamicsUpdateOpencl::~ndDynamicsUpdateOpencl()
{
	if (m_opencl)
	{
		delete m_opencl;
	}
}

const char* ndDynamicsUpdateOpencl::GetStringId() const
{
	return m_opencl ? m_opencl->m_platformName : "no opencl support";
}

void ndDynamicsUpdateOpencl::Update()
{
	if (m_opencl)
	{
		ndDynamicsUpdate::Update();
	}
	else
	{
		ndDynamicsUpdate::Update();
	}
}
