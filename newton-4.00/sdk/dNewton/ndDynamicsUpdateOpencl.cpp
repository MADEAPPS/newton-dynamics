/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdateOpencl.h"
#include "ndJointBilateralConstraint.h"

#ifdef _D_NEWTON_OPENCL
#include <CL/cl.h>

//using namespace ndOpencl;

class OpenclSystem
{
	public:
	OpenclSystem(cl_context context, cl_platform_id platform)
		:m_context(context)
		//,device(nullptr)
		//,commandQueue(nullptr)
		//,program(nullptr)
		//,kernel(nullptr)
		//,platformVersion(CL_TARGET_OPENCL_VERSION)
		//,deviceVersion(CL_TARGET_OPENCL_VERSION)
		//,compilerVersion(CL_TARGET_OPENCL_VERSION)
		//,srcA(nullptr)
		//,srcB(nullptr)
		//,dstMem(nullptr)
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

		char programFile[256];
		sprintf(programFile, "%s/CL/solver/solver.cl", CL_KERNEL_PATH);
		m_solverProgram = CompileProgram(programFile);
	}

	~OpenclSystem()
	{
		cl_int err;

		err = clReleaseProgram(m_solverProgram);
		dAssert(err == CL_SUCCESS);

		err = clReleaseCommandQueue(m_commandQueue);
		dAssert(err == CL_SUCCESS);

		err = clReleaseDevice(m_device);
		dAssert(err == CL_SUCCESS);

		err = clReleaseContext(m_context);
		dAssert(err == CL_SUCCESS);
	}

	cl_program CompileProgram(char* const filename)
	{
		FILE* const fp = fopen(filename, "rb");
		if (fp)
		{
			size_t sourceSize;
			fseek(fp, 0, SEEK_END);
			sourceSize = ftell(fp);
			fseek(fp, 0, SEEK_SET);

			char* const source = dAlloca(char, sourceSize * 256);
			fread(source, 1, sourceSize, fp);
			fclose(fp);

			int errorCode;
			cl_program program = clCreateProgramWithSource(m_context, 1, (const char**)&source, &sourceSize, &errorCode);
			dAssert(errorCode == CL_SUCCESS);

			errorCode = clBuildProgram(program, 1, &m_device, "", nullptr, nullptr);
			if (errorCode == CL_BUILD_PROGRAM_FAILURE)
			{
				size_t log_size = 0;
				clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, 0, NULL, &log_size);

				char* const build_log = dAlloca(char, log_size * 256);
				clGetProgramBuildInfo(program, m_device, CL_PROGRAM_BUILD_LOG, log_size, build_log, nullptr);
				dTrace((build_log));
			}
			dAssert(errorCode == CL_SUCCESS);

			return program;
		}
		return nullptr;
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

	// Regular OpenCL objects:
	cl_context m_context;					// hold the context handler
	cl_device_id m_device;					// hold the selected device handler
	cl_program	m_solverProgram;			// hold the program handler
	cl_command_queue m_commandQueue;		// hold the commands-queue handler

	//cl_kernel        kernel;				// hold the kernel handler
	//cl_mem           srcA;				// hold first source buffer
	//cl_mem           srcB;				// hold second source buffer
	//cl_mem           dstMem;				// hold destination buffer
	char m_platformName[128];
};


ndDynamicsUpdateOpencl::ndDynamicsUpdateOpencl(ndWorld* const world)
	:ndDynamicsUpdate(world)
	,m_openCl(nullptr)
{
	m_openCl = OpenclSystem::Singleton();
}

ndDynamicsUpdateOpencl::~ndDynamicsUpdateOpencl()
{
	if (m_openCl)
	{
		delete m_openCl;
	}
}

const char* ndDynamicsUpdateOpencl::GetStringId() const
{
	return m_openCl ? m_openCl->m_platformName : "no opencl support";
}

void ndDynamicsUpdateOpencl::Update()
{
	if (m_openCl)
	{
		GpuUpdate();
	}
}

void ndDynamicsUpdateOpencl::GpuUpdate()
{

}



#endif