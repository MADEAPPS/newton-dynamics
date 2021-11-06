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

#define D_USE_GPU_DEVICE
//#define D_DEBUG_GPU_KERNELS

#define D_OPENCL_BUFFER_SIZE	1024


template<class T>
class dOpenclBuffer: public dArray<T>
{
	public:
	dOpenclBuffer(cl_mem_flags flags);
	~dOpenclBuffer();

	void Cleanup();
	void SyncSize(cl_context context, dInt32 size);
	void ReadData(cl_command_queue commandQueue);
	void WriteData(cl_command_queue commandQueue);

	cl_mem m_gpuBuffer;
	cl_mem_flags m_flags;
};

class ndOpenclBodyBuffer
{
	public:
	typedef union 
	{
		cl_float8 m_data;
		struct
		{
			cl_float4 m_linear;
			cl_float4 m_angular;
		};
	} ndOpenclJacobian;

	ndOpenclBodyBuffer();
	~ndOpenclBodyBuffer();

	void Cleanup();
	void Resize(cl_context context, dArray<ndBodyKinematic*>& bodyArray);
	void CopyToGpu(cl_command_queue commandQueue, const dArray<ndBodyKinematic*>& bodyArray);
	void SetKernelParameters(cl_kernel kernel, dFloat32 timestep, const dArray<ndBodyKinematic*>& bodyArray);

#ifdef D_DEBUG_GPU_KERNELS
	dVector MakeQuat(const dVector& axis, float angle);
	dVector MultiplyQuat(const dVector& r, const dVector& q);
	dVector NormalizeQuat(const dVector& r);
	void DebudKernel(dFloat32 timestepIn, const dArray<ndBodyKinematic*>& bodyArray);
#endif

	dOpenclBuffer<ndOpenclJacobian> m_transform;
	dOpenclBuffer<ndOpenclJacobian> m_veloc;
	dOpenclBuffer<ndOpenclJacobian> m_accel;
};

class ndOpenclSystem: public dClassAlloc
{
	public:
	class ndKernel
	{
		public:
		ndKernel()
			:m_kernel(nullptr)
			,m_workWroupSize(0)
		{
		}

		cl_kernel m_kernel;
		size_t m_workWroupSize;
	};

	ndOpenclSystem(cl_context context, cl_platform_id);
	~ndOpenclSystem();

	void Finish();
	cl_program CompileProgram();
	void Resize(dArray<ndBodyKinematic*>& bodyArray);
	void SetKernel(const char* const name, ndKernel& kerner);
	void CopyToGpu(const dArray<ndBodyKinematic*>& bodyArray);
	void ExecuteIntegrateBodyPosition(dFloat32 timestep, const dArray<ndBodyKinematic*>& bodyArray);

	static ndOpenclSystem* Singleton(dInt32 driveNumber);

	ndOpenclBodyBuffer m_bodyArray;
	char m_platformName[128];

	// Regular OpenCL objects:
	cl_context m_context;					// hold the context handler
	cl_device_id m_device;					// hold the selected device handler
	cl_program	m_solverProgram;			// hold the program handler
	cl_command_queue m_commandQueue;		// hold the commands-queue handler

	ndKernel m_integrateBodiesPosition;
	ndKernel m_integrateBodiesVelocity;
	ndKernel m_integrateUnconstrainedBodies;
	static const char* m_kernelSource;
	dInt32 m_computeUnits;
};

template<class T>
dOpenclBuffer<T>::dOpenclBuffer(cl_mem_flags flags)
	:dArray<T>()
	,m_gpuBuffer(nullptr)
	,m_flags(flags)
{
}

template<class T>
dOpenclBuffer<T>::~dOpenclBuffer()
{
	dAssert(!m_gpuBuffer);
}

template<class T>
void dOpenclBuffer<T>::Cleanup()
{
	if (m_gpuBuffer)
	{
		cl_int err = CL_SUCCESS;
		err = clReleaseMemObject(m_gpuBuffer);
		dAssert(err == CL_SUCCESS);
		dArray<T>::Resize(0);
	}
	m_gpuBuffer = nullptr;
}

template<class T>
void dOpenclBuffer<T>::SyncSize(cl_context context, dInt32 size)
{
	cl_int err = CL_SUCCESS;

	if (m_gpuBuffer == nullptr)
	{
		if (m_flags & CL_MEM_USE_HOST_PTR)
		{
			dAssert(0);
			//void* const hostBuffer = &(*this)[0];
			//m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * dArray<T>::GetCapacity(), hostBuffer, &err);
		}
		else
		{
			m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * size, nullptr, &err);
		}
		dAssert(err == CL_SUCCESS);
		dArray<T>::Resize(size);
	}
	else
	{
		dAssert(0);
	}
}

template<class T>
void dOpenclBuffer<T>::ReadData(cl_command_queue commandQueue)
{
	cl_int err = CL_SUCCESS;
	void* const destination = &(*this)[0];
	err = clEnqueueReadBuffer(
		commandQueue, m_gpuBuffer,
		CL_FALSE, 0, sizeof(T) * dArray<T>::GetCount(), destination,
		0, nullptr, nullptr);
	dAssert(err == CL_SUCCESS);
}

template<class T>
void dOpenclBuffer<T>::WriteData(cl_command_queue commandQueue)
{
	const void* const source = &(*this)[0];

	cl_int err = CL_SUCCESS;
	err = clEnqueueWriteBuffer(
		commandQueue, m_gpuBuffer,
		CL_FALSE, 0, sizeof(T) * dArray<T>::GetCount(), source,
		0, nullptr, nullptr);
	dAssert(err == CL_SUCCESS);
}

