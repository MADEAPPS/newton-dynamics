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

#include "ndNewtonStdafx.h"
#include <cuda.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

//#define D_USE_GPU_DEVICE
//#define D_OPENCL_BUFFER_SIZE	1024

#if 0
template<class T>
class ndCudaBuffer: public ndArray<T>
{
	public:
	ndCudaBuffer(cl_mem_flags flags);
	~ndCudaBuffer();

	void Cleanup();
	void SyncSize(cl_context context, ndInt32 size);
	void ReadData(cl_command_queue commandQueue);
	void WriteData(cl_command_queue commandQueue);

	cl_mem m_gpuBuffer;
	cl_mem_flags m_flags;
};

class ndCudaBodyBuffer
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
	} ndCudaJacobian;

	ndCudaBodyBuffer();
	~ndCudaBodyBuffer();

	void Cleanup();
	void Resize(cl_context context, const ndArray<ndInt32>& bodyArray);
	void CopyToGpu(cl_command_queue commandQueue, const ndArray<ndInt32>& bodyArray);
	void SetKernelParameters(cl_kernel kernel, ndFloat32 timestep, const ndArray<ndBodyKinematic*>& bodyArray);

#ifdef D_DEBUG_GPU_KERNELS
	dVector MakeQuat(const dVector& axis, float angle);
	dVector MultiplyQuat(const dVector& r, const dVector& q);
	dVector NormalizeQuat(const dVector& r);
	void DebudKernel(ndFloat32 timestepIn, const ndArray<ndBodyKinematic*>& bodyArray);
#endif

	ndCudaBuffer<ndCudaJacobian> m_transform;
	ndCudaBuffer<ndCudaJacobian> m_veloc;
	ndCudaBuffer<ndCudaJacobian> m_accel;
};

class ndCudaSystem: public ndClassAlloc
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

	ndCudaSystem(cl_context context, cl_platform_id);
	~ndCudaSystem();

	void Finish();
	cl_program CompileProgram();
	void Resize(const ndArray<ndInt32>& bodyArray);
	void CopyToGpu(const ndArray<ndInt32>& bodyArray);
	void SetKernel(const char* const name, ndKernel& kerner);
	void ExecuteIntegrateBodyPosition(ndFloat32 timestep, const ndArray<ndBodyKinematic*>& bodyArray);

	static ndCudaSystem* Singleton(ndInt32 driveNumber);

	ndCudaBodyBuffer m_bodyArray;
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
	ndInt32 m_computeUnits;
};

template<class T>
ndCudaBuffer<T>::ndCudaBuffer(cl_mem_flags flags)
	:ndArray<T>()
	,m_gpuBuffer(nullptr)
	,m_flags(flags)
{
}

template<class T>
ndCudaBuffer<T>::~ndCudaBuffer()
{
	dAssert(!m_gpuBuffer);
}

template<class T>
void ndCudaBuffer<T>::Cleanup()
{
	if (m_gpuBuffer)
	{
		cl_int err = CL_SUCCESS;
		err = clReleaseMemObject(m_gpuBuffer);
		dAssert(err == CL_SUCCESS);
		ndArray<T>::Resize(0);
	}
	m_gpuBuffer = nullptr;
}

template<class T>
void ndCudaBuffer<T>::SyncSize(cl_context context, ndInt32 size)
{
	cl_int err = CL_SUCCESS;

	if (m_gpuBuffer == nullptr)
	{
		if (m_flags & CL_MEM_USE_HOST_PTR)
		{
			dAssert(0);
			//void* const hostBuffer = &(*this)[0];
			//m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * ndArray<T>::GetCapacity(), hostBuffer, &err);
		}
		else
		{
			m_gpuBuffer = clCreateBuffer(context, m_flags, sizeof(T) * size, nullptr, &err);
		}
		dAssert(err == CL_SUCCESS);
		ndArray<T>::Resize(size);
	}
	else
	{
		dAssert(0);
	}
}

template<class T>
void ndCudaBuffer<T>::ReadData(cl_command_queue commandQueue)
{
	cl_int err = CL_SUCCESS;
	void* const destination = &(*this)[0];
	err = clEnqueueReadBuffer(
		commandQueue, m_gpuBuffer,
		CL_FALSE, 0, sizeof(T) * ndArray<T>::GetCount(), destination,
		0, nullptr, nullptr);
	dAssert(err == CL_SUCCESS);
}

template<class T>
void ndCudaBuffer<T>::WriteData(cl_command_queue commandQueue)
{
	const void* const source = &(*this)[0];

	cl_int err = CL_SUCCESS;
	err = clEnqueueWriteBuffer(
		commandQueue, m_gpuBuffer,
		CL_FALSE, 0, sizeof(T) * ndArray<T>::GetCount(), source,
		0, nullptr, nullptr);
	dAssert(err == CL_SUCCESS);
}
#endif


class ndCudaSystem : public ndClassAlloc
{
	public: 
	ndCudaSystem();
	~ndCudaSystem();
	static ndCudaSystem* Singleton();

	struct cudaDeviceProp m_prop;
};
