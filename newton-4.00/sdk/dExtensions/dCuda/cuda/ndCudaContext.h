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


template<class T>
class ndDeviceBuffer
{
	public:
	ndDeviceBuffer();
	~ndDeviceBuffer();

	ndInt32 GetCount() const;
	void SetCount(ndInt32 count);

	void Clear();
	void Resize(ndInt32 count);
	ndInt32 GetCapacity() const;

	T& operator[] (ndInt32 i);
	const T& operator[] (ndInt32 i) const;

	void ReadData(const T* const src, ndInt32 elements);
	void WriteData(T* const dst, ndInt32 elements) const;

	T* m_array;
	ndInt32 m_size;
	ndInt32 m_capacity;
};

#if 0
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

	ndDeviceBuffer<ndCudaJacobian> m_transform;
	ndDeviceBuffer<ndCudaJacobian> m_veloc;
	ndDeviceBuffer<ndCudaJacobian> m_accel;
};

class ndCudaContext: public ndClassAlloc
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

	ndCudaContext(cl_context context, cl_platform_id);
	~ndCudaContext();

	void Finish();
	cl_program CompileProgram();
	void Resize(const ndArray<ndInt32>& bodyArray);
	void CopyToGpu(const ndArray<ndInt32>& bodyArray);
	void SetKernel(const char* const name, ndKernel& kerner);
	void ExecuteIntegrateBodyPosition(ndFloat32 timestep, const ndArray<ndBodyKinematic*>& bodyArray);

	static ndCudaContext* CreateContext(ndInt32 driveNumber);

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
void ndDeviceBuffer<T>::SyncSize(cl_context context, ndInt32 size)
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
void ndDeviceBuffer<T>::ReadData(cl_command_queue commandQueue)
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
void ndDeviceBuffer<T>::WriteData(cl_command_queue commandQueue)
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

class ndCudaContext : public ndClassAlloc
{
	public: 
	ndCudaContext();
	~ndCudaContext();
	static ndCudaContext* CreateContext();

	struct cudaDeviceProp m_prop;
	ndDeviceBuffer<ndInt32> A;
	ndDeviceBuffer<ndInt32> B;
	ndDeviceBuffer<ndInt32> C;
};

template<class T>
ndDeviceBuffer<T>::ndDeviceBuffer()
	:m_array(nullptr)
	,m_size(0)
	,m_capacity(0)
{
}

template<class T>
ndDeviceBuffer<T>::~ndDeviceBuffer()
{
	if (m_array)
	{
		cudaFree(m_array);
	}
}

template<class T>
const T& ndDeviceBuffer<T>::operator[] (ndInt32 i) const
{
	dAssert(i >= 0);
	dAssert(i < m_size);
	return m_array[i];
}

template<class T>
T& ndDeviceBuffer<T>::operator[] (ndInt32 i)
{
	dAssert(i >= 0);
	dAssert(i < m_size);
	return m_array[i];
}

template<class T>
ndInt32 ndDeviceBuffer<T>::GetCount() const
{
	return m_size;
}

template<class T>
void ndDeviceBuffer<T>::SetCount(ndInt32 count)
{
	while (count > m_capacity)
	{
		Resize(m_capacity * 2);
	}
	m_size = count;
}

template<class T>
ndInt32 ndDeviceBuffer<T>::GetCapacity() const
{
	return m_capacity;
}

template<class T>
void ndDeviceBuffer<T>::Clear()
{
	m_size = 0;
}

template<class T>
void ndDeviceBuffer<T>::Resize(ndInt32 newSize)
{
	cudaError_t cudaStatus;
	if (newSize > m_capacity || (m_capacity == 0))
	{
		T* newArray;
		newSize = dMax(newSize, 16);
		cudaError_t cudaStatus = cudaMalloc((void**)&newArray, newSize * sizeof(T));
		dAssert(cudaStatus == cudaSuccess);
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, m_size * sizeof(T), cudaMemcpyDeviceToDevice);
			dAssert(cudaStatus == cudaSuccess);
			cudaStatus = cudaFree(m_array);
			dAssert(cudaStatus == cudaSuccess);
		}
		m_array = newArray;
		m_capacity = newSize;
	}
	else if (newSize < m_capacity)
	{
		newSize = dMax(newSize, 16);
		T* const newArray = (T*)ndMemory::Malloc(ndInt32(sizeof(T) * newSize));
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, newSize * sizeof(T), cudaMemcpyDeviceToDevice);
			cudaStatus = cudaFree(m_array);
			dAssert(cudaStatus == cudaSuccess);
		}

		m_capacity = newSize;
		m_array = newArray;
	}
}

template<class T>
void ndDeviceBuffer<T>::ReadData(const T* const src, ndInt32 elements)
{
	dAssert(elements <= m_size);
	cudaMemcpy(m_array, src, sizeof (T) * elements, cudaMemcpyHostToDevice);
}

template<class T>
void ndDeviceBuffer<T>::WriteData(T* const dst, ndInt32 elements) const
{
	dAssert(elements <= m_size);
	cudaMemcpy(dst, m_array, sizeof(T) * elements, cudaMemcpyDeviceToHost);
}

