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

#ifndef __ND_CUDA_DEVICE_BUFFER_H__
#define __ND_CUDA_DEVICE_BUFFER_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include "ndCudaUtils.h"
#include "ndCudaIntrinsics.h"

template<class T>
class ndCudaHostBuffer;

template<class T>
class ndCudaDeviceBuffer
{
	public:
	ndCudaDeviceBuffer();
	ndCudaDeviceBuffer(const ndCudaDeviceBuffer& src);
	~ndCudaDeviceBuffer();

	int GetCount() const;
	void SetCount(int count);

	void Clear();
	void Resize(int count);
	int GetCapacity() const;

	T& operator[] (int i);
	const T& operator[] (int i) const;

	ndCudaDeviceBuffer& operator= (const ndCudaHostBuffer<T>& src);
	ndCudaDeviceBuffer& operator= (const ndCudaDeviceBuffer<T>& src);

	void ReadData(const T* const src, int elements);
	void WriteData(T* const dst, int elements) const;

	void ReadData(const T* const src, int elements, cudaStream_t stream);
	void WriteData(T* const dst, int elements, cudaStream_t stream) const;

	T* m_array;
	int m_size;
	int m_capacity;
};

template<class T>
class ndAssessor
{
	public:
	ndAssessor()
		:m_array(nullptr)
		,m_size(0)
		,m_capacity(0)
	{
	}

	__device__ __host__ ndAssessor(const ndCudaDeviceBuffer<T>& buffer)
		:m_array(buffer.m_array)
		,m_size(buffer.m_size)
		,m_capacity(buffer.m_capacity)
	{
	}

	__device__ __host__ T& operator[] (int i)
	{
		return m_array[i];
	}

	__device__ __host__ const T& operator[] (int i) const
	{
		return m_array[i];
	}

	T* m_array;
	int m_size;
	int m_capacity;
};

template<class T>
ndCudaDeviceBuffer<T>::ndCudaDeviceBuffer()
	:m_array(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	SetCount(D_GRANULARITY);
	SetCount(0);
}

template<class T>
ndCudaDeviceBuffer<T>::ndCudaDeviceBuffer(const ndCudaDeviceBuffer<T>& src)
	:m_array(SetCount(src.GetCount()))
	,m_size(src.m_size)
	,m_capacity(src.m_capacity)
{
	cudaError_t cudaStatus = cudaSuccess;
	cudaStatus = cudaMemcpy(m_array, src.m_array, m_size * sizeof(T), cudaMemcpyDeviceToDevice);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

template<class T>
ndCudaDeviceBuffer<T>::~ndCudaDeviceBuffer()
{
	if (m_array)
	{
		cudaError_t cudaStatus = cudaSuccess;
		cudaStatus = cudaFree(m_array);
		ndAssert(cudaStatus == cudaSuccess);
		if (cudaStatus != cudaSuccess)
		{
			ndAssert(0);
		}
	}
}

template<class T>
const T& ndCudaDeviceBuffer<T>::operator[] (int i) const
{
	ndAssert(i >= 0);
	ndAssert(i < m_size);
	return m_array[i];
}

template<class T>
T& ndCudaDeviceBuffer<T>::operator[] (int i)
{
	ndAssert(i >= 0);
	ndAssert(i < m_size);
	return m_array[i];
}

template<class T>
ndCudaDeviceBuffer<T>& ndCudaDeviceBuffer<T>::operator= (const ndCudaDeviceBuffer<T>& src)
{
	cudaError_t cudaStatus;
	SetCount(src.GetCount());
	cudaStatus = cudaMemcpy(m_array, src.m_array, m_size * sizeof(T), cudaMemcpyDeviceToDevice);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
	return *this;
}

template<class T>
ndCudaDeviceBuffer<T>& ndCudaDeviceBuffer<T>::operator= (const ndCudaHostBuffer<T>& src)
{
	cudaError_t cudaStatus;
	SetCount(src.GetCount());
	cudaStatus = cudaMemcpy(m_array, src.m_array, m_size * sizeof(T), cudaMemcpyHostToDevice);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}

	return *this;
}

template<class T>
int ndCudaDeviceBuffer<T>::GetCount() const
{
	return m_size;
}

template<class T>
void ndCudaDeviceBuffer<T>::SetCount(int count)
{
	while (count > m_capacity)
	{
		Resize(m_capacity * 2);
	}
	m_size = count;
}

template<class T>
int ndCudaDeviceBuffer<T>::GetCapacity() const
{
	return m_capacity;
}

template<class T>
void ndCudaDeviceBuffer<T>::Clear()
{
	m_size = 0;
}

template<class T>
void ndCudaDeviceBuffer<T>::Resize(int newSize)
{
	cudaError_t cudaStatus = cudaSuccess;
	if (newSize > m_capacity || (m_capacity == 0))
	{
		T* newArray;
		newSize = std::max(newSize, D_GRANULARITY);
		const int itemSizeInBytes = sizeof(T);
		cudaStatus = cudaMalloc((void**)&newArray, newSize * itemSizeInBytes);
		ndAssert(cudaStatus == cudaSuccess);
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, m_size * itemSizeInBytes, cudaMemcpyDeviceToDevice);
			ndAssert(cudaStatus == cudaSuccess);
			cudaStatus = cudaFree(m_array);
			ndAssert(cudaStatus == cudaSuccess);
		}
		m_array = newArray;
		m_capacity = newSize;
	}
	else if (newSize < m_capacity)
	{
		T* newArray;
		const int itemSizeInBytes = sizeof(T);
		newSize = std::max(newSize, D_GRANULARITY);
		cudaStatus = cudaMalloc((void**)&newArray, newSize * itemSizeInBytes);
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, newSize * itemSizeInBytes, cudaMemcpyDeviceToDevice);
			cudaStatus = cudaFree(m_array);
			ndAssert(cudaStatus == cudaSuccess);
		}

		m_capacity = newSize;
		m_array = newArray;
	}
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

template<class T>
void ndCudaDeviceBuffer<T>::ReadData(const T* const src, int elements)
{
	cudaError_t cudaStatus;
	ndAssert(elements <= m_size);
	cudaStatus = cudaMemcpy(m_array, src, sizeof (T) * elements, cudaMemcpyHostToDevice);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

template<class T>
void ndCudaDeviceBuffer<T>::WriteData(T* const dst, int elements) const
{
	cudaError_t cudaStatus;
	ndAssert(m_size <= elements);
	cudaStatus = cudaMemcpy(dst, m_array, sizeof(T) * elements, cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

template<class T>
void ndCudaDeviceBuffer<T>::ReadData(const T* const src, int elements, cudaStream_t stream)
{
	ndAssert(elements <= m_size);
	cudaError_t cudaStatus = cudaMemcpyAsync(m_array, src, sizeof(T) * elements, cudaMemcpyHostToDevice, stream);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

template<class T>
void ndCudaDeviceBuffer<T>::WriteData(T* const dst, int elements, cudaStream_t stream) const
{
	ndAssert(elements <= m_size);
	cudaError_t cudaStatus = cudaMemcpyAsync(dst, m_array, sizeof(T) * elements, cudaMemcpyDeviceToHost, stream);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

#endif