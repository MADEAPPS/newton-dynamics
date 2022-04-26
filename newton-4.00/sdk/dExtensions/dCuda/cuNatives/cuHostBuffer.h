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

#ifndef __ND_HOST_BUFFER_H__
#define __ND_HOST_BUFFER_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>
#include "cuIntrisics.h"

template<class T>
class cuHostBuffer
{
	public:
	cuHostBuffer();
	~cuHostBuffer();

	ndInt32 GetCount() const;
	void SetCount(ndInt32 count);

	void Clear();
	void Resize(ndInt32 count);
	ndInt32 GetCapacity() const;

	T& operator[] (ndInt32 i);
	const T& operator[] (ndInt32 i) const;

	void Swap(cuHostBuffer& buffer);

	void ReadData(const T* const src, ndInt32 elements);
	void WriteData(T* const dst, ndInt32 elements) const;

	void ReadData(const T* const src, ndInt32 elements, cudaStream_t stream);
	void WriteData(T* const dst, ndInt32 elements, cudaStream_t stream) const;

	T* m_array;
	ndInt32 m_size;
	ndInt32 m_capacity;
};

template<class T>
cuHostBuffer<T>::cuHostBuffer()
	:m_array(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	SetCount(D_GRANULARITY);
	SetCount(0);
}

template<class T>
cuHostBuffer<T>::~cuHostBuffer()
{
	if (m_array)
	{
		cudaError_t cudaStatus = cudaSuccess;
		cudaStatus = cudaFreeHost(m_array);
		dAssert(cudaStatus == cudaSuccess);
		if (cudaStatus != cudaSuccess)
		{
			dAssert(0);
		}
	}
}

template<class T>
const T& cuHostBuffer<T>::operator[] (ndInt32 i) const
{
	dAssert(i >= 0);
	dAssert(i < m_size);
	return m_array[i];
}

template<class T>
T& cuHostBuffer<T>::operator[] (ndInt32 i)
{
	dAssert(i >= 0);
	dAssert(i < m_size);
	return m_array[i];
}

template<class T>
ndInt32 cuHostBuffer<T>::GetCount() const
{
	return m_size;
}

template<class T>
void cuHostBuffer<T>::SetCount(ndInt32 count)
{
	while (count > m_capacity)
	{
		Resize(m_capacity * 2);
	}
	m_size = count;
}

template<class T>
ndInt32 cuHostBuffer<T>::GetCapacity() const
{
	return m_capacity;
}

template<class T>
void cuHostBuffer<T>::Clear()
{
	m_size = 0;
}

template<class T>
void cuHostBuffer<T>::Swap(cuHostBuffer& buffer)
{
	dSwap(m_size, buffer.m_size);
	dSwap(m_array, buffer.m_array);
	dSwap(m_capacity, buffer.m_capacity);
}

template<class T>
void cuHostBuffer<T>::Resize(ndInt32 newSize)
{
	cudaError_t cudaStatus = cudaSuccess;
	if (newSize > m_capacity || (m_capacity == 0))
	{
		T* newArray;
		newSize = dMax(newSize, D_GRANULARITY);
		cudaStatus = cudaMallocHost((void**)&newArray, newSize * sizeof(T));
		dAssert(cudaStatus == cudaSuccess);
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, m_size * sizeof(T), cudaMemcpyDeviceToDevice);
			dAssert(cudaStatus == cudaSuccess);
			cudaStatus = cudaFreeHost(m_array);
			dAssert(cudaStatus == cudaSuccess);
		}
		m_array = newArray;
		m_capacity = newSize;
	}
	else if (newSize < m_capacity)
	{
		T* newArray;
		newSize = dMax(newSize, D_GRANULARITY);
		cudaStatus = cudaMallocHost((void**)&newArray, newSize * sizeof(T));
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, newSize * sizeof(T), cudaMemcpyDeviceToDevice);
			cudaStatus = cudaFreeHost(m_array);
			dAssert(cudaStatus == cudaSuccess);
		}

		m_capacity = newSize;
		m_array = newArray;
	}
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
}

template<class T>
void cuHostBuffer<T>::ReadData(const T* const src, ndInt32 elements)
{
	dAssert(elements <= m_size);
	cudaMemcpy(m_array, src, sizeof (T) * elements, cudaMemcpyHostToDevice);
}

template<class T>
void cuHostBuffer<T>::WriteData(T* const dst, ndInt32 elements) const
{
	dAssert(elements <= m_size);
	cudaMemcpy(dst, m_array, sizeof(T) * elements, cudaMemcpyDeviceToHost);
}

template<class T>
void cuHostBuffer<T>::ReadData(const T* const src, ndInt32 elements, cudaStream_t stream)
{
	dAssert(elements <= m_size);
	cudaError_t cudaStatus = cudaMemcpyAsync(m_array, src, sizeof(T) * elements, cudaMemcpyHostToDevice, stream);
	dAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
}

template<class T>
void cuHostBuffer<T>::WriteData(T* const dst, ndInt32 elements, cudaStream_t stream) const
{
	dAssert(elements <= m_size);
	cudaError_t cudaStatus = cudaMemcpyAsync(dst, m_array, sizeof(T) * elements, cudaMemcpyDeviceToHost, stream);
	dAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
}

#endif