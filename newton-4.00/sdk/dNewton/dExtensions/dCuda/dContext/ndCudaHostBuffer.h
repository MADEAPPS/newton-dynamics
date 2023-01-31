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

#ifndef __ND_CUDA_HOST_BUFFER_H__
#define __ND_CUDA_HOST_BUFFER_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include "ndCudaIntrinsics.h"

//#define D_HOST_SORT_BLOCK_SIZE	(1<<9)
#define D_HOST_SORT_BLOCK_SIZE	(1<<10)

template<class T>
class ndCudaHostBuffer
{
	public:
	ndCudaHostBuffer();
	~ndCudaHostBuffer();

	int GetCount() const;
	void SetCount(int count);

	void Clear();
	void Resize(int count);
	int GetCapacity() const;

	T& operator[] (int i);
	const T& operator[] (int i) const;

	void Swap(ndCudaHostBuffer& buffer);

	void ReadData(const T* const src, int elements);
	void WriteData(T* const dst, int elements) const;

	void ReadData(const T* const src, int elements, cudaStream_t stream);
	void WriteData(T* const dst, int elements, cudaStream_t stream) const;

	T* m_array;
	int m_size;
	int m_capacity;
};

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const ndCudaHostBuffer<T>& src, ndCudaHostBuffer<T>& dst, ndCudaHostBuffer<int>& scansBuffer);

template<class T>
ndCudaHostBuffer<T>::ndCudaHostBuffer()
	:m_array(nullptr)
	,m_size(0)
	,m_capacity(0)
{
	SetCount(D_GRANULARITY);
	SetCount(0);
}

template<class T>
ndCudaHostBuffer<T>::~ndCudaHostBuffer()
{
	if (m_array)
	{
		cudaError_t cudaStatus = cudaSuccess;
		cudaStatus = cudaFreeHost(m_array);
		ndAssert(cudaStatus == cudaSuccess);
		if (cudaStatus != cudaSuccess)
		{
			ndAssert(0);
		}
	}
}

template<class T>
const T& ndCudaHostBuffer<T>::operator[] (int i) const
{
	ndAssert(i >= 0);
	ndAssert(i < m_size);
	return m_array[i];
}

template<class T>
T& ndCudaHostBuffer<T>::operator[] (int i)
{
	ndAssert(i >= 0);
	ndAssert(i < m_size);
	return m_array[i];
}

template<class T>
int ndCudaHostBuffer<T>::GetCount() const
{
	return m_size;
}

template<class T>
void ndCudaHostBuffer<T>::SetCount(int count)
{
	while (count > m_capacity)
	{
		Resize(m_capacity * 2);
	}
	m_size = count;
}

template<class T>
int ndCudaHostBuffer<T>::GetCapacity() const
{
	return m_capacity;
}

template<class T>
void ndCudaHostBuffer<T>::Clear()
{
	m_size = 0;
}

template<class T>
void ndCudaHostBuffer<T>::Swap(ndCudaHostBuffer& buffer)
{
	cuSwap(m_size, buffer.m_size);
	cuSwap(m_array, buffer.m_array);
	cuSwap(m_capacity, buffer.m_capacity);
}

template<class T>
void ndCudaHostBuffer<T>::Resize(int newSize)
{
	cudaError_t cudaStatus = cudaSuccess;
	if (newSize > m_capacity || (m_capacity == 0))
	{
		T* newArray;
		newSize = std::max(newSize, D_GRANULARITY);
		cudaStatus = cudaMallocHost((void**)&newArray, newSize * sizeof(T));
		ndAssert(cudaStatus == cudaSuccess);
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, m_size * sizeof(T), cudaMemcpyDeviceToDevice);
			ndAssert(cudaStatus == cudaSuccess);
			cudaStatus = cudaFreeHost(m_array);
			ndAssert(cudaStatus == cudaSuccess);
		}
		m_array = newArray;
		m_capacity = newSize;
	}
	else if (newSize < m_capacity)
	{
		T* newArray;
		newSize = std::max(newSize, D_GRANULARITY);
		cudaStatus = cudaMallocHost((void**)&newArray, newSize * sizeof(T));
		if (m_array)
		{
			cudaStatus = cudaMemcpy(newArray, m_array, newSize * sizeof(T), cudaMemcpyDeviceToDevice);
			cudaStatus = cudaFreeHost(m_array);
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
void ndCudaHostBuffer<T>::ReadData(const T* const src, int elements)
{
	ndAssert(elements <= m_size);
	cudaMemcpy(m_array, src, sizeof (T) * elements, cudaMemcpyHostToDevice);
}

template<class T>
void ndCudaHostBuffer<T>::WriteData(T* const dst, int elements) const
{
	ndAssert(elements <= m_size);
	cudaMemcpy(dst, m_array, sizeof(T) * elements, cudaMemcpyDeviceToHost);
}

template<class T>
void ndCudaHostBuffer<T>::ReadData(const T* const src, int elements, cudaStream_t stream)
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
void ndCudaHostBuffer<T>::WriteData(T* const dst, int elements, cudaStream_t stream) const
{
	ndAssert(elements <= m_size);
	cudaError_t cudaStatus = cudaMemcpyAsync(dst, m_array, sizeof(T) * elements, cudaMemcpyDeviceToHost, stream);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const ndCudaHostBuffer<T>& src, ndCudaHostBuffer<T>& dst, ndCudaHostBuffer<int>& scansBuffer)
{
	auto AddPrefix = [&](int blockIdx, int blockDim, int computeUnits)
	{
		int sum[D_HOST_SORT_BLOCK_SIZE];
		int offset[D_HOST_SORT_BLOCK_SIZE];
		int localPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + D_HOST_SORT_BLOCK_SIZE + 1];

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			sum[threadId] = 0;
			localPrefixScan[threadId] = 0;
			offset[threadId] = threadId;
		}

		for (int i = 0; i < computeUnits; ++i)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int count = scansBuffer[offset[threadId]];
				scansBuffer[offset[threadId]] = sum[threadId];
				sum[threadId] += count;
				offset[threadId] += blockDim;
			}
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			localPrefixScan[blockDim / 2 + threadId + 1] = sum[threadId];
		}

		for (int i = 1; i < blockDim; i = i << 1)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				sum[threadId] = localPrefixScan[blockDim / 2 + threadId] + localPrefixScan[blockDim / 2 - i + threadId];
			}
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				localPrefixScan[blockDim / 2 + threadId] = sum[threadId];
			}
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			scansBuffer[offset[threadId]] = localPrefixScan[blockDim / 2 + threadId];
		}
	};

	auto CountItems = [&](int blockIndex, int blocksCount)
	{
		int radixCountBuffer[D_HOST_SORT_BLOCK_SIZE];

		int size = src.GetCount();
		int blockStride = D_HOST_SORT_BLOCK_SIZE;
		int bashSize = blocksCount * blockStride * blockIndex;

		ndEvaluateKey evaluator;

		for (int threadId = 0; threadId < blockStride; ++threadId)
		{
			radixCountBuffer[threadId] = 0;
		}

		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int radix = evaluator.GetRadix(src[index]);
					radixCountBuffer[radix] ++;
				}
			}
			bashSize += blockStride;
		}

		for (int threadId = 0; threadId < (1 << exponentRadix); ++threadId)
		{
			int index = threadId + (1 << exponentRadix) * blockIndex;
			scansBuffer[index] = radixCountBuffer[threadId];
		}
	};

	auto MergeBuckects = [&](int blockIdx, int blocksCount, int computeUnits)
	{
		ndEvaluateKey evaluator;
		T cachedItems[D_HOST_SORT_BLOCK_SIZE];
		int sortedRadix[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixCount[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixStart[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixBatchScan[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + D_HOST_SORT_BLOCK_SIZE + 1];

		int size = src.GetCount();
		int radixSize = (1 << exponentRadix);
		int blockDim = D_HOST_SORT_BLOCK_SIZE;
		int radixBase = blockIdx * radixSize;
		int bashSize = blocksCount * blockDim * blockIdx;
		int radixPrefixOffset = computeUnits * radixSize;

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			radixPrefixScan[threadId] = 0;
		}

		for (int threadId = 0; threadId < radixSize; ++threadId)
		{
			radixPrefixStart[threadId] = scansBuffer[radixBase + threadId];
			radixPrefixBatchScan[threadId] = scansBuffer[radixPrefixOffset + threadId];
		}

		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				radixPrefixCount[threadId] = 0;
				sortedRadix[threadId] = (radixSize << 16) + threadId;
			}

			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					cachedItems[threadId] = src[index];
					int radix = evaluator.GetRadix(cachedItems[threadId]);
					radixPrefixCount[radix] ++;
					sortedRadix[threadId] = (radix << 16) + threadId;
				}
			}

			for (int k = 2; k <= blockDim; k = k << 1)
			{
				for (int j = k >> 1; j > 0; j = j >> 1)
				{
					for (int threadId0 = 0; threadId0 < blockDim; ++threadId0)
					{
						int threadId1 = threadId0 ^ j;
						if (threadId1 > threadId0)
						{
							//const int a = sortedRadix[threadId0];
							//const int b = sortedRadix[threadId1];
							//const int mask0 = (-(threadId0 & k)) >> 31;
							//const int mask1 = -(a > b);
							//const int mask2 = mask0 ^ mask1;
							//const int mask3 = -(threadId1 < threadId0);
							//const int a1 = (b & mask3) | (a & ~mask3);
							//const int b1 = (a & mask3) | (b & ~mask3);
							//sortedRadix[threadId0] = (b1 & mask2) | (a1 & ~mask2);



							const int a = sortedRadix[threadId0];
							const int b = sortedRadix[threadId1];
							const int mask0 = (-(threadId0 & k)) >> 31;
							const int mask1 = -(a > b);
							const int mask2 = mask0 ^ mask1;
							//const int a1 = (b & mask2) | (a & ~mask2);
							//const int b1 = (a & mask2) | (b & ~mask2);
							const int a1 = mask2 ? b : a;
							const int b1 = mask2 ? a : b;
							sortedRadix[threadId0] = a1;
							sortedRadix[threadId1] = b1;
						}
					}
				}
			}

			for (int threadId = 0; threadId < radixSize; ++threadId)
			{
				radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId + 1] = radixPrefixCount[threadId];
			}

			for (int k = 1; k < radixSize; k = k << 1)
			{
				int sumReg[D_HOST_SORT_BLOCK_SIZE];
				for (int threadId = 0; threadId < radixSize; ++threadId)
				{
					sumReg[threadId] = radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId] + radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId - k];
				}
				for (int threadId = 0; threadId < radixSize; ++threadId)
				{
					radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + threadId] = sumReg[threadId];
				}
			}

			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyValue = sortedRadix[threadId];
					int keyHigh = keyValue >> 16;
					int keyLow = keyValue & 0xffff;
					int dstOffset1 = radixPrefixBatchScan[keyHigh] + radixPrefixStart[keyHigh];
					int dstOffset0 = threadId - radixPrefixScan[D_HOST_SORT_BLOCK_SIZE / 2 + keyHigh];
					dst[dstOffset0 + dstOffset1] = cachedItems[keyLow];
				}
			}

			for (int threadId = 0; threadId < radixSize; ++threadId)
			{
				radixPrefixStart[threadId] += radixPrefixCount[threadId];
			}

			bashSize += blockDim;
		}
	};

	ndAssert(src.GetCount() == dst.GetCount());

	int deviceComputeUnits = 20;
	int itemCount = src.GetCount();
	int computeUnitsBashCount = (itemCount + D_HOST_SORT_BLOCK_SIZE - 1) / D_HOST_SORT_BLOCK_SIZE;
	int bashCount = (computeUnitsBashCount + deviceComputeUnits - 1) / deviceComputeUnits;
	int computeUnits = (itemCount + bashCount * D_HOST_SORT_BLOCK_SIZE - 1) / (bashCount * D_HOST_SORT_BLOCK_SIZE);
	ndAssert(computeUnits <= deviceComputeUnits);

	for (int block = 0; block < computeUnits; ++block)
	{
		CountItems(block, bashCount);
	}

	for (int block = 0; block < 1; ++block)
	{
		AddPrefix(block, 1 << exponentRadix, computeUnits);
	}

	for (int block = 0; block < computeUnits; ++block)
	{
		MergeBuckects(block, bashCount, computeUnits);
	}
}

#endif