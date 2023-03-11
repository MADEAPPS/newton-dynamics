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
#include "ndCudaUtils.h"
#include "ndCudaIntrinsics.h"

//#define D_HOST_SORT_BLOCK_SIZE	(8)
//#define D_HOST_SORT_BLOCK_SIZE	(16)
#define D_HOST_SORT_BLOCK_SIZE		(1<<8)
//#define D_HOST_SORT_BLOCK_SIZE	(1<<9)
//#define D_HOST_SORT_BLOCK_SIZE	(1<<10)

#define D_HOST_MAX_RADIX_SIZE	(1<<8)
//#define D_HOST_MAX_RADIX_SIZE	(1<<3)

#if D_HOST_MAX_RADIX_SIZE > D_HOST_SORT_BLOCK_SIZE
	#error counting sort diget larger that block
#endif

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
		int sum[D_HOST_MAX_RADIX_SIZE];
		int offset[D_HOST_MAX_RADIX_SIZE];
		int localPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + D_HOST_MAX_RADIX_SIZE + 1];
		
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
		int radixCountBuffer[D_HOST_MAX_RADIX_SIZE];
		
		int size = src.GetCount();
		int radixStride = 1 << exponentRadix;
		int blockStride = D_HOST_SORT_BLOCK_SIZE;
		int bashSize = blocksCount * blockStride * blockIndex;
		
		ndEvaluateKey evaluator;
		for (int threadId = 0; threadId < radixStride; ++threadId)
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
		
		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			int index = threadId + radixStride * blockIndex;
			scansBuffer[index] = radixCountBuffer[threadId];
		}
	};

#define D_USE_BITONIC_MERGE

#ifdef D_USE_BITONIC_MERGE
	auto MergeBuckects = [&](int blockIdx, int blocksCount, int computeUnits)
	{
		T cachedItems[D_HOST_SORT_BLOCK_SIZE];
		int sortedRadix[D_HOST_SORT_BLOCK_SIZE];
		int radixPrefixCount[D_HOST_MAX_RADIX_SIZE];
		int radixPrefixStart[D_HOST_MAX_RADIX_SIZE];
		int radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + D_HOST_MAX_RADIX_SIZE + 1];
		
		int size = src.GetCount();
		int radixStride = (1 << exponentRadix);
		int blockStride = D_HOST_SORT_BLOCK_SIZE;
		int radixBase = blockIdx * radixStride;
		int bashSize = blocksCount * blockStride * blockIdx;
		int radixPrefixOffset = computeUnits * radixStride;

		ndEvaluateKey evaluator;
		radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2] = 0;
		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			radixPrefixScan[(D_HOST_MAX_RADIX_SIZE - radixStride) / 2 + threadId] = 0;
			radixPrefixStart[threadId] = scansBuffer[radixBase + threadId] + scansBuffer[radixPrefixOffset + threadId];
		}

		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixCount[threadId] = 0;
			}

#if 1
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					cachedItems[threadId] = src[index];
					int radix = evaluator.GetRadix(cachedItems[threadId]);
					radixPrefixCount[radix] ++;
					sortedRadix[threadId] = (radix << 16) + threadId;
				}
				else
				{
					ndAssert(0);
					sortedRadix[threadId] = (radixStride << 16) + threadId;
				}
			}
		
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId + 1] = radixPrefixCount[threadId];
			}
		
			for (int k = 1; k < radixStride; k = k << 1)
			{
				int sumReg[D_HOST_MAX_RADIX_SIZE];
				for (int threadId = 0; threadId < radixStride; ++threadId)
				{
					sumReg[threadId] = radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId] + radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId - k];
				}
				for (int threadId = 0; threadId < radixStride; ++threadId)
				{
					radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId] = sumReg[threadId];
				}
			}
		
			for (int k = 2; k <= blockStride; k = k << 1)
			{
				for (int j = k >> 1; j > 0; j = j >> 1)
				{
					for (int threadId0 = 0; threadId0 < blockStride; ++threadId0)
					{
						int threadId1 = threadId0 ^ j;
						if (threadId1 > threadId0)
						{
							const int a = sortedRadix[threadId0];
							const int b = sortedRadix[threadId1];
							const int mask0 = (-(threadId0 & k)) >> 31;
							const int mask1 = -(a > b);
							const int mask2 = mask0 ^ mask1;
							if (mask2)
							{
								sortedRadix[threadId0] = b;
								sortedRadix[threadId1] = a;
							}
						}
					}
				}
			}
		
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyValue = sortedRadix[threadId];
					int keyHigh = keyValue >> 16;
					int keyLow = keyValue & 0xffff;
					int dstOffset1 = radixPrefixStart[keyHigh];
					int dstOffset0 = threadId - radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + keyHigh];
					dst[dstOffset0 + dstOffset1] = cachedItems[keyLow];
				}
			}
#else
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					cachedItems[threadId] = src[index];
					int radix = evaluator.GetRadix(cachedItems[threadId]);
					radixPrefixCount[radix] ++;
					sortedRadix[threadId] = radix;
				}
			}

			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId + 1] = radixPrefixCount[threadId];
			}

			for (int k = 1; k < radixStride; k = k << 1)
			{
				int sumReg[D_HOST_MAX_RADIX_SIZE];
				for (int threadId = 0; threadId < radixStride; ++threadId)
				{
					sumReg[threadId] = radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId] + radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId - k];
				}
				for (int threadId = 0; threadId < radixStride; ++threadId)
				{
					radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + threadId] = sumReg[threadId];
				}
			}

			
			int sortedRadix1[D_HOST_SORT_BLOCK_SIZE + 1];
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int radix = sortedRadix[threadId];
				int address = radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + radix]++;
				sortedRadix1[address] = (radix << 16) + threadId;
			}
			sortedRadix1[D_HOST_SORT_BLOCK_SIZE] = radixStride << 16;

			int passes = 1;
			while (passes)
			{
				passes = 0;
				for (int threadId = 0; threadId < blockStride / 2; ++threadId)
				{
					int id0 = threadId * 2 + 0;
					int id1 = threadId * 2 + 1;
					int key0 = sortedRadix1[id0];
					int key1 = sortedRadix1[id1];
					if (key1 < key0)
					{
						passes = 1;
						sortedRadix1[id0] = key1;
						sortedRadix1[id1] = key0;
					}
				}

				for (int threadId = 0; threadId < blockStride / 2; ++threadId)
				{
					int id0 = threadId * 2 + 1;
					int id1 = threadId * 2 + 2;
					int key0 = sortedRadix1[id0];
					int key1 = sortedRadix1[id1];
					if (key1 < key0)
					{
						passes = 1;
						sortedRadix1[id0] = key1;
						sortedRadix1[id1] = key0;
					}
				}
			}

			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyValue = sortedRadix1[threadId];
					int keyHigh = keyValue >> 16;
					int keyLow = keyValue & 0xffff;
					int dstOffset1 = radixPrefixStart[keyHigh];
					int dstOffset0 = threadId - radixPrefixScan[D_HOST_MAX_RADIX_SIZE / 2 + keyHigh];
					dst[dstOffset0 + dstOffset1] = cachedItems[keyLow];
				}
			}

#endif

			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixStart[threadId] += radixPrefixCount[threadId];
			}

			bashSize += blockStride;
		}
	};

#else

	auto MergeBuckects = [&](int blockIdx, int blocksCount, int computeUnits)
	{
		int scanBaseAdress[D_HOST_MAX_RADIX_SIZE];

		int size = src.GetCount();
		int blockIndex = blockIdx;
		int radixStride = (1 << exponentRadix);
		int blockStride = D_HOST_SORT_BLOCK_SIZE;
		int radixBase = blockIndex * radixStride;
		int radixPrefixOffset = computeUnits * radixStride;
		int bashSize = blocksCount * blockStride * blockIdx;

		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			scanBaseAdress[threadId] = scansBuffer[radixPrefixOffset + threadId] + scansBuffer[radixBase + threadId];
		}

		ndEvaluateKey evaluator;
		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					const T item(src[index]);
					int radix = evaluator.GetRadix(item);
					//int address = atomicAdd(&scanBaseAdress[radix], 1);
					int address = scanBaseAdress[radix]++;
					dst[address] = item;
				}
			}
			bashSize += blockStride;
		}
	};
#endif

	ndAssert(src.GetCount() == dst.GetCount());
	ndAssert((1 << exponentRadix) <= D_HOST_MAX_RADIX_SIZE);

	//int deviceComputeUnits = 20;
	int deviceComputeUnits = 1;
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