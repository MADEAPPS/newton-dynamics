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
#include "ndCudaPrefixScan.cuh"

#define D_HOST_SORTING_ALGORITHM	1

template<class T>
class ndCudaDeviceBuffer;

template<class T>
class ndCudaHostBuffer
{
	public:
	ndCudaHostBuffer();
	ndCudaHostBuffer(const ndCudaHostBuffer<T>& src);
	~ndCudaHostBuffer();

	int GetCount() const;
	void SetCount(int count);

	void Clear();
	void Resize(int count);
	int GetCapacity() const;

	T& operator[] (int i);
	const T& operator[] (int i) const;

	ndCudaHostBuffer& operator= (const ndCudaHostBuffer<T>& src);
	ndCudaHostBuffer& operator= (const ndCudaDeviceBuffer<T>& src);

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
void ndCountingSort(ndCudaHostBuffer<T>& buffer, ndCudaHostBuffer<T>& auxiliaryBuffer, ndCudaHostBuffer<int>& scansBuffer);

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
ndCudaHostBuffer<T>::ndCudaHostBuffer(const ndCudaHostBuffer<T>& src)
	:m_array(SetCount(src.GetCount()))
	,m_size(src.m_size)
	,m_capacity(src.m_capacity)
{
	cudaError_t cudaStatus = cudaSuccess;
	cudaStatus = cudaMemcpy(m_array, src.m_array, m_size * sizeof(T), cudaMemcpyHostToHost);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
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
ndCudaHostBuffer<T>& ndCudaHostBuffer<T>::operator=(const ndCudaHostBuffer<T>& src)
{
	cudaError_t cudaStatus;
	SetCount(src.GetCount());
	cudaStatus = cudaMemcpy(m_array, src.m_array, m_size * sizeof(T), cudaMemcpyHostToHost);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
	return* this;
}

template<class T>
ndCudaHostBuffer<T>& ndCudaHostBuffer<T>::operator=(const ndCudaDeviceBuffer<T>& src)
{
	cudaError_t cudaStatus;
	SetCount(src.GetCount());
	cudaStatus = cudaMemcpy(m_array, src.m_array, m_size * sizeof(T), cudaMemcpyDeviceToHost);
	ndAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
	return*this;
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
			cudaStatus = cudaMemcpy(newArray, m_array, m_size * sizeof(T), cudaMemcpyHostToHost);
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
			ndAssert(0);
			cudaStatus = cudaMemcpy(newArray, m_array, newSize * sizeof(T), cudaMemcpyHostToHost);
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

template <class T, int size>
class ndBankFreeArray
{
	public:
	ndBankFreeArray()
	{
	}

	int GetBankAddress(int address) const
	{
		int low = address & (D_BANK_COUNT_GPU - 1);
		int high = address >> D_LOG_BANK_COUNT_GPU;
		int dst = high * (D_BANK_COUNT_GPU + 1) + low;
		return dst;
	}

	T& operator[] (int address)
	{
		int index = GetBankAddress(address);
		return m_array[index];
	}

	T m_array[((size + D_BANK_COUNT_GPU - 1) >> D_LOG_BANK_COUNT_GPU) * (D_BANK_COUNT_GPU + 1)];
};

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(ndCudaHostBuffer<T>& buffer, ndCudaHostBuffer<T>& auxiliaryBuffer, ndCudaHostBuffer<int>& scansBuffer)
{
#if 1
	//optimized Hillis-Steele prefix scan sum
	auto AddPrefix = [&](int blockIdx, int blockDim, int computeUnits)
	{
		int localPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

		auto ShuffleUp = [&](const int* in, int* out, int offset)
		{
			//for (int i = 0; i < offset; ++i)
			for (int i = 0; i < D_BANK_COUNT_GPU; ++i)
			{
				out[i] = in[i];
			}

			for (int i = D_BANK_COUNT_GPU - offset - 1; i >= 0; --i)
			{
				out[i + offset] = in[i];
			}
		};

		int sumReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int offsetReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			sumReg[threadId] = 0;
			offsetReg[threadId] = threadId;
		}
		localPrefixScan[0] = 0;

		for (int i = 0; i < computeUnits; ++i)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int count = scansBuffer[offsetReg[threadId]];
				scansBuffer[offsetReg[threadId]] = sumReg[threadId];
				sumReg[threadId] += count;
				offsetReg[threadId] += blockDim;
			}
		}

		int memoryTransactions = 0;
		for (int bankBase = 0; bankBase < blockDim; bankBase += D_BANK_COUNT_GPU)
		{
			for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
			{
				int sumTempReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
				ShuffleUp(&sumReg[bankBase], &sumTempReg[bankBase], n);
				for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
				{
					int laneId = threadId & (D_BANK_COUNT_GPU - 1);
					if (laneId >= n)
					{
						sumReg[bankBase + threadId] += sumTempReg[bankBase + threadId];
					}
				}
			}
		}
		
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			if (!(threadId & D_BANK_COUNT_GPU))
			{
				localPrefixScan[threadId + 1] = sumReg[threadId];
				memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
			}
		}

		int scale = 0;
		for (int segment = blockDim; segment > D_BANK_COUNT_GPU; segment >>= 1)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
				int warpBase = threadId & bank;
				if (warpBase)
				{
					int warpSumIndex = threadId & (-warpBase);
					sumReg[threadId] += localPrefixScan[warpSumIndex - 1 + 1];
					localPrefixScan[threadId + 1] = sumReg[threadId];

					memoryTransactions += 2 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
				}
			}
			scale++;
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			scansBuffer[offsetReg[threadId]] = localPrefixScan[threadId];
		}
	};

#else
	//implementation of the Blelloch scan, not better than optimized Hillis-Steele prefix scan sum
	//actually three time slower,
	//however this is still good for porting to hardware without shuffle instructions
	auto AddPrefix = [&](int blockIdx, int blockDim, int computeUnits)
	{
		int sumReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int offsetReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		//int localPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];
		ndBankFreeArray<T, D_DEVICE_SORT_MAX_RADIX_SIZE + 1> localPrefixScan;

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			sumReg[threadId] = 0;
			offsetReg[threadId] = threadId;
		}

		for (int i = 0; i < computeUnits; ++i)
		{
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				int count = scansBuffer[offsetReg[threadId]];
				scansBuffer[offsetReg[threadId]] = sumReg[threadId];
				sumReg[threadId] += count;
				offsetReg[threadId] += blockDim;
			}
		}

		int memoryTransactions = 0;
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			localPrefixScan[threadId] = sumReg[threadId];
			memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
		}

		int radixBit = 0;
		for (int k = 1; k < blockDim; k = k * 2)
		{
			radixBit++;
			for (int threadId = 0; threadId < (blockDim >> radixBit); threadId++)
			{
				int id1 = ((threadId + 1) << radixBit) - 1;
				int id0 = id1 - (1 << (radixBit - 1));
				//cuTrace(("%d ", localPrefixScan.GetBankAddress(id0) % D_BANK_COUNT_GPU));
				int a = localPrefixScan[id0];
				int b = localPrefixScan[id1];
				localPrefixScan[id1] = a + b;

				memoryTransactions += 3 * ((threadId & (D_BANK_COUNT_GPU-1)) == 1);
			}
			//cuTrace(("\n"));
		}
		//cuTrace(("\n"));

		localPrefixScan[blockDim] = localPrefixScan[blockDim - 1];
		localPrefixScan[blockDim - 1] = 0;
		for (int k = 1; k < blockDim; k = k * 2)
		{
			for (int threadId = 0; threadId < k; threadId++)
			{
				int id1 = ((threadId + 1) << radixBit) - 1;
				int id0 = id1 - (1 << (radixBit - 1));
				int a = localPrefixScan[id1];
				int b = localPrefixScan[id0] + a;
		
				localPrefixScan[id0] = a;
				localPrefixScan[id1] = b;

				memoryTransactions += 4 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
			}
			radixBit--;
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			scansBuffer[offsetReg[threadId]] = localPrefixScan[threadId];
		}
	};
#endif

#if (D_HOST_SORTING_ALGORITHM == 0)
	//optimized Hillis-Steele prefix scan sum
	//using a simple bitonic sort with two ways bank conflict
	auto CountAndSortBlockItems = [&](int blockIndex, int blocksCount)
	{
		T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
		int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
		int radixCountBuffer[D_DEVICE_SORT_MAX_RADIX_SIZE];

		int size = buffer.GetCount();
		int radixStride = 1 << exponentRadix;
		int blockStride = D_DEVICE_SORT_BLOCK_SIZE;
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
				int sortedRadixReg[D_DEVICE_SORT_BLOCK_SIZE];
				int index = bashSize + threadId;
				sortedRadixReg[threadId] = (radixStride << 16);
				if (index < size)
				{
					const T item(buffer[index]);
					cachedItems[threadId] = item;
					int radix = evaluator.GetRadix(item);
					radixCountBuffer[radix] ++;
					sortedRadixReg[threadId] = (radix << 16) + threadId;
				}
				sortedRadix[threadId] = sortedRadixReg[threadId];
			}

			int memoryTransactions = 0;
			for (int k = 1; k < blockStride; k = k << 1)
			{
				for (int j = k; j > 0; j = j >> 1)
				{
					int highMask = -j;
					int lowMask = ~highMask;
					for (int threadId = 0; threadId < blockStride / 2; ++threadId)
					{
						int lowIndex = threadId & lowMask;
						int highIndex = (threadId & highMask) * 2;

						int id0 = highIndex + lowIndex;
						int id1 = highIndex + lowIndex + j;
						int oddEven = highIndex & k * 2;
						//cuTrace(("%d ", id0 % D_BANK_COUNT));

						int a = sortedRadix[id0];
						int b = sortedRadix[id1];

						int test = a < b;
						int a1 = test ? a : b;
						int b1 = test ? b : a;

						int a2 = oddEven ? b1 : a1;
						int b2 = oddEven ? a1 : b1;

						sortedRadix[id0] = a2;
						sortedRadix[id1] = b2;

						memoryTransactions += 4 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
					}
					//cuTrace(("\n"));
				}
				//cuTrace(("\n"));
			}

			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyIndex = sortedRadix[threadId] & 0xffff;
					auxiliaryBuffer[index] = cachedItems[keyIndex];
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

	auto MergeBuckects = [&](int blockIdx, int blocksCount, int computeUnits)
	{
		//T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
		int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int radixPrefixStart[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int radixPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

		auto ShuffleUp = [&](const int* in, int* out, int offset)
		{
			//for (int i = 0; i < offset; ++i)
			for (int i = 0; i < D_BANK_COUNT_GPU; ++i)
			{
				out[i] = in[i];
			}

			for (int i = D_BANK_COUNT_GPU - offset - 1; i >= 0; --i)
			{
				out[i + offset] = in[i];
			}
		};

		int size = buffer.GetCount();
		int radixStride = (1 << exponentRadix);
		int blockStride = D_DEVICE_SORT_BLOCK_SIZE;
		int radixBase = blockIdx * radixStride;
		int bashSize = blocksCount * blockStride * blockIdx;
		int radixPrefixOffset = computeUnits * radixStride;

		ndEvaluateKey evaluator;

		int radixPrefixStartReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			int a = scansBuffer[radixBase + threadId];
			int b = scansBuffer[radixPrefixOffset + threadId];
			radixPrefixStartReg[threadId] = a + b;
			radixPrefixStart[threadId] = radixPrefixStartReg[threadId];
		}

		radixPrefixScan[0] = 0;
		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixCount[threadId] = 0;
			}

			T cachedItemsReg[D_DEVICE_SORT_BLOCK_SIZE];
			int sortedRadixReg[D_DEVICE_SORT_BLOCK_SIZE];
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				int radix = radixStride - 1;
				sortedRadixReg[threadId] = radix;
				if (index < size)
				{
					const T item(auxiliaryBuffer[index]);
					//cachedItems[threadId] = item;
					cachedItemsReg[threadId] = item;
					radix = evaluator.GetRadix(item);
					sortedRadixReg[threadId] = radix;
				}
				radixPrefixCount[radix] ++;
			}

			int memoryTransactions = 0;

			int radixPrefixScanReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
			int radixPrefixCountReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixCountReg[threadId] = radixPrefixCount[threadId];
				radixPrefixScanReg[threadId] = radixPrefixCountReg[threadId];
				memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
			}

			for (int bankBase = 0; bankBase < radixStride; bankBase += D_BANK_COUNT_GPU)
			{
				for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
				{
					int radixPrefixScanRegTemp[D_DEVICE_SORT_MAX_RADIX_SIZE];
					ShuffleUp(&radixPrefixScanReg[bankBase], &radixPrefixScanRegTemp[bankBase], n);
					for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
					{
						if ((threadId & (D_BANK_COUNT_GPU - 1)) >= n)
						{
							radixPrefixScanReg[bankBase + threadId] += radixPrefixScanRegTemp[bankBase + threadId];
						}
					}
				}
			}

			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				if (!(threadId & D_BANK_COUNT_GPU))
				{
					radixPrefixScan[threadId + 1] = radixPrefixScanReg[threadId];
					memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
				}
			}

			int scale = 0;
			for (int segment = radixStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
			{
				for (int threadId = 0; threadId < radixStride; ++threadId)
				{
					int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
					int warpBase = threadId & bank;
					if (warpBase)
					{
						int warpSumIndex = threadId & (-warpBase);
						radixPrefixScanReg[threadId] += radixPrefixScan[warpSumIndex];
						radixPrefixScan[threadId + 1] = radixPrefixScanReg[threadId];
						memoryTransactions += 2 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
					}
				}
				scale++;
			}

			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyLow = sortedRadixReg[threadId];
					int dstOffset1 = radixPrefixStart[keyLow];
					int dstOffset0 = threadId - radixPrefixScan[keyLow];
					//buffer[dstOffset0 + dstOffset1] = cachedItems[threadId];
					buffer[dstOffset0 + dstOffset1] = cachedItemsReg[threadId];
				}
			}

			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixStartReg[threadId] += radixPrefixCountReg[threadId];
				radixPrefixStart[threadId] = radixPrefixStartReg[threadId];
			}

			bashSize += blockStride;
		}
	};

#elif (D_HOST_SORTING_ALGORITHM == 1)
	//optimized Hillis-Steele prefix scan sum
	//using a two bits counting sort
	auto CountAndSortBlockItems = [&](int blockIndex, int blocksCount)
	{
		T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
		int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
		int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1)];

		auto ShuffleUp = [&](const int* in, int* out, int offset)
		{
			//for (int i = 0; i < offset; ++i)
			for (int i = 0; i < D_BANK_COUNT_GPU; ++i)
			{
				out[i] = in[i];
			}

			for (int i = D_BANK_COUNT_GPU - offset - 1; i >= 0; --i)
			{
				out[i + offset] = in[i];
			}
		};

		int size = buffer.GetCount();
		int radixStride = 1 << exponentRadix;
		int blockStride = D_DEVICE_SORT_BLOCK_SIZE;
		int bashSize = blocksCount * blockStride * blockIndex;

		ndEvaluateKey evaluator;
		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			radixPrefixCount[threadId] = 0;
		}

		radixPrefixScan[0] = 0;
		radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1] = 0;

		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				int radix = radixStride - 1;
				int sortKey = radix;
				if (index < size)
				{
					const T item(buffer[index]);
					cachedItems[threadId] = item;
					radix = evaluator.GetRadix(item);
					sortKey = (threadId << 16) + radix;
				}
				radixPrefixCount[radix] ++;
				sortedRadix[threadId] = sortKey;
			}

			int memoryTransactions = 0;
			for (int bit = 0; (1 << (bit * 2)) < radixStride; ++bit)
			{
				int keyReg[D_DEVICE_SORT_BLOCK_SIZE];
				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					keyReg[threadId] = sortedRadix[threadId];
					memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
				}

				int dstLocalOffsetReg[D_DEVICE_SORT_BLOCK_SIZE];
				int radixPrefixScanReg0[D_DEVICE_SORT_BLOCK_SIZE];
				int radixPrefixScanReg1[D_DEVICE_SORT_BLOCK_SIZE];
				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					int test = (keyReg[threadId] >> (bit * 2)) & 0x3;
					dstLocalOffsetReg[threadId] = test;
					int bit0 = (test == 0) ? 1 : 0;
					int bit1 = (test == 1) ? 1 << 16 : 0;
					int bit2 = (test == 2) ? 1 : 0;
					int bit3 = (test == 3) ? 1 << 16 : 0;
					radixPrefixScanReg0[threadId] = bit0 + bit1;
					radixPrefixScanReg1[threadId] = bit2 + bit3;
				}

				for (int bankBase = 0; bankBase < blockStride; bankBase += D_BANK_COUNT_GPU)
				{
					for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
					{
						int radixPrefixScanRegTemp0[D_DEVICE_SORT_BLOCK_SIZE];
						int radixPrefixScanRegTemp1[D_DEVICE_SORT_BLOCK_SIZE];
						ShuffleUp(&radixPrefixScanReg0[bankBase], &radixPrefixScanRegTemp0[bankBase], n);
						ShuffleUp(&radixPrefixScanReg1[bankBase], &radixPrefixScanRegTemp1[bankBase], n);
						for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
						{
							if ((threadId & (D_BANK_COUNT_GPU - 1)) >= n)
							{
								radixPrefixScanReg0[bankBase + threadId] += radixPrefixScanRegTemp0[bankBase + threadId];
								radixPrefixScanReg1[bankBase + threadId] += radixPrefixScanRegTemp1[bankBase + threadId];
							}
						}
					}
				}

				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					if (!(threadId & D_BANK_COUNT_GPU))
					{
						radixPrefixScan[threadId + 1] = radixPrefixScanReg0[threadId];
						radixPrefixScan[threadId + 1 + D_DEVICE_SORT_BLOCK_SIZE + 1] = radixPrefixScanReg1[threadId];
						memoryTransactions += 2 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
					}
				}

				int scale = 0;
				for (int segment = blockStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
				{
					for (int threadId = 0; threadId < blockStride; ++threadId)
					{
						int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
						int warpBase = threadId & bank;
						if (warpBase)
						{
							int warpSumIndex = threadId & (-warpBase);

							radixPrefixScanReg0[threadId] += radixPrefixScan[warpSumIndex - 1 + 1];
							radixPrefixScan[threadId + 1] = radixPrefixScanReg0[threadId];

							radixPrefixScanReg1[threadId] += radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + warpSumIndex - 1 + 1];
							radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + threadId + 1] = radixPrefixScanReg1[threadId];

							memoryTransactions += 4 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
						}
					}
					scale++;
				}

				int sum0 = radixPrefixScan[1 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
				int sum1 = radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
				int base0 = 0;
				int base1 = sum0 & 0xffff;
				int base2 = base1 + (sum0 >> 16);
				int base3 = base2 + (sum1 & 0xffff);

				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					int key0 = radixPrefixScan[threadId];
					int key1 = radixPrefixScan[threadId + D_DEVICE_SORT_BLOCK_SIZE + 1];
					int shift = dstLocalOffsetReg[threadId];

					int dstIndex = 0;
					dstIndex += (shift == 1) ? base1 + (key0 >> 16) : 0;
					dstIndex += (shift == 3) ? base3 + (key1 >> 16) : 0;
					dstIndex += (shift == 0) ? base0 + (key0 & 0xffff) : 0;
					dstIndex += (shift == 2) ? base2 + (key1 & 0xffff) : 0;

					ndAssert(dstIndex >= 0);
					ndAssert(dstIndex < blockStride);
					sortedRadix[dstIndex] = keyReg[threadId];
				}
			}

			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyIndex = sortedRadix[threadId] >> 16;
					auxiliaryBuffer[index] = cachedItems[keyIndex];
				}
			}

			bashSize += blockStride;
		}

		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			int index = threadId + radixStride * blockIndex;
			scansBuffer[index] = radixPrefixCount[threadId];
		}
	};

	auto MergeBuckects = [&](int blockIdx, int blocksCount, int computeUnits)
	{
		int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int radixPrefixStart[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int radixPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

		auto ShuffleUp = [&](const int* in, int* out, int offset)
		{
			//for (int i = 0; i < offset; ++i)
			for (int i = 0; i < D_BANK_COUNT_GPU; ++i)
			{
				out[i] = in[i];
			}

			for (int i = D_BANK_COUNT_GPU - offset - 1; i >= 0; --i)
			{
				out[i + offset] = in[i];
			}
		};

		int size = buffer.GetCount();
		int radixStride = (1 << exponentRadix);
		int blockStride = D_DEVICE_SORT_BLOCK_SIZE;
		int radixBase = blockIdx * radixStride;
		int bashSize = blocksCount * blockStride * blockIdx;
		int radixPrefixOffset = computeUnits * radixStride;

		ndEvaluateKey evaluator;
		int radixPrefixStartReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			int a = scansBuffer[radixBase + threadId];
			int b = scansBuffer[radixPrefixOffset + threadId];
			radixPrefixStartReg[threadId] = a + b;
			radixPrefixStart[threadId] = radixPrefixStartReg[threadId];
		}

		radixPrefixScan[0] = 0;
		for (int i = 0; i < blocksCount; ++i)
		{
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixCount[threadId] = 0;
			}

			T cachedItemsReg[D_DEVICE_SORT_BLOCK_SIZE];
			int sortedRadixReg[D_DEVICE_SORT_BLOCK_SIZE];
			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				int radix = radixStride - 1;
				sortedRadixReg[threadId] = radix;
				if (index < size)
				{
					const T item(auxiliaryBuffer[index]);
					cachedItemsReg[threadId] = item;
					radix = evaluator.GetRadix(item);
					sortedRadixReg[threadId] = radix;
				}
				radixPrefixCount[radix] ++;
			}

			int memoryTransactions = 0;

			int radixPrefixScanReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
			int radixPrefixCountReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixCountReg[threadId] = radixPrefixCount[threadId];
				radixPrefixScanReg[threadId] = radixPrefixCountReg[threadId];
				memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
			}

			for (int bankBase = 0; bankBase < radixStride; bankBase += D_BANK_COUNT_GPU)
			{
				for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
				{
					int radixPrefixScanRegTemp[D_DEVICE_SORT_MAX_RADIX_SIZE];
					ShuffleUp(&radixPrefixScanReg[bankBase], &radixPrefixScanRegTemp[bankBase], n);
					for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
					{
						if ((threadId & (D_BANK_COUNT_GPU - 1)) >= n)
						{
							radixPrefixScanReg[bankBase + threadId] += radixPrefixScanRegTemp[bankBase + threadId];
						}
					}
				}
			}

			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				if (!(threadId & D_BANK_COUNT_GPU))
				{
					radixPrefixScan[threadId + 1] = radixPrefixScanReg[threadId];
					memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
				}
			}

			int scale = 0;
			for (int segment = radixStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
			{
				for (int threadId = 0; threadId < radixStride; ++threadId)
				{
					int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
					int warpBase = threadId & bank;
					if (warpBase)
					{
						int warpSumIndex = threadId & (-warpBase);
						radixPrefixScanReg[threadId] += radixPrefixScan[warpSumIndex];
						radixPrefixScan[threadId + 1] = radixPrefixScanReg[threadId];
						memoryTransactions += 2 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
					}
				}
				scale++;
			}

			for (int threadId = 0; threadId < blockStride; ++threadId)
			{
				int index = bashSize + threadId;
				if (index < size)
				{
					int keyLow = sortedRadixReg[threadId];
					int dstOffset1 = radixPrefixStart[keyLow];
					int dstOffset0 = threadId - radixPrefixScan[keyLow];
					buffer[dstOffset0 + dstOffset1] = cachedItemsReg[threadId];
				}
			}
			
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				radixPrefixStartReg[threadId] += radixPrefixCountReg[threadId];
				radixPrefixStart[threadId] = radixPrefixStartReg[threadId];
			}

			bashSize += blockStride;
		}
	};

#elif (D_HOST_SORTING_ALGORITHM == 1)
//optimized Hillis-Steele prefix scan sum
//using a two bits counting sort
auto CountAndSortBlockItems = [&](int blockIndex, int blocksCount)
{
	T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
	int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
	int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
	int radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1)];
	int skipDigit[D_BANK_COUNT_GPU];

	auto ShuffleUp = [&](const int* in, int* out, int offset)
	{
		//for (int i = 0; i < offset; ++i)
		for (int i = 0; i < D_BANK_COUNT_GPU; ++i)
		{
			out[i] = in[i];
		}

		for (int i = D_BANK_COUNT_GPU - offset - 1; i >= 0; --i)
		{
			out[i + offset] = in[i];
		}
	};

	auto ShuffleDown = [&](const int* in, int* out, int offset)
	{
		for (int i = 0; i < D_BANK_COUNT_GPU; ++i)
		{
			out[i] = in[i];
		}

		for (int i = D_BANK_COUNT_GPU - offset - 1; i >= 0; --i)
		{
			out[i] = in[i + offset];
		}
	};

	int size = buffer.GetCount();
	int radixStride = 1 << exponentRadix;
	int blockStride = D_DEVICE_SORT_BLOCK_SIZE;
	int bashSize = blocksCount * blockStride * blockIndex;

	ndEvaluateKey evaluator;
	for (int threadId = 0; threadId < radixStride; ++threadId)
	{
		radixPrefixCount[threadId] = 0;
	}

	radixPrefixScan[0] = 0;
	radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1] = 0;

	for (int i = 0; i < blocksCount; ++i)
	{
		for (int threadId = 0; threadId < blockStride; ++threadId)
		{
			int index = bashSize + threadId;
			int radix = radixStride - 1;
			int sortKey = radix;
			if (index < size)
			{
				const T item(buffer[index]);
				cachedItems[threadId] = item;
				radix = evaluator.GetRadix(item);
				sortKey = (threadId << 16) + radix;
			}
			radixPrefixCount[radix] ++;
			sortedRadix[threadId] = sortKey;
		}

#if 1
		int keyTest = sortedRadix[0] & 0xff;
		int sortedRadixReg[D_DEVICE_SORT_BLOCK_SIZE];
		for (int threadId = 0; threadId < blockStride; ++threadId)
		{
			sortedRadixReg[threadId] = (~(sortedRadix[threadId] ^ keyTest)) & 0xff;
		}
		for (int bankBase = 0; bankBase < blockStride; bankBase += D_BANK_COUNT_GPU)
		{
			for (int n = D_BANK_COUNT_GPU / 2; n; n = n / 2)
			{
				int radixPrefixScanRegTemp[D_DEVICE_SORT_BLOCK_SIZE];
				ShuffleDown(&sortedRadixReg[bankBase], &radixPrefixScanRegTemp[bankBase], n);
				for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
				{
					sortedRadixReg[bankBase + threadId] = sortedRadixReg[bankBase + threadId] & radixPrefixScanRegTemp[bankBase + threadId];
				}
			}
		}
		for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
		{
			skipDigit[threadId] = 0xff;
		}
		for (int threadId = 0; threadId < blockStride; threadId += D_BANK_COUNT_GPU)
		{
			skipDigit[threadId >> D_LOG_BANK_COUNT_GPU] = sortedRadixReg[threadId];
		}
		int skipDigitReg[D_BANK_COUNT_GPU];
		for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
		{
			skipDigitReg[threadId] = skipDigit[threadId];
		}
		for (int n = D_BANK_COUNT_GPU / 2; n; n = n / 2)
		{
			int radixPrefixScanRegTemp[D_DEVICE_SORT_BLOCK_SIZE];
			ShuffleDown(&skipDigitReg[0], &radixPrefixScanRegTemp[0], n);
			for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
			{
				skipDigitReg[threadId] = skipDigitReg[threadId] & radixPrefixScanRegTemp[threadId];
			}
		}
#endif

		keyTest = skipDigitReg[0];
		int memoryTransactions = 0;
		for (int bit = 0; (1 << (bit * 2)) < radixStride; ++bit)
		{
			if ((keyTest & 0x03) != 0x03)
			{
				int keyReg[D_DEVICE_SORT_BLOCK_SIZE];
				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					keyReg[threadId] = sortedRadix[threadId];
					memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
				}

				int dstLocalOffsetReg[D_DEVICE_SORT_BLOCK_SIZE];
				int radixPrefixScanReg0[D_DEVICE_SORT_BLOCK_SIZE];
				int radixPrefixScanReg1[D_DEVICE_SORT_BLOCK_SIZE];
				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					int test = (keyReg[threadId] >> (bit * 2)) & 0x3;
					dstLocalOffsetReg[threadId] = test;
					int bit0 = (test == 0) ? 1 : 0;
					int bit1 = (test == 1) ? 1 << 16 : 0;
					int bit2 = (test == 2) ? 1 : 0;
					int bit3 = (test == 3) ? 1 << 16 : 0;
					radixPrefixScanReg0[threadId] = bit0 + bit1;
					radixPrefixScanReg1[threadId] = bit2 + bit3;
				}

				for (int bankBase = 0; bankBase < blockStride; bankBase += D_BANK_COUNT_GPU)
				{
					for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
					{
						int radixPrefixScanRegTemp0[D_DEVICE_SORT_BLOCK_SIZE];
						int radixPrefixScanRegTemp1[D_DEVICE_SORT_BLOCK_SIZE];
						ShuffleUp(&radixPrefixScanReg0[bankBase], &radixPrefixScanRegTemp0[bankBase], n);
						ShuffleUp(&radixPrefixScanReg1[bankBase], &radixPrefixScanRegTemp1[bankBase], n);
						for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
						{
							if ((threadId & (D_BANK_COUNT_GPU - 1)) >= n)
							{
								radixPrefixScanReg0[bankBase + threadId] += radixPrefixScanRegTemp0[bankBase + threadId];
								radixPrefixScanReg1[bankBase + threadId] += radixPrefixScanRegTemp1[bankBase + threadId];
							}
						}
					}
				}

				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					if (!(threadId & D_BANK_COUNT_GPU))
					{
						radixPrefixScan[threadId + 1] = radixPrefixScanReg0[threadId];
						radixPrefixScan[threadId + 1 + D_DEVICE_SORT_BLOCK_SIZE + 1] = radixPrefixScanReg1[threadId];
						memoryTransactions += 2 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
					}
				}

				int scale = 0;
				for (int segment = blockStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
				{
					for (int threadId = 0; threadId < blockStride; ++threadId)
					{
						int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
						int warpBase = threadId & bank;
						if (warpBase)
						{
							int warpSumIndex = threadId & (-warpBase);

							radixPrefixScanReg0[threadId] += radixPrefixScan[warpSumIndex - 1 + 1];
							radixPrefixScan[threadId + 1] = radixPrefixScanReg0[threadId];

							radixPrefixScanReg1[threadId] += radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + warpSumIndex - 1 + 1];
							radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + threadId + 1] = radixPrefixScanReg1[threadId];

							memoryTransactions += 4 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
						}
					}
					scale++;
				}

				int sum0 = radixPrefixScan[1 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
				int sum1 = radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
				int base0 = 0;
				int base1 = sum0 & 0xffff;
				int base2 = base1 + (sum0 >> 16);
				int base3 = base2 + (sum1 & 0xffff);

				for (int threadId = 0; threadId < blockStride; ++threadId)
				{
					int key0 = radixPrefixScan[threadId];
					int key1 = radixPrefixScan[threadId + D_DEVICE_SORT_BLOCK_SIZE + 1];
					int shift = dstLocalOffsetReg[threadId];

					int dstIndex = 0;
					dstIndex += (shift == 1) ? base1 + (key0 >> 16) : 0;
					dstIndex += (shift == 3) ? base3 + (key1 >> 16) : 0;
					dstIndex += (shift == 0) ? base0 + (key0 & 0xffff) : 0;
					dstIndex += (shift == 2) ? base2 + (key1 & 0xffff) : 0;

					ndAssert(dstIndex >= 0);
					ndAssert(dstIndex < blockStride);
					sortedRadix[dstIndex] = keyReg[threadId];
				}
			}
			keyTest >>= 2;
		}

		for (int threadId = 0; threadId < blockStride; ++threadId)
		{
			int index = bashSize + threadId;
			if (index < size)
			{
				int keyIndex = sortedRadix[threadId] >> 16;
				auxiliaryBuffer[index] = cachedItems[keyIndex];
			}
		}

		bashSize += blockStride;
	}

	for (int threadId = 0; threadId < radixStride; ++threadId)
	{
		int index = threadId + radixStride * blockIndex;
		scansBuffer[index] = radixPrefixCount[threadId];
	}
};

auto MergeBuckects = [&](int blockIdx, int blocksCount, int computeUnits)
{
	int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
	int radixPrefixStart[D_DEVICE_SORT_MAX_RADIX_SIZE];
	int radixPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

	auto ShuffleUp = [&](const int* in, int* out, int offset)
	{
		//for (int i = 0; i < offset; ++i)
		for (int i = 0; i < D_BANK_COUNT_GPU; ++i)
		{
			out[i] = in[i];
		}

		for (int i = D_BANK_COUNT_GPU - offset - 1; i >= 0; --i)
		{
			out[i + offset] = in[i];
		}
	};

	int size = buffer.GetCount();
	int radixStride = (1 << exponentRadix);
	int blockStride = D_DEVICE_SORT_BLOCK_SIZE;
	int radixBase = blockIdx * radixStride;
	int bashSize = blocksCount * blockStride * blockIdx;
	int radixPrefixOffset = computeUnits * radixStride;

	ndEvaluateKey evaluator;
	int radixPrefixStartReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
	for (int threadId = 0; threadId < radixStride; ++threadId)
	{
		int a = scansBuffer[radixBase + threadId];
		int b = scansBuffer[radixPrefixOffset + threadId];
		radixPrefixStartReg[threadId] = a + b;
		radixPrefixStart[threadId] = radixPrefixStartReg[threadId];
	}

	radixPrefixScan[0] = 0;
	for (int i = 0; i < blocksCount; ++i)
	{
		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			radixPrefixCount[threadId] = 0;
		}

		T cachedItemsReg[D_DEVICE_SORT_BLOCK_SIZE];
		int sortedRadixReg[D_DEVICE_SORT_BLOCK_SIZE];
		for (int threadId = 0; threadId < blockStride; ++threadId)
		{
			int index = bashSize + threadId;
			int radix = radixStride - 1;
			sortedRadixReg[threadId] = radix;
			if (index < size)
			{
				const T item(auxiliaryBuffer[index]);
				cachedItemsReg[threadId] = item;
				radix = evaluator.GetRadix(item);
				sortedRadixReg[threadId] = radix;
			}
			radixPrefixCount[radix] ++;
		}

		int memoryTransactions = 0;

		int radixPrefixScanReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		int radixPrefixCountReg[D_DEVICE_SORT_MAX_RADIX_SIZE];
		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			radixPrefixCountReg[threadId] = radixPrefixCount[threadId];
			radixPrefixScanReg[threadId] = radixPrefixCountReg[threadId];
			memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
		}

		for (int bankBase = 0; bankBase < radixStride; bankBase += D_BANK_COUNT_GPU)
		{
			for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
			{
				int radixPrefixScanRegTemp[D_DEVICE_SORT_MAX_RADIX_SIZE];
				ShuffleUp(&radixPrefixScanReg[bankBase], &radixPrefixScanRegTemp[bankBase], n);
				for (int threadId = 0; threadId < D_BANK_COUNT_GPU; ++threadId)
				{
					if ((threadId & (D_BANK_COUNT_GPU - 1)) >= n)
					{
						radixPrefixScanReg[bankBase + threadId] += radixPrefixScanRegTemp[bankBase + threadId];
					}
				}
			}
		}

		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			if (!(threadId & D_BANK_COUNT_GPU))
			{
				radixPrefixScan[threadId + 1] = radixPrefixScanReg[threadId];
				memoryTransactions += 1 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
			}
		}

		int scale = 0;
		for (int segment = radixStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
		{
			for (int threadId = 0; threadId < radixStride; ++threadId)
			{
				int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
				int warpBase = threadId & bank;
				if (warpBase)
				{
					int warpSumIndex = threadId & (-warpBase);
					radixPrefixScanReg[threadId] += radixPrefixScan[warpSumIndex];
					radixPrefixScan[threadId + 1] = radixPrefixScanReg[threadId];
					memoryTransactions += 2 * ((threadId & (D_BANK_COUNT_GPU - 1)) == 1);
				}
			}
			scale++;
		}

		for (int threadId = 0; threadId < blockStride; ++threadId)
		{
			int index = bashSize + threadId;
			if (index < size)
			{
				int keyLow = sortedRadixReg[threadId];
				int dstOffset1 = radixPrefixStart[keyLow];
				int dstOffset0 = threadId - radixPrefixScan[keyLow];
				buffer[dstOffset0 + dstOffset1] = cachedItemsReg[threadId];
			}
		}

		for (int threadId = 0; threadId < radixStride; ++threadId)
		{
			radixPrefixStartReg[threadId] += radixPrefixCountReg[threadId];
			radixPrefixStart[threadId] = radixPrefixStartReg[threadId];
		}

		bashSize += blockStride;
	}
};

#else
	#error implement new local sort and merge algorthm?
#endif

	ndAssert(buffer.GetCount() == auxiliaryBuffer.GetCount());
	ndAssert((1 << exponentRadix) <= D_DEVICE_SORT_MAX_RADIX_SIZE);

	//int deviceComputeUnits = 20;
	int deviceComputeUnits = 1;
	int itemCount = buffer.GetCount();
	int computeUnitsBashCount = (itemCount + D_DEVICE_SORT_BLOCK_SIZE - 1) / D_DEVICE_SORT_BLOCK_SIZE;
	int bashCount = (computeUnitsBashCount + deviceComputeUnits - 1) / deviceComputeUnits;
	int computeUnits = (itemCount + bashCount * D_DEVICE_SORT_BLOCK_SIZE - 1) / (bashCount * D_DEVICE_SORT_BLOCK_SIZE);
	ndAssert(computeUnits <= deviceComputeUnits);

	for (int block = 0; block < computeUnits; ++block)
	{
		CountAndSortBlockItems(block, bashCount);
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