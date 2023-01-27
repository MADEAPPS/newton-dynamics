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

#ifndef __CU_COUNTIN_SORT_H__
#define __CU_COUNTIN_SORT_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include "ndCudaContext.h"
//#include "ndCudaSceneInfo.h"
//#include "ndCudaIntrinsics.h"
#include "ndCudaHostBuffer.h"
#include "ndCudaDeviceBuffer.h"

#define D_COUNTING_SORT_BLOCK_SIZE		(1<<8)

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndCudaDeviceBuffer<int>& scansBuffer, ndEvaluateRadix evaluateRadix);

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const ndCudaHostBuffer<T>& src, ndCudaHostBuffer<T>& dst, ndCudaHostBuffer<int>& scansBuffer);


// *****************************************************************
// 
// support function declarations
//
// *****************************************************************
template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountItems(const BufferItem* src, int size, int* histogram, SortKeyPredicate getRadix);

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaAddPartialScans(const BufferItem* src, int size, int* histogram, SortKeyPredicate getRadix);

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const BufferItem* src, BufferItem* dst, int size, int* histogram, SortKeyPredicate getRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndCudaDeviceBuffer<int>& scansBuffer, ndEvaluateRadix evaluateRadix)
{
	ndAssert(src.m_size == dst.m_size);
	ndAssert(context->m_sortPrefixBuffer.m_size >= src.m_size);

	int stride = 1 << exponentRadix;
	int blocks = (src.m_size + stride - 1) / stride;
	ndAssert(stride < D_COUNTING_SORT_BLOCK_SIZE);

	ndCudaCountItems << <blocks, stride, 0 >> > (src.m_array, src.m_size, context->m_sortPrefixBuffer.m_array, evaluateRadix);
	ndCudaAddPartialScans << <1, stride, 0 >> > (src.m_array, src.m_size, context->m_sortPrefixBuffer.m_array, evaluateRadix);
	ndCudaMergeBuckets << <blocks, stride, 0 >> > (src.m_array, dst.m_array, src.m_size, context->m_sortPrefixBuffer.m_array, evaluateRadix);
}

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaAddPartialScans(const BufferItem* src, int size, int* histogram, SortKeyPredicate getRadix)
{
	int sum = 0;
	int offset = 0;
	int threadId = threadIdx.x;
	int radixSize = blockDim.x;
	int blockCount = (size + radixSize - 1) / radixSize;
	__shared__  int localPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

	localPrefixScan[threadId] = 0;
	for (int i = 0; i < blockCount; i++)
	{
		int count = histogram[offset + threadId];
		histogram[offset + threadId] = sum;
		sum += count;
		offset += radixSize;
	}
	histogram[offset + threadId] = sum;
	localPrefixScan[radixSize / 2 + threadId + 1] = sum;

	for (int i = 1; i < radixSize; i = i << 1)
	{
		int sum = localPrefixScan[radixSize / 2 + threadId] + localPrefixScan[radixSize / 2 - i + threadId];
		__syncthreads();
		localPrefixScan[radixSize / 2 + threadId] = sum;
		__syncthreads();
	}
	histogram[offset + radixSize + threadId] = localPrefixScan[radixSize / 2 + threadId];
}

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountItems(const BufferItem* src, int size, int* histogram, SortKeyPredicate getRadix)
{
	__shared__  int cacheBuffer[D_COUNTING_SORT_BLOCK_SIZE];

	int blockId = blockIdx.x;
	int threadId = threadIdx.x;

	cacheBuffer[threadId] = 0;
	__syncthreads();

	int index = threadId + blockDim.x * blockId;
	if (index < size)
	{
		int radix = getRadix(src[index]);
		atomicAdd(&cacheBuffer[radix], 1);
	}
	__syncthreads();

	histogram[index] = cacheBuffer[threadId];
}

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const BufferItem* src, BufferItem* dst, int size, int* histogram, SortKeyPredicate getRadix)
{
	//__shared__  BufferItem cachedCells[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int cacheSortedKey[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int cacheBaseOffset[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int cacheKeyPrefix[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int cacheItemCount[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blocks = (size + blockDim.x - 1) / blockDim.x;

	int srcOffset = blockId * blockDim.x;
	int lastRoadOffset = blocks * blockDim.x;
	int prefixBase = blockDim.x / 2;

	int index = threadId + blockDim.x * blockId;
	cacheItemCount[threadId] = 0;
	cacheKeyPrefix[threadId] = histogram[lastRoadOffset + blockDim.x + threadId];
	//cacheSortedKey[threadId] = (prefixKeySize << 16) | threadId;
	cacheSortedKey[threadId] = (blockDim.x << 16) + threadId;
	if (index < size)
	{
		//cachedCells[threadId] = src[index];
		int radix = getRadix(src[index]);
		cacheSortedKey[threadId] = (radix << 16) + threadId;
	}
	
	//__syncthreads();
	cacheBaseOffset[threadId] = histogram[srcOffset + threadId];
	cacheItemCount[prefixBase + 1 + threadId] = histogram[srcOffset + blockDim.x + threadId] - cacheBaseOffset[threadId];
	
	const int threadId0 = threadId;
	for (int k = 2; k <= blockDim.x; k *= 2)
	{
		for (int j = k / 2; j > 0; j /= 2)
		{
			const int threadId1 = threadId0 ^ j;
			if (threadId1 > threadId0)
			{
				const int a = cacheSortedKey[threadId0];
				const int b = cacheSortedKey[threadId1];
				const int mask0 = (-(threadId0 & k)) >> 31;
				const int mask1 = -(a > b);
				const int mask = mask0 ^ mask1;
				cacheSortedKey[threadId0] = (b & mask) | (a & ~mask);
				cacheSortedKey[threadId1] = (a & mask) | (b & ~mask);
			}
			__syncthreads();
		}
	}
	
	//for (int i = 1; i < prefixKeySize; i = i << 1)
	for (int i = 1; i < blockDim.x; i = i << 1)
	{
		int countSum = cacheItemCount[prefixBase + threadId] + cacheItemCount[prefixBase - i + threadId];
		__syncthreads();
		cacheItemCount[prefixBase + threadId] = countSum;
		__syncthreads();
	}
	
	if (index < size)
	{
		int keyValue = cacheSortedKey[threadId];
		int keyHigh = keyValue >> 16;
		int keyLow = keyValue & 0xffff;
		int cacheItem = cacheItemCount[prefixBase + keyHigh];
		int dstOffset0 = threadId - cacheItem;
		int dstOffset1 = cacheKeyPrefix[keyHigh] + cacheBaseOffset[keyHigh];
		dst[dstOffset0 + dstOffset1] = src[srcOffset + keyLow];
	}
}

template <class T, class ndEvaluateKey, int exponentRadix>
void ndCountingSort(const ndCudaHostBuffer<T>& src, ndCudaHostBuffer<T>& dst, ndCudaHostBuffer<int>& scansBuffer)
{
	//ndAssert(0);
	auto CountItems = [&](int blockIdx, int coreCount)
	{
		ndEvaluateKey evaluator;
		int cacheBuffer[D_COUNTING_SORT_BLOCK_SIZE];
		int size = src.GetCount();
		int blockDim = 1 << exponentRadix;

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			cacheBuffer[threadId] = 0;
		}
		
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			int index = threadId + blockDim * blockIdx;
			if (index < size)
			{
				int radix = evaluator.GetRadix(src[index]);
				//atomicAdd(&cacheBuffer[radix], 1);
				cacheBuffer[radix] ++;
			}
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			int index = threadId + blockDim * blockIdx;
			scansBuffer[index] = cacheBuffer[threadId];
		}
	};

	auto AddPrefix = [&](int blockIdx, int coreCount)
	{
		int size = src.GetCount();
		int blockDim = 1 << exponentRadix;
		int blockCount = (size + blockDim - 1) / blockDim;
		int localPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			localPrefixScan[threadId] = 0;
		}

		int sum[D_COUNTING_SORT_BLOCK_SIZE];
		int offset[D_COUNTING_SORT_BLOCK_SIZE];
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			sum[threadId] = 0;
			offset[threadId] = threadId;
		}
		for (int i = 0; i < blockCount; i++)
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
			scansBuffer[offset[threadId]] = sum[threadId];
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
			scansBuffer[offset[threadId] + blockDim] = localPrefixScan[blockDim / 2 + threadId];
		}
	};

	auto MergeBuckects = [&](int blockIdx, int coreCount)
	{
		ndEvaluateKey evaluator;

		//__shared__  BufferItem cachedCells[D_COUNTING_SORT_BLOCK_SIZE];
		int cacheSortedKey[D_COUNTING_SORT_BLOCK_SIZE];
		int cacheBaseOffset[D_COUNTING_SORT_BLOCK_SIZE];
		int cacheKeyPrefix[D_COUNTING_SORT_BLOCK_SIZE];
		int cacheItemCount[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

		int size = src.GetCount();
		int blockDim = 1 << exponentRadix;
		int blocks = (size + blockDim - 1) / blockDim;

		int srcOffset = blockIdx * blockDim;
		int lastRoadOffset = blocks * blockDim;
		int prefixBase = blockDim / 2;
		
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			cacheItemCount[threadId] = 0;
			cacheKeyPrefix[threadId] = scansBuffer[lastRoadOffset + blockDim + threadId];
			cacheSortedKey[threadId] = (blockDim << 16) + threadId;
		}
		 
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			int index = threadId + blockDim * blockIdx;
			if (index < size)
			{
				//cachedCells[threadId] = src[index];
				int radix = evaluator.GetRadix(src[index]);
				cacheSortedKey[threadId] = (radix << 16) + threadId;
			}
		}

		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			cacheBaseOffset[threadId] = scansBuffer[srcOffset + threadId];
			cacheItemCount[prefixBase + 1 + threadId] = scansBuffer[srcOffset + blockDim + threadId] - cacheBaseOffset[threadId];
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
						const int a = cacheSortedKey[threadId0];
						const int b = cacheSortedKey[threadId1];
						const int mask0 = (-(threadId0 & k)) >> 31;
						const int mask1 = -(a > b);
						const int mask = mask0 ^ mask1;
						cacheSortedKey[threadId0] = (b & mask) | (a & ~mask);
						cacheSortedKey[threadId1] = (a & mask) | (b & ~mask);
					}
				}
			}
		}
		
		//for (int i = 1; i < prefixKeySize; i = i << 1)
		for (int i = 1; i < blockDim; i = i << 1)
		{
			int sumReg[D_COUNTING_SORT_BLOCK_SIZE];
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				sumReg[threadId] = cacheItemCount[prefixBase + threadId] + cacheItemCount[prefixBase - i + threadId];
			}
			for (int threadId = 0; threadId < blockDim; ++threadId)
			{
				cacheItemCount[prefixBase + threadId] = sumReg[threadId];
			}
		}
		
		for (int threadId = 0; threadId < blockDim; ++threadId)
		{
			int index = threadId + blockDim * blockIdx;
			if (index < size)
			{
				int keyValue = cacheSortedKey[threadId];
				int keyHigh = keyValue >> 16;
				int keyLow = keyValue & 0xffff;
				int cacheItem = cacheItemCount[prefixBase + keyHigh];
				int dstOffset0 = threadId - cacheItem;
				int dstOffset1 = cacheKeyPrefix[keyHigh] + cacheBaseOffset[keyHigh];
				dst[dstOffset0 + dstOffset1] = src[srcOffset + keyLow];
			}
		}
	};

	ndAssert(src.GetCount() == dst.GetCount());
	ndAssert(scansBuffer.GetCount() >= src.GetCount());

	int coreCount = 2;
	int stride = 1 << exponentRadix;
	int blocks = (src.GetCount() + stride - 1) / stride;
	int superBlock = (blocks + coreCount - 1) / coreCount;
	ndAssert(stride < D_COUNTING_SORT_BLOCK_SIZE);

	for (int block = 0; block < blocks; ++block)
	{
		CountItems(block, coreCount);
	}

	for (int block = 0; block < 1; ++block)
	{
		AddPrefix(block, coreCount);
	}

	for (int block = 0; block < blocks; ++block)
	{
		MergeBuckects(block, coreCount);
	}
}

#endif