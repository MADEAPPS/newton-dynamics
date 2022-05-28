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
#include "ndCudaSceneInfo.h"
#include "ndCudaIntrinsics.h"

//#define D_COUNTING_SORT_BLOCK_SIZE			(1<<3)
#define D_COUNTING_SORT_BLOCK_SIZE				(1<<10)
#define D_COUNTING_INTERNAL_SCAN_BUFFER_SIZE	(1<<8)

__global__ void ndCudaCountingCellsPrefixScanInternal(unsigned* histogram, unsigned blockCount)
{
	unsigned sum = 0;
	unsigned offset = 0;
	const unsigned keySize = blockDim.x;

	const unsigned threadId = threadIdx.x;
	for (int i = 0; i < blockCount; i++)
	{
		const unsigned count = histogram[offset + threadId];
		histogram[offset + threadId] = sum;
		sum += count;
		offset += keySize;
	}
	histogram[offset + threadId] = sum;
}

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountItemsInternal(const BufferItem* src, unsigned* histogram, unsigned size, SortKeyPredicate sortKey, unsigned prefixKeySize)
{
	__shared__  unsigned cacheBuffer[D_COUNTING_INTERNAL_SCAN_BUFFER_SIZE];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	
	if (threadId < prefixKeySize)
	{
		cacheBuffer[threadId] = 0;
	}
	__syncthreads();

	const unsigned index = threadId + blockDim.x * blockId;
	if (index < size)
	{
		const unsigned key = sortKey(src[index]);
		atomicAdd(&cacheBuffer[key], 1);
	}
	__syncthreads();
	
	if (threadId < prefixKeySize)
	{
		const unsigned dstBase = prefixKeySize * blockId;
		histogram[dstBase + threadId] = cacheBuffer[threadId];
	}
}

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountShuffleItemsInternal(const BufferItem* src, BufferItem* dst, unsigned* histogram, unsigned size, SortKeyPredicate GetSortKey, unsigned prefixKeySize)
{
	__shared__  BufferItem cachedCells[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  unsigned cacheSortedKey[D_COUNTING_SORT_BLOCK_SIZE];
	//__shared__  unsigned cacheBaseOffset[D_COUNTING_INTERNAL_SCAN_BUFFER_SIZE];
	//__shared__  unsigned cacheKeyPrefix[D_COUNTING_INTERNAL_SCAN_BUFFER_SIZE / 2 + D_COUNTING_INTERNAL_SCAN_BUFFER_SIZE + 1 + D_COUNTING_SORT_BLOCK_SIZE];
	//__shared__  unsigned cacheItemCount[D_COUNTING_INTERNAL_SCAN_BUFFER_SIZE / 2 + D_COUNTING_INTERNAL_SCAN_BUFFER_SIZE + 1 + D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  unsigned cacheBaseOffset[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  unsigned cacheKeyPrefix[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];
	__shared__  unsigned cacheItemCount[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	const unsigned blocks = (size + blockDim.x - 1) / blockDim.x;
	
	const unsigned index = threadId + blockDim.x * blockId;
	cacheSortedKey[threadId] = (prefixKeySize << 16) | threadId;
	if (index < size)
	{
		cachedCells[threadId] = src[index];
		const unsigned key = GetSortKey(src[index]);
		cacheSortedKey[threadId] = (key << 16) | threadId;
	}

	const unsigned srcOffset = blockId * prefixKeySize;
	const unsigned lastRoadOffset = blocks * prefixKeySize;
	const unsigned prefixBase = D_COUNTING_SORT_BLOCK_SIZE / 2;

	if (threadId < prefixKeySize)
	{
		cacheKeyPrefix[threadId] = 0;
		cacheItemCount[threadId] = 0;
		cacheBaseOffset[threadId] = histogram[srcOffset + threadId];
	}
	__syncthreads();

	if (threadId < prefixKeySize)
	{
		cacheKeyPrefix[prefixBase + 1 + threadId] = histogram[lastRoadOffset + threadId];
		cacheItemCount[prefixBase + 1 + threadId] = histogram[srcOffset + prefixKeySize + threadId] - cacheBaseOffset[threadId];
	}
	
	const int threadId0 = threadId;
	for (int k = 2; k <= D_COUNTING_SORT_BLOCK_SIZE; k *= 2)
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
	
	for (int i = 1; i < D_COUNTING_SORT_BLOCK_SIZE; i = i << 1)
	{
		const unsigned prefixSum = cacheKeyPrefix[prefixBase + threadId] + cacheKeyPrefix[prefixBase - i + threadId];
		const unsigned countSum = cacheItemCount[prefixBase + threadId] + cacheItemCount[prefixBase - i + threadId];
		__syncthreads();
		cacheKeyPrefix[prefixBase + threadId] = prefixSum;
		cacheItemCount[prefixBase + threadId] = countSum;
		__syncthreads();
	}
	
	//const unsigned keyValue = cacheSortedKey[threadId];
	//const unsigned key = keyValue >> 16;
	//const unsigned threadIdBase = cacheItemCount[prefixBase + key];
	//const unsigned dstOffset0 = threadId - threadIdBase;
	//const unsigned dstOffset1 = cacheKeyPrefix[prefixBase + key] + cacheBaseOffset[key];
	//if ((dstOffset0 + dstOffset1)> size)
	//{
	//	printf("size(%d) dstOffset0(%d) dstOffset1(%d)\n", size, dstOffset0, dstOffset1);
	//}
	
	if (index < size)
	{
		const unsigned keyValue = cacheSortedKey[threadId];
		const unsigned key = keyValue >> 16;
		const unsigned threadIdBase = cacheItemCount[prefixBase + key];
	
		const unsigned dstOffset0 = threadId - threadIdBase;
		const unsigned dstOffset1 = cacheKeyPrefix[prefixBase + key] + cacheBaseOffset[key];
		dst[dstOffset0 + dstOffset1] = cachedCells[keyValue & 0xffff];
	}
}

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountSanityCheckInternal(ndCudaSceneInfo& info, const BufferItem* dst, unsigned size, SortKeyPredicate GetSortKey)
{
	const unsigned index = threadIdx.x + blockIdx.x * blockDim.x;
	if ((index > 1) && (index < size))
	{
		printf("esto es un mierda\n");
		const unsigned key0 = GetSortKey(dst[index - 0]);
		const unsigned key1 = GetSortKey(dst[index - 1]);
		if (info.m_frameIsValid && (key0 > key1))
		{
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
		}
	}
}

inline unsigned __device__ ndCudaCountingSortCalculateScanPrefixSize(unsigned items, unsigned keySize)
{
	unsigned blocks = (items + D_COUNTING_SORT_BLOCK_SIZE - 1) / D_COUNTING_SORT_BLOCK_SIZE;
	return keySize * (blocks + 2);
}

template <typename BufferItem, typename PredicateGetSrcBuffer, typename PredicateGetDstBuffer, typename PredicateGetItemsCount, typename PredicateGetSortKey>
__global__ void ndCudaCountingSort(
	ndCudaSceneInfo& info, BufferItem, 
	PredicateGetSrcBuffer GetSrcBuffer, PredicateGetDstBuffer GetDstBuffer, PredicateGetItemsCount GetItemsCount,
	PredicateGetSortKey GetSortKey, unsigned keySize)
{
	if (info.m_frameIsValid)
	{
		unsigned size = GetItemsCount(info);
		BufferItem* dst = GetDstBuffer(info);
		const BufferItem* src = GetSrcBuffer(info);

		unsigned* histogram = info.m_histogram.m_array;
		const unsigned blocks = (size + D_COUNTING_SORT_BLOCK_SIZE - 1) / D_COUNTING_SORT_BLOCK_SIZE;
		ndCudaCountingSortCountItemsInternal << <blocks, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (src, histogram, size, GetSortKey, keySize);
		ndCudaCountingCellsPrefixScanInternal << <1, keySize, 0 >> > (histogram, blocks);
		ndCudaCountingSortCountShuffleItemsInternal << <blocks, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (src, dst, histogram, size, GetSortKey, keySize);

		#ifdef _DEBUG
		dst[0] = 0xffffffffffffffff;
		ndCudaCountingSortCountSanityCheckInternal << <blocks, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (info, dst, size, GetSortKey);
		#endif
	}
}

#endif