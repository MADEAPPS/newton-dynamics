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


#define D_COUNTING_SORT_BLOCK_SIZE		(1<<8)

#if 0
__global__ void ndCudaCountingCellsPrefixScanInternal(unsigned* histogram, unsigned blockCount);


inline unsigned __device__ ndCudaCountingSortCalculateScanPrefixSize(unsigned items, unsigned keySize)
{
	unsigned blocks = (items + D_COUNTING_SORT_BLOCK_SIZE - 1) / D_COUNTING_SORT_BLOCK_SIZE;
	return keySize * (blocks + 2);
}

template <typename BufferItem, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountItemsInternal(const BufferItem* src, unsigned* histogram, unsigned size, SortKeyPredicate sortKey, unsigned prefixKeySize)
{
	__shared__  unsigned cacheBuffer[D_COUNTING_SORT_BLOCK_SIZE];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	
	cacheBuffer[threadId] = 0;
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

	cacheKeyPrefix[threadId] = 0;
	cacheItemCount[threadId] = 0;
	__syncthreads();

	if (threadId < prefixKeySize)
	{
		cacheBaseOffset[threadId] = histogram[srcOffset + threadId];
		cacheKeyPrefix[prefixBase + 1 + threadId] = histogram[lastRoadOffset + threadId];
		cacheItemCount[prefixBase + 1 + threadId] = histogram[srcOffset + prefixKeySize + threadId] - cacheBaseOffset[threadId];
	}
	
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
	
	for (int i = 1; i < prefixKeySize; i = i << 1)
	{
		const unsigned prefixSum = cacheKeyPrefix[prefixBase + threadId] + cacheKeyPrefix[prefixBase - i + threadId];
		const unsigned countSum = cacheItemCount[prefixBase + threadId] + cacheItemCount[prefixBase - i + threadId];
		__syncthreads();
		cacheKeyPrefix[prefixBase + threadId] = prefixSum;
		cacheItemCount[prefixBase + threadId] = countSum;
		__syncthreads();
	}
	
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
	if ((index >= 1) && (index < size))
	{
		const unsigned key0 = GetSortKey(dst[index - 1]);
		const unsigned key1 = GetSortKey(dst[index - 0]);
		//printf("%llx id:%d (%llx %llx) (%x %x)\n", dst, threadIdx.x, dst[index - 1], dst[index - 0], key0, key1);
		if (info.m_frameIsValid && (key0 > key1))
		{
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
		}
	}
}

template <typename BufferItem, typename PredicateGetSrcBuffer, typename PredicateGetDstBuffer, typename PredicateGetItemsCount, typename PredicateGetSortKey>
__global__ void ndCudaCountingSort(
	ndCudaSceneInfo& info, BufferItem, 
	PredicateGetSrcBuffer GetSrcBuffer, PredicateGetDstBuffer GetDstBuffer, PredicateGetItemsCount GetItemsCount,
	PredicateGetSortKey GetSortKey, unsigned keySize)
{
	if (info.m_frameIsValid)
	{
		//printf("%s %d\n", __FUNCTION__, info.m_frameCount);
		unsigned size = GetItemsCount(info);
		BufferItem* dst = GetDstBuffer(info);
		const BufferItem* src = GetSrcBuffer(info);

		unsigned* histogram = info.m_histogram.m_array;
		const unsigned blocks = (size + D_COUNTING_SORT_BLOCK_SIZE - 1) / D_COUNTING_SORT_BLOCK_SIZE;
		ndCudaCountingSortCountItemsInternal << <blocks, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (src, histogram, size, GetSortKey, keySize);
		ndCudaCountingCellsPrefixScanInternal << <1, keySize, 0 >> > (histogram, blocks);
		ndCudaCountingSortCountShuffleItemsInternal << <blocks, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (src, dst, histogram, size, GetSortKey, keySize);

		#ifdef _DEBUG
		//cudaDeviceSynchronize();
		//printf("unsorted: %s count(%d)\n", __FUNCTION__, size);
		//for (int i = 0; i < size; i++)
		//{
		//	ndCudaBodyAabbCell cell;
		//	cell.m_value = src[i];
		//	printf("thread(%d) x(%d) y(%d) z(%d) id(%d)  %llx\n", i, cell.m_x, cell.m_y, cell.m_z, cell.m_id, cell.m_value);
		//}
		//printf("\n");
		//
		//printf("sorted: %s count(%d)\n", __FUNCTION__, size);
		//for (int i = 0; i < size; i++)
		//{
		//	ndCudaBodyAabbCell cell;
		//	cell.m_value = dst[i];
		//	printf("thread(%d) x(%d) y(%d) z(%d) id(%d)  %llx\n", i, cell.m_x, cell.m_y, cell.m_z, cell.m_id, cell.m_value);
		//}
		//printf("\n");
		//printf("\n");

		ndCudaCountingSortCountSanityCheckInternal << <blocks, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (info, dst, size, GetSortKey);
		#endif
	}
}
#endif


template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndCudaDeviceBuffer<int>& scansBuffer, ndEvaluateRadix evaluateRadix);


// *****************************************************************
// 
// support functions implementation 
//
// *****************************************************************

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
__global__ void ndAddPartialScans(const BufferItem* src, int size, int* histogram, SortKeyPredicate getRadix)
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

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndCudaDeviceBuffer<int>& scansBuffer, ndEvaluateRadix evaluateRadix)
{
	ndAssert(src.m_size == dst.m_size);
	ndAssert(src.m_size >= context->m_sortPrefixBuffer.m_size);

	int stride = 1 << exponentRadix;
	int blocks = (src.m_size + stride - 1) / stride;
	ndAssert(stride < D_COUNTING_SORT_BLOCK_SIZE);

	ndCudaCountItems << <blocks, stride, 0 >> > (src.m_array, src.m_size, context->m_sortPrefixBuffer.m_array, evaluateRadix);
	ndAddPartialScans << <1, stride, 0 >> > (src.m_array, src.m_size, context->m_sortPrefixBuffer.m_array, evaluateRadix);
}

#endif