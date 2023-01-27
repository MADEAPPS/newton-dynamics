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


template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndCudaDeviceBuffer<int>& scansBuffer, ndEvaluateRadix evaluateRadix);


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
	__shared__  unsigned cacheSortedKey[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  unsigned cacheBaseOffset[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  unsigned cacheKeyPrefix[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  unsigned cacheItemCount[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

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
	cacheSortedKey[threadId] = (blockDim.x << 16) | threadId;
	if (index < size)
	{
		//cachedCells[threadId] = src[index];
		int radix = getRadix(src[index]);
		cacheSortedKey[threadId] = (radix << 16) | threadId;
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

#endif