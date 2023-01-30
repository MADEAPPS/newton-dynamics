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
#include "ndCudaHostBuffer.h"
#include "ndCudaDeviceBuffer.h"

//#define D_COUNTING_SORT_BLOCK_SIZE	(8)
//#define D_COUNTING_SORT_BLOCK_SIZE	(1<<8)
//#define D_COUNTING_SORT_BLOCK_SIZE	(1<<9)
#define D_COUNTING_SORT_BLOCK_SIZE	(1<<10)

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndCudaDeviceBuffer<int>& scansBuffer, ndEvaluateRadix evaluateRadix);

// *****************************************************************
// 
// support function declarations
//
// *****************************************************************
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaAddPrefix(const T* src, int blocksCount, int* scansBuffer, SortKeyPredicate getRadix);

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaCountItems(const T* src, int bufferSize, int blocksCount, int* scansBuffer, int radixStride, SortKeyPredicate getRadix);

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const T* src, T* dst, int bufferSize, int blocksCount, int* scansBuffer, int radixStride, SortKeyPredicate getRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaAddPrefix(const T* src, int computeUnits, int* scansBuffer, SortKeyPredicate getRadix)
{
	__shared__  int localPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

	int threadId = threadIdx.x;
	int radixSize = blockDim.x;
	int radixHalfSize = radixSize / 2;

	int sum = 0;
	int offset = 0;
	localPrefixScan[threadId] = 0;
	for (int i = 0; i < computeUnits; ++i)
	{
		int count = scansBuffer[offset + threadId];
		scansBuffer[offset + threadId] = sum;
		sum += count;
		offset += radixSize;
	}

	localPrefixScan[radixHalfSize + threadId + 1] = sum;
	__syncthreads();

	for (int i = 1; i < radixSize; i = i << 1)
	{
		int acc = localPrefixScan[radixHalfSize + threadId] + localPrefixScan[radixHalfSize - i + threadId];
		__syncthreads();
		localPrefixScan[radixHalfSize + threadId] = acc;
		__syncthreads();
	}
	scansBuffer[offset + threadId] = localPrefixScan[radixHalfSize + threadId];
}

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaCountItems(const T* src, int bufferSize, int blocksCount, int* scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int radixCountBuffer[D_COUNTING_SORT_BLOCK_SIZE];

	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int blockIndex = blockIdx.x;
	int bashSize = blocksCount * blockSride * blockIndex;

	radixCountBuffer[threadId] = 0;
	__syncthreads();

	for (int i = 0; i < blocksCount; ++i)
	{
		int index = bashSize + threadId;
		if (index < bufferSize)
		{
			int radix = getRadix(src[index]);
			atomicAdd(&radixCountBuffer[radix], 1);
		}
		bashSize += blockSride;
	}

	__syncthreads();
	if (threadId < radixStride)
	{
		int index = threadId + radixStride * blockIndex;
		scansBuffer[index] = radixCountBuffer[threadId];
	}
}

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const T* src, T* dst, int bufferSize, int blocksCount, int* scansBuffer, int radixStride, int computeUnits, SortKeyPredicate getRadix)
{
	__shared__  T cachedItems[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int sortedRadix[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int radixPrefixCount[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int radixPrefixStart[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int radixPrefixBatchScan[D_COUNTING_SORT_BLOCK_SIZE];
	__shared__  int radixPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int blockIndex = blockIdx.x;
	int radixBase = blockIndex * radixStride;
	int bashSize = blocksCount * blockSride * blockIndex;
	int radixPrefixOffset = computeUnits * radixStride;

	radixPrefixScan[threadId] = 0;
	if (threadId < radixStride)
	{
		radixPrefixStart[threadId] = scansBuffer[radixBase + threadId];
		radixPrefixBatchScan[threadId] = scansBuffer[radixPrefixOffset + threadId];
	}

	for (int i = 0; i < blocksCount; ++i)
	{
		radixPrefixCount[threadId] = 0;
		sortedRadix[threadId] = (radixStride << 16) + threadId;
		__syncthreads();
	
		int index = bashSize + threadId;
		if (index < bufferSize)
		{
			cachedItems[threadId] = src[index];
			int radix = getRadix(cachedItems[threadId]);
			atomicAdd(&radixPrefixCount[radix], 1);
			sortedRadix[threadId] = (radix << 16) + threadId;
		}
		__syncthreads();

		radixPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + threadId + 1] = radixPrefixCount[threadId];
		for (int k = 1; k < radixStride; k = k << 1)
		{
			__syncthreads();
			int sum = radixPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + threadId] + radixPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + threadId - k];
			__syncthreads();
			radixPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + threadId] = sum;
		}

		int threadId0 = threadId;
		for (int k = 2; k <= blockSride; k = k << 1)
		{
			for (int j = k >> 1; j > 0; j = j >> 1)
			{
				int threadId1 = threadId0 ^ j;
				if (threadId1 > threadId0)
				{
					const int a = sortedRadix[threadId0];
					const int b = sortedRadix[threadId1];
					const int mask0 = (-(threadId0 & k)) >> 31;
					const int mask1 = -(a > b);
					const int mask = mask0 ^ mask1;
					sortedRadix[threadId0] = (b & mask) | (a & ~mask);
					sortedRadix[threadId1] = (a & mask) | (b & ~mask);
				}
				__syncthreads();
			}
		}

		if (index < bufferSize)
		{
			int keyValue = sortedRadix[threadId];
			int keyHigh = keyValue >> 16;
			int keyLow = keyValue & 0xffff;
			int dstOffset1 = radixPrefixBatchScan[keyHigh] + radixPrefixStart[keyHigh];
			int dstOffset0 = threadId - radixPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + keyHigh];
			dst[dstOffset0 + dstOffset1] = cachedItems[keyLow];
		}
	
		__syncthreads();
		radixPrefixStart[threadId] += radixPrefixCount[threadId];
		bashSize += blockSride;
	}
}

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndCudaDeviceBuffer<int>& scansBuffer, ndEvaluateRadix evaluateRadix)
{
	int itemCount = src.GetCount();
	int radixStride = 1 << exponentRadix;
	int deviceComputeUnits = context->GetComputeUnits();
	int computeUnitsBashCount = (itemCount + D_COUNTING_SORT_BLOCK_SIZE - 1) / D_COUNTING_SORT_BLOCK_SIZE;
	int bashCount = (computeUnitsBashCount + deviceComputeUnits - 1) / deviceComputeUnits;
	int computeUnits = (itemCount + bashCount * D_COUNTING_SORT_BLOCK_SIZE - 1) / (bashCount * D_COUNTING_SORT_BLOCK_SIZE);
	ndAssert(computeUnits <= deviceComputeUnits);

	ndCudaCountItems << <computeUnits, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (src.m_array, itemCount, bashCount, context->m_sortPrefixBuffer.m_array, radixStride, evaluateRadix);
	ndCudaAddPrefix << <1, radixStride, 0 >> > (src.m_array, computeUnits, context->m_sortPrefixBuffer.m_array, evaluateRadix);
	ndCudaMergeBuckets << <computeUnits, D_COUNTING_SORT_BLOCK_SIZE, 0 >> > (src.m_array, dst.m_array, itemCount, bashCount, context->m_sortPrefixBuffer.m_array, radixStride, computeUnits, evaluateRadix);
}

#endif