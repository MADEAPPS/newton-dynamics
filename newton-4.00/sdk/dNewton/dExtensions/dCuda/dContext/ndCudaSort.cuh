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

#ifndef __CUDA_SORT_H__
#define __CUDA_SORT_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include "ndCudaDevice.h"
#include "ndCudaContext.h"
#include "ndCudaHostBuffer.h"
#include "ndCudaDeviceBuffer.h"

//#define D_COUNTING_SORT_BLOCK_SIZE	(8)
//#define D_COUNTING_SORT_BLOCK_SIZE	(1<<8)
#define D_COUNTING_SORT_BLOCK_SIZE	(1<<9)
//#define D_COUNTING_SORT_BLOCK_SIZE	(1<<10)

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaAddPrefix(const ndKernelParams params, const ndAssessor<T> dommy, ndAssessor<int> scansBuffer, SortKeyPredicate getRadix)
{
	__shared__  int localPrefixScan[D_COUNTING_SORT_BLOCK_SIZE / 2 + D_COUNTING_SORT_BLOCK_SIZE + 1];

	int threadId = threadIdx.x;
	int radixSize = blockDim.x;
	int radixHalfSize = radixSize / 2;

	int sum = 0;
	int offset = 0;
	localPrefixScan[threadId] = 0;
	for (int i = 0; i < params.m_kernelCount; ++i)
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
__global__ void ndCudaCountItems(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int radixCountBuffer[D_COUNTING_SORT_BLOCK_SIZE];

	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int blockIndex = blockIdx.x;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		radixCountBuffer[threadId] = 0;
	}
	__syncthreads();

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = bashSize + threadId;
		if (index < input.m_size)
		{
			int radix = getRadix(input[index]);
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
__global__ void ndCudaMergeBuckets(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
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
	int halfRadixStride = radixStride / 2;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		radixPrefixScan[threadId] = 0;
		radixPrefixStart[threadId] = scansBuffer[radixBase + threadId];
		radixPrefixBatchScan[threadId] = scansBuffer[radixPrefixOffset + threadId];
	}

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		if (threadId < radixStride)
		{
			radixPrefixCount[threadId] = 0;
		}
		sortedRadix[threadId] = (radixStride << 16) + threadId;
		__syncthreads();
	
		int index = bashSize + threadId;
		if (index < input.m_size)
		{
			cachedItems[threadId] = input[index];
			int radix = getRadix(cachedItems[threadId]);
			atomicAdd(&radixPrefixCount[radix], 1);
			sortedRadix[threadId] = (radix << 16) + threadId;
		}
		__syncthreads();

		if (threadId < radixStride)
		{
			radixPrefixScan[halfRadixStride + threadId + 1] = radixPrefixCount[threadId];
		}
		for (int k = 1; k < radixStride; k = k << 1)
		{
			int sum;
			__syncthreads();
			if (threadId < radixStride)
			{
				sum = radixPrefixScan[halfRadixStride + threadId] + radixPrefixScan[halfRadixStride + threadId - k];
			}
			__syncthreads();
			if (threadId < radixStride)
			{
				radixPrefixScan[halfRadixStride + threadId] = sum;
			}
		}

		int threadId0 = threadId;
		for (int k = 2; k <= blockSride; k = k << 1)
		{
			for (int j = k >> 1; j > 0; j = j >> 1)
			{
				int threadId1 = threadId0 ^ j;
				#if 1
				if (threadId1 > threadId0)
				{
					const int a = sortedRadix[threadId0];
					const int b = sortedRadix[threadId1];
					const int mask0 = (-(threadId0 & k)) >> 31;
					const int mask1 = -(a > b);
					const int mask2 = mask0 ^ mask1;
					const int a1 = mask2 ? b : a;
					const int b1 = mask2 ? a : b;
					sortedRadix[threadId0] = a1;
					sortedRadix[threadId1] = b1;
				}
				__syncthreads();
				#else
				const int a = sortedRadix[threadId0];
				const int b = sortedRadix[threadId1];
				const int mask0 = (-(threadId0 & k)) >> 31;
				const int mask1 = -(a > b);
				const int mask2 = mask0 ^ mask1;
				const int mask3 = -(threadId1 < threadId0);
				const int a1 = mask2 ? b : a;
				const int b1 = mask2 ? a : b;
				//const int a1 = (b & mask2) | (a & ~mask2);
				const int a2 = mask3 ? b1 : a1;
				__syncthreads();
				//sortedRadix[threadId0] = (b1 & mask2) | (a1 & ~mask2);
				sortedRadix[threadId0] = a2;
				__syncthreads();
				#endif
			}
		}

		if (index < input.m_size)
		{
			int keyValue = sortedRadix[threadId];
			int keyHigh = keyValue >> 16;
			int keyLow = keyValue & 0xffff;
			int dstOffset1 = radixPrefixBatchScan[keyHigh] + radixPrefixStart[keyHigh];
			int dstOffset0 = threadId - radixPrefixScan[halfRadixStride + keyHigh];
			output[dstOffset0 + dstOffset1] = cachedItems[keyLow];
		}
		__syncthreads();
		if (threadId < radixStride)
		{
			radixPrefixStart[threadId] += radixPrefixCount[threadId];
		}
		bashSize += blockSride;
	}
}

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix)
{
	ndAssessor<T> input(src);
	ndAssessor<T> output(dst);
	ndAssessor<int> prefixScanBuffer(context->GetPrefixScanBuffer());
	ndKernelParams params(context->GetDevice(), D_COUNTING_SORT_BLOCK_SIZE, src.GetCount());

	int radixStride = 1 << exponentRadix;
	ndCudaCountItems << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, prefixScanBuffer, radixStride, evaluateRadix);
	ndCudaAddPrefix << <1, radixStride, 0 >> > (params, input, prefixScanBuffer, evaluateRadix);
	ndCudaMergeBuckets << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, output, prefixScanBuffer, radixStride, evaluateRadix);
}

#endif