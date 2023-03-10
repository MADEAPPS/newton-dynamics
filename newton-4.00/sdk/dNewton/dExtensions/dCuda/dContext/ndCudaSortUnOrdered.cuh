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

#ifndef __CUDA_SORT_UNORDERED_H__
#define __CUDA_SORT_UNORDERED_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include "ndCudaDevice.h"
#include "ndCudaContext.h"
#include "ndCudaHostBuffer.h"
#include "ndCudaDeviceBuffer.h"

#define D_DEVICE_UNORDERED_SORT_BLOCK_SIZE	(1<<8)
//#define D_DEVICE_UNORDERED_SORT_BLOCK_SIZE	(1<<9)
//#define D_DEVICE_UNORDERED_SORT_BLOCK_SIZE	(1<<10)

#define D_DEVICE_UNORDERED_MAX_RADIX_SIZE	(1<<8)

#if D_DEVICE_UNORDERED_MAX_RADIX_SIZE > D_DEVICE_UNORDERED_SORT_BLOCK_SIZE
	#error counting sort diget larger that block
#endif

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSortUnOrdered(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaAddPrefixUnordered(const ndKernelParams params, const ndAssessor<T> dommy, ndAssessor<int> scansBuffer, SortKeyPredicate getRadix)
{
	__shared__  int localPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2 + D_DEVICE_UNORDERED_MAX_RADIX_SIZE + 1];

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
__global__ void ndCudaCountItemsUnordered(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int radixCountBuffer[D_DEVICE_UNORDERED_MAX_RADIX_SIZE];

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

#define D_USE_BITONIC_SORT

#ifdef D_USE_BITONIC_SORT
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBucketsUnOrdered(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  T cachedItems[D_DEVICE_UNORDERED_SORT_BLOCK_SIZE];
	__shared__  int sortedRadix[D_DEVICE_UNORDERED_SORT_BLOCK_SIZE + 1];
	__shared__  int radixPrefixCount[D_DEVICE_UNORDERED_MAX_RADIX_SIZE];
	__shared__  int radixPrefixStart[D_DEVICE_UNORDERED_MAX_RADIX_SIZE];
	__shared__  int radixPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2 + D_DEVICE_UNORDERED_MAX_RADIX_SIZE + 1];

	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int blockIndex = blockIdx.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	radixPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2] = 0;
	if (threadId < radixStride)
	{
		radixPrefixScan[(D_DEVICE_UNORDERED_MAX_RADIX_SIZE - radixStride) / 2 + threadId] = 0;
		radixPrefixStart[threadId] = scansBuffer[radixBase + threadId] + scansBuffer[radixPrefixOffset + threadId];
	}

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		if (threadId < radixStride)
		{
			radixPrefixCount[threadId] = 0;
		}
		__syncthreads();
	
		int index = bashSize + threadId;
		if (index < input.m_size)
		{
			cachedItems[threadId] = input[index];
			int radix = getRadix(cachedItems[threadId]);
			atomicAdd(&radixPrefixCount[radix], 1);
			sortedRadix[threadId] = (radix << 16) + threadId;
		} 
		else
		{
			sortedRadix[threadId] = (radixStride << 16);
		}
		__syncthreads();

		if (threadId < radixStride)
		{
			radixPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2 + threadId + 1] = radixPrefixCount[threadId];
		}
		for (int k = 1; k < radixStride; k = k << 1)
		{
			int sum;
			__syncthreads();
			if (threadId < radixStride)
			{
				sum = radixPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2 + threadId] + radixPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2 + threadId - k];
			}
			__syncthreads();
			if (threadId < radixStride)
			{
				radixPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2 + threadId] = sum;
			}
		}

#if 1
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
					const int mask2 = mask0 ^ mask1;
					//const int a1 = mask2 ? b : a;
					//const int b1 = mask2 ? a : b;
					//sortedRadix[threadId0] = a1;
					//sortedRadix[threadId1] = b1;
					if (mask2)
					{
						sortedRadix[threadId0] = b;
						sortedRadix[threadId1] = a;
					}
				}
				__syncthreads();
			}
		}
#else
		__shared__  int passes[1];
		if (threadId == 0)
		{
			passes[0] = 1;
			sortedRadix[blockSride] = (radixStride << 16);
		}
		__syncthreads();
		while (passes[0])
		{
			if (threadId == 0)
			{
				passes[0] = 0;
			}
			__syncthreads();

			if (threadId < blockSride / 2)
			{
				int id0 = threadId * 2 + 0;
				int id1 = threadId * 2 + 1;
				const int a = sortedRadix[id0];
				const int b = sortedRadix[id1];
				if (b > a)
				{
					passes[0] = 1;
					sortedRadix[id0] = b;
					sortedRadix[id1] = a;
				}
			}
			__syncthreads();

			if (threadId < blockSride / 2)
			{
				int id0 = threadId * 2 + 1;
				int id1 = threadId * 2 + 2;
				const int a = sortedRadix[id0];
				const int b = sortedRadix[id1];
				if (b > a)
				{
					passes[0] = 1;
					sortedRadix[id0] = b;
					sortedRadix[id1] = a;
				}
			}
			__syncthreads();
		}

#endif

		if (index < input.m_size)
		{
			int keyValue = sortedRadix[threadId];
			int keyHigh = keyValue >> 16;
			int keyLow = keyValue & 0xffff;
			int dstOffset1 = radixPrefixStart[keyHigh];
			int dstOffset0 = threadId - radixPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE / 2 + keyHigh];
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

#else

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBucketsUnOrdered(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int scanBaseAdress[D_DEVICE_UNORDERED_MAX_RADIX_SIZE];
	int threadId = threadIdx.x;
	int blockIndex = blockIdx.x;
	int blockStride = blockDim.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		scanBaseAdress[threadId] = scansBuffer[radixPrefixOffset + threadId] + scansBuffer[radixBase + threadId];
	}
	__syncthreads();

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = bashSize + threadId;
		if (index < input.m_size)
		{
			T item(input[index]);
			int radix = getRadix(item);
			int address = atomicAdd(&scanBaseAdress[radix], 1);
			output[address] = item;
		}
		bashSize += blockStride;
	}
}
#endif

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSortUnOrdered(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix)
{
	ndAssessor<T> input(src);
	ndAssessor<T> output(dst);
	ndAssessor<int> prefixScanBuffer(context->GetPrefixScanBuffer());
	ndKernelParams params(context->GetDevice(), D_DEVICE_UNORDERED_SORT_BLOCK_SIZE, src.GetCount());

	ndAssert(src.GetCount() == dst.GetCount());
	ndAssert((1 << exponentRadix) <= D_DEVICE_UNORDERED_MAX_RADIX_SIZE);

	int radixStride = 1 << exponentRadix;
	ndCudaCountItemsUnordered << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, prefixScanBuffer, radixStride, evaluateRadix);
	ndCudaAddPrefixUnordered << <1, radixStride, 0 >> > (params, input, prefixScanBuffer, evaluateRadix);
	ndCudaMergeBucketsUnOrdered << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, output, prefixScanBuffer, radixStride, evaluateRadix);
}

#endif