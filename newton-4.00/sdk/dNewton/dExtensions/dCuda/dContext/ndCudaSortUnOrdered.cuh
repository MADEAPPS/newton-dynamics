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

//#define D_DEVICE_UNORDERED_MAX_RADIX_SIZE	(1<<8)
#define D_DEVICE_UNORDERED_MAX_RADIX_SIZE	(1<<10)
#define D_DEVICE_UNORDERED_SORT_BLOCK_SIZE	(1<<10)

#if D_DEVICE_UNORDERED_MAX_RADIX_SIZE > D_DEVICE_UNORDERED_SORT_BLOCK_SIZE
	#error counting sort diget larger that block
#endif

/// <summary>
/// ndCountingSortUnOrdered use for fast unstable sort.
/// since the orted array is unstable, this funtion can not be used as the base for a stable radix sort
/// </summary>
/// <typeparam name="T">item class or type</typeparam>
/// <typeparam name="ndEvaluateRadix">radix sort size in bits</typeparam>
/// <param name="context">cuda context class, for internal temporary scan array and other info</param>
/// <param name="src">source input array</param>
/// <param name="dst">destnation output array</param>
/// <param name="evaluateRadix">lambda function to get radix form items in array</param>
template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSortUnOrdered(ndCudaContextImplement* const context, const ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaAddPrefixUnordered(const ndKernelParams params, const ndAssessor<T> dommy, ndAssessor<int> scansBuffer, SortKeyPredicate getRadix)
{
#if 1
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

#else
	__shared__  int localPrefixScan[D_DEVICE_UNORDERED_MAX_RADIX_SIZE + 1];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;

	int sum = 0;
	int offset = 0;
	if (threadId == 0)
	{
		localPrefixScan[0] = 0;
	}
	for (int i = 0; i < params.m_kernelCount; ++i)
	{
		int count = scansBuffer[offset + threadId];
		scansBuffer[offset + threadId] = sum;
		sum += count;
		offset += blockStride;
	}

	int lane = threadId & (D_BANK_COUNT_GPU - 1);
	for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
	{
		int radixPrefixScanRegTemp = __shfl_up_sync(0xffffffff, sum, n, D_BANK_COUNT_GPU);
		if (lane >= n)
		{
			sum += radixPrefixScanRegTemp;
		}
	}
	localPrefixScan[threadId + 1] = sum;

	int scale = 0;
	__syncthreads();
	for (int segment = blockStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
	{
		if (threadId < blockStride / 2)
		{
			int baseBank = threadId >> (D_LOG_BANK_COUNT_GPU + scale);
			int baseIndex = (baseBank << (D_LOG_BANK_COUNT_GPU + scale + 1)) + (1 << (D_LOG_BANK_COUNT_GPU + scale)) + 1 - 1;
			int bankIndex = threadId & ((1 << (D_LOG_BANK_COUNT_GPU + scale)) - 1);
			int scanIndex = baseIndex + bankIndex + 1;

			localPrefixScan[scanIndex] += localPrefixScan[baseIndex];
		}
		scale++;
		__syncthreads();
	}
	scansBuffer[offset + threadId] = localPrefixScan[threadId];
#endif
}

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaCountItemsUnordered(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int radixCountBuffer[D_DEVICE_UNORDERED_MAX_RADIX_SIZE];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
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
			T item(input[index]);
			output[index] = item;
			int radix = getRadix(item);
			atomicAdd(&radixCountBuffer[radix], 1);
		}
		bashSize += blockStride;
	}

	__syncthreads();
	if (threadId < radixStride)
	{
		int index = threadId + radixStride * blockIndex;
		scansBuffer[index] = radixCountBuffer[threadId];
	}
}

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

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSortUnOrdered(ndCudaContextImplement* const context, const ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix)
{
	ndAssessor<T> output(dst);
	ndAssessor<T> input(src);
	ndAssessor<int> prefixScanBuffer(context->GetPrefixScanBuffer());
	ndKernelParams params(context->GetDevice(), D_DEVICE_UNORDERED_SORT_BLOCK_SIZE, src.GetCount());

	ndAssert(src.GetCount() == dst.GetCount());
	ndAssert((1 << exponentRadix) <= D_DEVICE_UNORDERED_MAX_RADIX_SIZE);

	int radixStride = 1 << exponentRadix;
	ndCudaCountItemsUnordered << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, output, prefixScanBuffer, radixStride, evaluateRadix);
	ndCudaAddPrefixUnordered << <1, radixStride, 0 >> > (params, input, prefixScanBuffer, evaluateRadix);
	ndCudaMergeBucketsUnOrdered << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, output, input, prefixScanBuffer, radixStride, evaluateRadix);
}

#endif