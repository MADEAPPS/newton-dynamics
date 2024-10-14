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
#include "ndCudaPrefixScan.cuh"

#define D_DEVICE_UNORDERED_SORT_BLOCK_SIZE	(1<<10)

#if D_DEVICE_SORT_MAX_RADIX_SIZE > D_DEVICE_UNORDERED_SORT_BLOCK_SIZE
	#error counting sort diget larger that block
#endif

/// <summary>
/// ndCountingSortUnOrdered use for fast unstable sort.
/// since the orted array is unstable, this funtion can not be used as the base for a stable radix sort
/// </summary>
/// <typeparam name="T">item class or type</typeparam>
/// <typeparam name="ndEvaluateRadix">radix sort size in bits</typeparam>
/// <param name="context">cuda context class, for internal temporary scan array and other info</param>
/// <param name="buffer">source input array, data will be sort on this buffer</param>
/// <param name="auxiliaryBuffer">temporary scratch buffer for intemidiate work, must be the same size as buffer</param>
/// <param name="evaluateRadix">lambda function to get radix form items in array</param>
template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSortUnOrdered(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& buffer, ndCudaDeviceBuffer<T>& auxiliaryBuffer, ndEvaluateRadix evaluateRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
#if 0
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaAddPrefixUnordered(const ndKernelParams params, const ndAssessor<T> dommy, ndAssessor<int> scansBuffer, SortKeyPredicate getRadix)
{
	//optimized Hillis-Steele prefix scan sum
	__shared__  int localPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

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

	if (!(threadId & D_BANK_COUNT_GPU))
	{
		localPrefixScan[threadId + 1] = sum;
	}

	int scale = 0;
	__syncthreads();
	for (int segment = blockStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
	{
		int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
		int warpBase = threadId & bank;
		if (warpBase)
		{
			int warpSumIndex = threadId & (-warpBase);
			sum += localPrefixScan[warpSumIndex];
			localPrefixScan[threadId + 1] = sum;
		}
		scale++;
		__syncthreads();
	}
	scansBuffer[offset + threadId] = localPrefixScan[threadId];
}
#endif

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaCountItemsAndCopyUnordered(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	// the bank free template does not seems to make any difference, but I use it anyway.
	//__shared__  int radixCountBuffer[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__ cuBankFreeArray<int, D_DEVICE_SORT_MAX_RADIX_SIZE> radixCountBuffer;

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
	// the bank free template does not seems to make any difference, but I use it anyway.
	//__shared__  int radixCountBuffer[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__ cuBankFreeArray<int, D_DEVICE_SORT_MAX_RADIX_SIZE> radixCountBuffer;
	
	int threadId = threadIdx.x;
	int blockIndex = blockIdx.x;
	int blockStride = blockDim.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		radixCountBuffer[threadId] = scansBuffer[radixPrefixOffset + threadId] + scansBuffer[radixBase + threadId];
	}
	__syncthreads();

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = bashSize + threadId;
		if (index < input.m_size)
		{
			T item(input[index]);
			int radix = getRadix(item);
			int address = atomicAdd(&radixCountBuffer[radix], 1);
			output[address] = item;
		}
		bashSize += blockStride;
	}
}

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSortUnOrdered(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& buffer, ndCudaDeviceBuffer<T>& auxiliaryBuffer, ndEvaluateRadix evaluateRadix)
{
	ndAssessor<T> assessor0(buffer);
	ndAssessor<T> assessor1(auxiliaryBuffer);
	ndAssessor<int> prefixScanBuffer(context->GetPrefixScanBuffer());
	ndKernelParams params(context->GetDevice(), D_DEVICE_UNORDERED_SORT_BLOCK_SIZE, buffer.GetCount());

	ndAssert(buffer.GetCount() == auxiliaryBuffer.GetCount());
	ndAssert((1 << exponentRadix) <= D_DEVICE_SORT_MAX_RADIX_SIZE);

	int radixStride = 1 << exponentRadix;
	ndCudaCountItemsAndCopyUnordered << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, assessor0, assessor1, prefixScanBuffer, radixStride, evaluateRadix);
	//ndCudaAddPrefixUnordered << <1, radixStride, 0 >> > (params, input, prefixScanBuffer, evaluateRadix);
	ndCudaAddPrefix << < 1, radixStride, 0 >> > (params, prefixScanBuffer);
	ndCudaMergeBucketsUnOrdered << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, assessor1, assessor0, prefixScanBuffer, radixStride, evaluateRadix);
}

#endif