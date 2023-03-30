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
#include "ndCudaIntrinsics.h"
#include "ndCudaDeviceBuffer.h"
#include "ndCudaPrefixScan.cuh"


#define D_GPU_SORTING_ALGORITHM		1

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& buffer, ndCudaDeviceBuffer<T>& auxiliaryBuffer, ndEvaluateRadix evaluateRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
#if (D_GPU_SORTING_ALGORITHM == 0)

//optimized Hillis-Steele prefix scan sum
//using a simple bitonic sort with two ways bank conflict
template <typename T, typename SortKeyPredicate>
__global__ void CountAndSortBlockItems(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, ndAssessor<int> scanBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__  int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__  int radixCountBuffer[D_DEVICE_SORT_MAX_RADIX_SIZE];

	int threadId = threadIdx.x;
	int blockIndex = blockIdx.x;
	int blockStride = blockDim.x;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		radixCountBuffer[threadId] = 0;
	}
	__syncthreads();

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = bashSize + threadId;
		int sortedRadixReg = (radixStride << 16);
		if (index < input.m_size)
		{
			const T item(input[index]);
			cachedItems[threadId] = item;
			int radix = getRadix(item);
			atomicAdd(&radixCountBuffer[radix], 1);
			sortedRadixReg = (radix << 16) + threadId;
		}

		sortedRadix[threadId] = sortedRadixReg;
		__syncthreads();

		for (int k = 1; k < blockStride; k = k << 1)
		{
			for (int j = k; j > 0; j = j >> 1)
			{
				int highMask = -j;
				int lowMask = ~highMask;
				if (threadId < blockStride / 2)
				{
					int lowIndex = threadId & lowMask;
					int highIndex = (threadId & highMask) * 2;
				
					int id0 = highIndex + lowIndex;
					int id1 = highIndex + lowIndex + j;
					int oddEven = highIndex & k * 2;
				
					int a = sortedRadix[id0];
					int b = sortedRadix[id1];
				
					int test = a < b;
					int a1 = test ? a : b;
					int b1 = test ? b : a;
				
					int a2 = oddEven ? b1 : a1;
					int b2 = oddEven ? a1 : b1;
				
					sortedRadix[id0] = a2;
					sortedRadix[id1] = b2;
				}
				__syncthreads();
			}
		}

		if (index < input.m_size)
		{
			int keyIndex = sortedRadix[threadId] & 0xffff;
			output[index] = cachedItems[keyIndex];
		}

		bashSize += blockStride;
	}

	__syncthreads();
	if (threadId < radixStride)
	{
		int index = threadId + radixStride * blockIndex;
		scanBuffer[index] = radixCountBuffer[threadId];
	}
}

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scanBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__  int radixPrefixStart[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__  int radixPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
	int blockIndex = blockIdx.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	int radixPrefixStartReg;
	if (threadId < radixStride)
	{
		int a = scanBuffer[radixBase + threadId];
		int b = scanBuffer[radixPrefixOffset + threadId];
		radixPrefixStartReg = a + b;
		radixPrefixStart[threadId] = radixPrefixStartReg;

		if (threadId == 0)
		{
			radixPrefixScan[0] = 0;
		}
	}

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		if (threadId < radixStride)
		{
			radixPrefixCount[threadId] = 0;
		}
		__syncthreads();

		T cachedItemsReg;
		int sortedRadixReg;
		int index = bashSize + threadId;
		int radix = radixStride - 1;
		sortedRadixReg = radix;
		if (index < input.m_size)
		{
			const T item(input[index]);
			cachedItemsReg = item;
			radix = getRadix(item);
			sortedRadixReg = radix;
		}
		atomicAdd(&radixPrefixCount[radix], 1);
		__syncthreads();

		int radixPrefixScanReg;
		int radixPrefixCountReg;
		if (threadId < radixStride)
		{
			radixPrefixCountReg = radixPrefixCount[threadId];
			radixPrefixScanReg = radixPrefixCountReg;
		}

		int lane = threadId & (D_BANK_COUNT_GPU - 1);
		for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
		{
			int radixPrefixScanRegTemp = __shfl_up_sync(0xffffffff, radixPrefixScanReg, n, D_BANK_COUNT_GPU);
			if (lane >= n)
			{
				radixPrefixScanReg += radixPrefixScanRegTemp;
			}
		}

		if (threadId < radixStride)
		{
			if (!(threadId & D_BANK_COUNT_GPU))
			{
				radixPrefixScan[threadId + 1] = radixPrefixScanReg;
			}
		}

		int scale = 0;
		__syncthreads();
		for (int segment = radixStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
		{
			if (threadId < radixStride)
			{
				int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
				int warpBase = threadId & bank;
				if (warpBase)
				{
					int warpSumIndex = threadId & (-warpBase);
					radixPrefixScanReg += radixPrefixScan[warpSumIndex];
					radixPrefixScan[threadId + 1] = radixPrefixScanReg;
				}
			}
			scale++;
			__syncthreads();
		}

		if (index < input.m_size)
		{
			int keyLow = sortedRadixReg;
			int dstOffset1 = radixPrefixStart[keyLow];
			int dstOffset0 = threadId - radixPrefixScan[keyLow];
			output[dstOffset0 + dstOffset1] = cachedItemsReg;
		}

		__syncthreads();
		if (threadId < radixStride)
		{
			radixPrefixStartReg += radixPrefixCountReg;
			radixPrefixStart[threadId] = radixPrefixStartReg;
		}

		bashSize += blockStride;
	}
}

#elif (D_GPU_SORTING_ALGORITHM == 1)
//optimized Hillis-Steele prefix scan sum
//using a two bits counting sort, in theory, not bank conflits
template <typename T, typename SortKeyPredicate>
__global__ void CountAndSortBlockItems(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, ndAssessor<int> scanBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__ T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__ int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__ int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__ int radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1)];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
	int blockIndex = blockIdx.x;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		radixPrefixCount[threadId] = 0;
		if (threadId == 0)
		{
			radixPrefixScan[0] = 0;
			radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1] = 0;
		}
	}
	__syncthreads();

	int lane = threadId & (D_BANK_COUNT_GPU - 1);
	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = bashSize + threadId;
		int radix = radixStride - 1;
		int sortKey = radix;
		if (index < input.m_size)
		{
			const T item(input[index]);
			cachedItems[threadId] = item;
			radix = getRadix(item);
			atomicAdd(&radixPrefixCount[radix], 1);
			sortKey = (threadId << 16) + radix;
		}
		sortedRadix[threadId] = sortKey;
		__syncthreads();

		for (int bit = 0; (1 << (bit * 2)) < radixStride; ++bit)
		{
			int keyReg = sortedRadix[threadId];
			int test = (keyReg >> (bit * 2)) & 0x3;
			int dstLocalOffsetReg = test;
			int bit0 = (test == 0) ? 1 : 0;
			int bit1 = (test == 1) ? 1 << 16 : 0;
			int bit2 = (test == 2) ? 1 : 0;
			int bit3 = (test == 3) ? 1 << 16 : 0;
			int radixPrefixScanReg0 = bit0 + bit1;
			int radixPrefixScanReg1 = bit2 + bit3;

			for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
			{
				int radixPrefixScanRegTemp0 = __shfl_up_sync(0xffffffff, radixPrefixScanReg0, n, D_BANK_COUNT_GPU);
				int radixPrefixScanRegTemp1 = __shfl_up_sync(0xffffffff, radixPrefixScanReg1, n, D_BANK_COUNT_GPU);
				if (lane >= n)
				{
					radixPrefixScanReg0 += radixPrefixScanRegTemp0;
					radixPrefixScanReg1 += radixPrefixScanRegTemp1;
				}
			}

			if (!(threadId & D_BANK_COUNT_GPU))
			{
				radixPrefixScan[threadId + 1] = radixPrefixScanReg0;
				radixPrefixScan[threadId + 1 + D_DEVICE_SORT_BLOCK_SIZE + 1] = radixPrefixScanReg1;
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
				
					radixPrefixScanReg0 += radixPrefixScan[warpSumIndex - 1 + 1];
					radixPrefixScan[threadId + 1] = radixPrefixScanReg0;
				
					radixPrefixScanReg1 += radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + warpSumIndex - 1 + 1];
					radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + threadId + 1] = radixPrefixScanReg1;
				}
			
				scale++;
				__syncthreads();
			}
			
			int sum0 = radixPrefixScan[1 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
			int sum1 = radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
			int base0 = 0;
			int base1 = sum0 & 0xffff;
			int base2 = base1 + (sum0 >> 16);
			int base3 = base2 + (sum1 & 0xffff);
			
			int key0 = radixPrefixScan[threadId];
			int key1 = radixPrefixScan[threadId + D_DEVICE_SORT_BLOCK_SIZE + 1];
			int shift = dstLocalOffsetReg;
			
			int dstIndex = 0;
			dstIndex += (shift == 1) ? base1 + (key0 >> 16) : 0;
			dstIndex += (shift == 3) ? base3 + (key1 >> 16) : 0;
			dstIndex += (shift == 0) ? base0 + (key0 & 0xffff) : 0;
			dstIndex += (shift == 2) ? base2 + (key1 & 0xffff) : 0;
			
			sortedRadix[dstIndex] = keyReg;
			__syncthreads();
		}	

		if (index < input.m_size)
		{
			int keyIndex = sortedRadix[threadId] >> 16;
			output[index] = cachedItems[keyIndex];
		}

		bashSize += blockStride;
	}

	__syncthreads();
	if (threadId < radixStride)
	{
		int index = threadId + radixStride * blockIndex;
		scanBuffer[index] = radixPrefixCount[threadId];
	}
}

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scanBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__  int radixPrefixStart[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__  int radixPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
	int blockIndex = blockIdx.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	int radixPrefixStartReg;
	if (threadId < radixStride)
	{
		int a = scanBuffer[radixBase + threadId];
		int b = scanBuffer[radixPrefixOffset + threadId];
		radixPrefixStartReg = a + b;
		radixPrefixStart[threadId] = radixPrefixStartReg;

		if (threadId == 0)
		{
			radixPrefixScan[0] = 0;
		}
	}

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		if (threadId < radixStride)
		{
			radixPrefixCount[threadId] = 0;
		}
		__syncthreads();
	
		T cachedItemsReg;
		int sortedRadixReg;
		int index = bashSize + threadId;
		int radix = radixStride - 1;
		sortedRadixReg = radix;
		if (index < input.m_size)
		{
			const T item(input[index]);
			cachedItemsReg = item;
			radix = getRadix(item);
			sortedRadixReg = radix;
		}
		atomicAdd(&radixPrefixCount[radix], 1);
		__syncthreads();

		int radixPrefixScanReg;
		int radixPrefixCountReg;
		if (threadId < radixStride)
		{
			radixPrefixCountReg = radixPrefixCount[threadId];
			radixPrefixScanReg = radixPrefixCountReg;
		}
	
		int lane = threadId & (D_BANK_COUNT_GPU - 1);
		for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
		{
			int radixPrefixScanRegTemp = __shfl_up_sync(0xffffffff, radixPrefixScanReg, n, D_BANK_COUNT_GPU);
			if (lane >= n)
			{
				radixPrefixScanReg += radixPrefixScanRegTemp;
			}
		}

		if (threadId < radixStride)
		{
			if (!(threadId & D_BANK_COUNT_GPU))
			{
				radixPrefixScan[threadId + 1] = radixPrefixScanReg;
			}
		}
	
		int scale = 0;
		__syncthreads();
		for (int segment = radixStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
		{
			if (threadId < radixStride)
			{
				int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
				int warpBase = threadId & bank;
				if (warpBase)
				{
					int warpSumIndex = threadId & (-warpBase);
					radixPrefixScanReg += radixPrefixScan[warpSumIndex];
					radixPrefixScan[threadId + 1] = radixPrefixScanReg;
				}
			}
			scale++;
			__syncthreads();
		}
	
		if (index < input.m_size)
		{
			int keyLow = sortedRadixReg;
			int dstOffset1 = radixPrefixStart[keyLow];
			int dstOffset0 = threadId - radixPrefixScan[keyLow];
			output[dstOffset0 + dstOffset1] = cachedItemsReg;
		}
	
		__syncthreads();
		if (threadId < radixStride)
		{
			radixPrefixStartReg += radixPrefixCountReg;
			radixPrefixStart[threadId] = radixPrefixStartReg;
		}
	
		bashSize += blockStride;
	}
}


#elif (D_GPU_SORTING_ALGORITHM == 2)
//optimized Hillis-Steele prefix scan sum
//using a two bits counting sort, in theory, not bank conflits
template <typename T, typename SortKeyPredicate>
__global__ void CountAndSortBlockItems(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, ndAssessor<int> scanBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__ T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__ int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__ int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__ int radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1)];
	__shared__ int skipDigit[D_BANK_COUNT_GPU];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
	int blockIndex = blockIdx.x;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		radixPrefixCount[threadId] = 0;
		if (threadId == 0)
		{
			radixPrefixScan[0] = 0;
			radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1] = 0;
		}
	}
	__syncthreads();

	int lane = threadId & (D_BANK_COUNT_GPU - 1);
	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		int index = bashSize + threadId;
		int radix = radixStride - 1;
		int sortKey = radix;
		if (index < input.m_size)
		{
			const T item(input[index]);
			cachedItems[threadId] = item;
			radix = getRadix(item);
			atomicAdd(&radixPrefixCount[radix], 1);
			sortKey = (threadId << 16) + radix;
		}
		sortedRadix[threadId] = sortKey;

sortedRadix[threadId] = 0x05;
sortedRadix[15] = 0x04;
radix = sortedRadix[threadId];

		__syncthreads();

		int keyTest = sortedRadix[0] & 0xffff;
		//int sortedRadixReg = (~(radix ^ keyTest)) & 0xff;
		int sortedRadixReg = ~(radix ^ keyTest);
		for (int n = D_BANK_COUNT_GPU / 2; n; n = n / 2)
		{
			int radixPrefixScanRegTemp = __shfl_down_sync(0xffffffff, sortedRadixReg, n, D_BANK_COUNT_GPU);
			sortedRadixReg = sortedRadixReg & radixPrefixScanRegTemp;
		}

		if (threadId < D_BANK_COUNT_GPU)
		{
			skipDigit[threadId] = 0xff;
		}
		__syncthreads();
		if ((threadId & (D_BANK_COUNT_GPU - 1)) == 0)
		{
			skipDigit[threadId >> D_LOG_BANK_COUNT_GPU] = sortedRadixReg;
		}
		__syncthreads();

		if (threadId < D_BANK_COUNT_GPU)
		{
			printf("%d %x\n", threadId, skipDigit[threadId]);
			keyTest = skipDigit[threadId];
			for (int n = D_BANK_COUNT_GPU / 2; n; n = n / 2)
			{
				int radixPrefixScanRegTemp = __shfl_down_sync(0xffffffff, keyTest, n, D_BANK_COUNT_GPU);
				keyTest = keyTest & radixPrefixScanRegTemp;
			}

			if (threadId == 0)
			{
				skipDigit[0] = keyTest;
			}
		}
		__syncthreads();
		keyTest = skipDigit[0];

		for (int bit = 0; (1 << (bit * 2)) < radixStride; ++bit)
		{
			int keyReg = sortedRadix[threadId];
			int test = (keyReg >> (bit * 2)) & 0x3;
			int dstLocalOffsetReg = test;
			int bit0 = (test == 0) ? 1 : 0;
			int bit1 = (test == 1) ? 1 << 16 : 0;
			int bit2 = (test == 2) ? 1 : 0;
			int bit3 = (test == 3) ? 1 << 16 : 0;
			int radixPrefixScanReg0 = bit0 + bit1;
			int radixPrefixScanReg1 = bit2 + bit3;

			for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
			{
				int radixPrefixScanRegTemp0 = __shfl_up_sync(0xffffffff, radixPrefixScanReg0, n, D_BANK_COUNT_GPU);
				int radixPrefixScanRegTemp1 = __shfl_up_sync(0xffffffff, radixPrefixScanReg1, n, D_BANK_COUNT_GPU);
				if (lane >= n)
				{
					radixPrefixScanReg0 += radixPrefixScanRegTemp0;
					radixPrefixScanReg1 += radixPrefixScanRegTemp1;
				}
			}

			if (!(threadId & D_BANK_COUNT_GPU))
			{
				radixPrefixScan[threadId + 1] = radixPrefixScanReg0;
				radixPrefixScan[threadId + 1 + D_DEVICE_SORT_BLOCK_SIZE + 1] = radixPrefixScanReg1;
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

					radixPrefixScanReg0 += radixPrefixScan[warpSumIndex - 1 + 1];
					radixPrefixScan[threadId + 1] = radixPrefixScanReg0;

					radixPrefixScanReg1 += radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + warpSumIndex - 1 + 1];
					radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE + 1 + threadId + 1] = radixPrefixScanReg1;
				}

				scale++;
				__syncthreads();
			}

			int sum0 = radixPrefixScan[1 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
			int sum1 = radixPrefixScan[2 * (D_DEVICE_SORT_BLOCK_SIZE + 1) - 1];
			int base0 = 0;
			int base1 = sum0 & 0xffff;
			int base2 = base1 + (sum0 >> 16);
			int base3 = base2 + (sum1 & 0xffff);

			int key0 = radixPrefixScan[threadId];
			int key1 = radixPrefixScan[threadId + D_DEVICE_SORT_BLOCK_SIZE + 1];
			int shift = dstLocalOffsetReg;

			int dstIndex = 0;
			dstIndex += (shift == 1) ? base1 + (key0 >> 16) : 0;
			dstIndex += (shift == 3) ? base3 + (key1 >> 16) : 0;
			dstIndex += (shift == 0) ? base0 + (key0 & 0xffff) : 0;
			dstIndex += (shift == 2) ? base2 + (key1 & 0xffff) : 0;

			sortedRadix[dstIndex] = keyReg;
			__syncthreads();
		}

		if (index < input.m_size)
		{
			int keyIndex = sortedRadix[threadId] >> 16;
			output[index] = cachedItems[keyIndex];
		}

		bashSize += blockStride;
	}

	__syncthreads();
	if (threadId < radixStride)
	{
		int index = threadId + radixStride * blockIndex;
		scanBuffer[index] = radixPrefixCount[threadId];
	}
}

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scanBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  int radixPrefixCount[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__  int radixPrefixStart[D_DEVICE_SORT_MAX_RADIX_SIZE];
	__shared__  int radixPrefixScan[D_DEVICE_SORT_MAX_RADIX_SIZE + 1];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
	int blockIndex = blockIdx.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	int radixPrefixStartReg;
	if (threadId < radixStride)
	{
		int a = scanBuffer[radixBase + threadId];
		int b = scanBuffer[radixPrefixOffset + threadId];
		radixPrefixStartReg = a + b;
		radixPrefixStart[threadId] = radixPrefixStartReg;

		if (threadId == 0)
		{
			radixPrefixScan[0] = 0;
		}
	}

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		if (threadId < radixStride)
		{
			radixPrefixCount[threadId] = 0;
		}
		__syncthreads();

		T cachedItemsReg;
		int sortedRadixReg;
		int index = bashSize + threadId;
		int radix = radixStride - 1;
		sortedRadixReg = radix;
		if (index < input.m_size)
		{
			const T item(input[index]);
			cachedItemsReg = item;
			radix = getRadix(item);
			sortedRadixReg = radix;
		}
		atomicAdd(&radixPrefixCount[radix], 1);
		__syncthreads();

		int radixPrefixScanReg;
		int radixPrefixCountReg;
		if (threadId < radixStride)
		{
			radixPrefixCountReg = radixPrefixCount[threadId];
			radixPrefixScanReg = radixPrefixCountReg;
		}

		int lane = threadId & (D_BANK_COUNT_GPU - 1);
		for (int n = 1; n < D_BANK_COUNT_GPU; n *= 2)
		{
			int radixPrefixScanRegTemp = __shfl_up_sync(0xffffffff, radixPrefixScanReg, n, D_BANK_COUNT_GPU);
			if (lane >= n)
			{
				radixPrefixScanReg += radixPrefixScanRegTemp;
			}
		}

		if (threadId < radixStride)
		{
			if (!(threadId & D_BANK_COUNT_GPU))
			{
				radixPrefixScan[threadId + 1] = radixPrefixScanReg;
			}
		}

		int scale = 0;
		__syncthreads();
		for (int segment = radixStride; segment > D_BANK_COUNT_GPU; segment >>= 1)
		{
			if (threadId < radixStride)
			{
				int bank = 1 << (D_LOG_BANK_COUNT_GPU + scale);
				int warpBase = threadId & bank;
				if (warpBase)
				{
					int warpSumIndex = threadId & (-warpBase);
					radixPrefixScanReg += radixPrefixScan[warpSumIndex];
					radixPrefixScan[threadId + 1] = radixPrefixScanReg;
				}
			}
			scale++;
			__syncthreads();
		}

		if (index < input.m_size)
		{
			int keyLow = sortedRadixReg;
			int dstOffset1 = radixPrefixStart[keyLow];
			int dstOffset0 = threadId - radixPrefixScan[keyLow];
			output[dstOffset0 + dstOffset1] = cachedItemsReg;
		}

		__syncthreads();
		if (threadId < radixStride)
		{
			radixPrefixStartReg += radixPrefixCountReg;
			radixPrefixStart[threadId] = radixPrefixStartReg;
		}

		bashSize += blockStride;
	}
}

#else
	#error implement new local gpu sort and merge algorthm?
#endif


template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* const context, ndCudaDeviceBuffer<T>& buffer, ndCudaDeviceBuffer<T>& auxiliaryBuffer, ndEvaluateRadix evaluateRadix)
{
	ndAssessor<T> assessor0(buffer);
	ndAssessor<T> assessor1(auxiliaryBuffer);
	ndAssessor<int> prefixScanBuffer(context->GetPrefixScanBuffer());
	ndKernelParams params(context->GetDevice(), D_DEVICE_SORT_BLOCK_SIZE, buffer.GetCount());

	ndAssert(buffer.GetCount() == auxiliaryBuffer.GetCount());
	ndAssert((1 << exponentRadix) <= D_DEVICE_SORT_MAX_RADIX_SIZE);

	int radixStride = 1 << exponentRadix;
	CountAndSortBlockItems << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, assessor0, assessor1, prefixScanBuffer, radixStride, evaluateRadix);
	ndCudaAddPrefix <<< 1, radixStride, 0 >>>  (params, prefixScanBuffer);
	ndCudaMergeBuckets << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, assessor1, assessor0, prefixScanBuffer, radixStride, evaluateRadix);
}

#endif