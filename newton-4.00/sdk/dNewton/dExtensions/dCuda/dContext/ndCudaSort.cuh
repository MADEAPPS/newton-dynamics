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

#define D_DEVICE_SORT_BLOCK_SIZE	(1<<8)
//#define D_DEVICE_SORT_BLOCK_SIZE	(1<<9)
//#define D_DEVICE_SORT_BLOCK_SIZE	(1<<10)
#define D_DEVICE_MAX_RADIX_SIZE		(1<<8)

#if D_DEVICE_MAX_RADIX_SIZE > D_DEVICE_SORT_BLOCK_SIZE
	#error counting sort diget larger that block
#endif

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* const context, const ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix);

// *****************************************************************
// 
// support function implementation 
//
// *****************************************************************
template <typename T, typename SortKeyPredicate>
__global__ void ndCudaAddPrefix(const ndKernelParams params, const ndAssessor<T> dommy, ndAssessor<int> scansBuffer, SortKeyPredicate getRadix)
{
	__shared__  int localPrefixScan[D_DEVICE_MAX_RADIX_SIZE / 2 + D_DEVICE_MAX_RADIX_SIZE + 1];

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
	__shared__  int radixCountBuffer[D_DEVICE_MAX_RADIX_SIZE];

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

#define D_USE_LOCAL_BITONIC_SORT

#ifdef D_USE_LOCAL_BITONIC_SORT


#define D_LOG_BANK_COUNT_GPU	5
#define D_BANK_COUNT_GPU		(1<<D_LOG_BANK_COUNT_GPU)

template <int size>
class cuBankFreeArray
{
	public:
	__device__ cuBankFreeArray()
	{
	}

	__device__ int& operator[] (int address)
	{
		int low = address & -D_BANK_COUNT_GPU;
		int high = address >> D_LOG_BANK_COUNT_GPU;
		return m_array[high][low];
	}

	int m_array[(size + D_BANK_COUNT_GPU - 1) >> D_LOG_BANK_COUNT_GPU][D_BANK_COUNT_GPU + 1];
};

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
	//__shared__  int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__  int radixPrefixCount[D_DEVICE_MAX_RADIX_SIZE];
	__shared__  int radixPrefixStart[D_DEVICE_MAX_RADIX_SIZE];
	__shared__  cuBankFreeArray<D_DEVICE_SORT_BLOCK_SIZE> sortedRadix;
	//__shared__  int radixPrefixScan[D_DEVICE_MAX_RADIX_SIZE / 2 + D_DEVICE_MAX_RADIX_SIZE + 1];
	__shared__  cuBankFreeArray<D_DEVICE_MAX_RADIX_SIZE> radixPrefixScan;

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
	int blockIndex = blockIdx.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		int a = scansBuffer[radixBase + threadId];
		int b = scansBuffer[radixPrefixOffset + threadId];
		radixPrefixStart[threadId] = a + b;
		radixPrefixScan[threadId] = 0;
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


		//if (threadId < radixStride)
		//{
		//	radixPrefixScan[radixStride / 2 + threadId + 1] = radixPrefixCount[threadId];
		//}
		//for (int k = 1; k < radixStride; k = k << 1)
		//{
		//	int sum;
		//	__syncthreads();
		//	if (threadId < radixStride)
		//	{
		//		int a = radixPrefixScan[radixStride / 2 + threadId];
		//		int b = radixPrefixScan[radixStride / 2 + threadId - k];
		//		sum = a + b;
		//	}
		//	__syncthreads();
		//	if (threadId < radixStride)
		//	{
		//		radixPrefixScan[radixStride / 2 + threadId] = sum;
		//	}
		//}

		radixPrefixScan[threadId] = 1;
		int radixBit = 0;
		int xxxxx = 64;

		//__shared__  int xxxxxxxxxxxxxx[D_DEVICE_MAX_RADIX_SIZE];
		for (int k = 1; k < 64; k = k * 2)
		{
			radixBit++;

			int base = (threadId & -32) * 2;
			int bankIndex = threadId & 0x1f;

			//xxxxxxxxxxxxxx[threadId] = -1;
			if ((threadId < radixStride / 2) && (bankIndex < (xxxxx >> radixBit)))
			{
				int id1 = ((bankIndex + 1) << radixBit) - 1;
				int id0 = id1 - (1 << (radixBit - 1));
				radixPrefixScan[base + id1] += radixPrefixScan[base + id0];
				//printf("thread=%d id0=%d id1=%d bit=%d\n", threadId, id0, id1, radixBit);
				//printf("thread=%d bit=%d base=%d id0=%d \n", threadId, radixBit, base, id0);
				//xxxxxxxxxxxxxx[threadId] = id0;
				//printf("thread=%d bit=%d base=%d id0=%d \n", threadId, radixBit, base, id0);
			}

			//__syncthreads();
			//if (threadId == 0)
			//{
			//	//printf("thread=%d bit=%d base=%d\n", k, radixBit, base);
			//	printf("bit=%d\n", radixBit);
			//	for (int base = 0; base < radixStride/2; base += 32)
			//	{
			//		for (int k = 0; k < 32; ++k)
			//		{
			//			printf("%d ", xxxxxxxxxxxxxx[base + k]);
			//		}
			//		printf("\n");
			//	}
			//	printf("\n");
			//}
			//__syncthreads();
		}
		//__syncthreads();

		//radixPrefixScan[xxxxx - 1] = 0;
		////for (int k = 1; k < xxxxx; k = k * 2)
		//for (int k = 1; k < 2; k = k * 2)
		//{
		//	int base = (threadId & -32) * 2;
		//	int bankIndex = threadId & 0x1f;
		//	int id1 = ((bankIndex + 1) << radixBit) - 1;
		//	int id0 = id1 - (1 << (radixBit - 1));
		//	
		//	printf("thread=%d id0=%d id1=%d bit=%d\n", threadId, id0, id1, radixBit);
		//	//int a = radixPrefixScan[id1];
		//	//int b = radixPrefixScan[id0] + a;
		//	//
		//	//radixPrefixScan[id0] = a;
		//	//radixPrefixScan[id1] = b;
		//
		//	radixBit--;
		//}


		if (threadId == 0)
		{
			for (int k = 0; k < radixStride; ++k)
			{
				printf("thread(%d)=%d\n", k, radixPrefixScan[k]);
			}
		}

/*
		//int value = 1;
		//int laneId = threadIdx.x & 0x1f;
		//int size___ = 32;
		//for (int i = 1; i <= size___; i *= 2)
		//{
		//	int n = __shfl_up_sync(0xffffffff, value, i, size___);
		//	if ((laneId & (size___ - 1)) >= i)
		//	{
		//		value += n;
		//	}
		//}
		//radixPrefixScan[threadIdx.x] = value;



#if 1
		for (int k = 1; k < blockStride; k = k << 1)
		{
			for (int j = k; j > 0; j = j >> 1)
			{
				if (threadId < blockStride / 2)
				{
					int highMask = -j;
					int lowMask = ~highMask;
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
#else		
		int id0 = threadId;
		for (int k = 2; k <= blockStride; k = k << 1)
		{
			for (int j = k >> 1; j > 0; j = j >> 1)
			{
				int id1 = id0 ^ j;
		
				if (id1 > id0)
				{
					const int a = sortedRadix[id0];
					const int b = sortedRadix[id1];
					const int mask0 = -(id0 & k);
					const int mask1 = -(a > b);
					const int mask2 = (mask0 ^ mask1) & 0x80000000;
					if (mask2)
					{
						sortedRadix[id0] = b;
						sortedRadix[id1] = a;
					}
				}
				__syncthreads();
			}
		}
#endif

		if (index < input.m_size)
		{
			int keyValue = sortedRadix[threadId];
			int keyHigh = keyValue >> 16;
			int keyLow = keyValue & 0xffff;
			int dstOffset1 = radixPrefixStart[keyHigh];
			int dstOffset0 = threadId - radixPrefixScan[radixStride / 2 + keyHigh];
			output[dstOffset0 + dstOffset1] = cachedItems[keyLow];
		}
		__syncthreads();
		if (threadId < radixStride)
		{
			radixPrefixStart[threadId] += radixPrefixCount[threadId];
		}
*/
		bashSize += blockStride;
	}
}

#else

template <typename T, typename SortKeyPredicate>
__global__ void ndCudaMergeBuckets(const ndKernelParams params, const ndAssessor<T> input, ndAssessor<T> output, const ndAssessor<int> scansBuffer, int radixStride, SortKeyPredicate getRadix)
{
	__shared__  T cachedItems[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__  int sortedRadix[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__  int dstLocalOffset[D_DEVICE_SORT_BLOCK_SIZE];
	__shared__  int radixPrefixCount[D_DEVICE_MAX_RADIX_SIZE];
	__shared__  int radixPrefixStart[D_DEVICE_MAX_RADIX_SIZE];
	__shared__  int radixPrefixScan[D_DEVICE_SORT_BLOCK_SIZE / 2 + D_DEVICE_SORT_BLOCK_SIZE + 1];

	int threadId = threadIdx.x;
	int blockStride = blockDim.x;
	int blockIndex = blockIdx.x;
	int radixBase = blockIndex * radixStride;
	int radixPrefixOffset = params.m_kernelCount * radixStride;
	int bashSize = params.m_blocksPerKernel * params.m_workGroupSize * blockIndex;

	if (threadId < radixStride)
	{
		int a = scansBuffer[radixBase + threadId];
		int b = scansBuffer[radixPrefixOffset + threadId];
		radixPrefixStart[threadId] = a + b;
	}

	for (int i = 0; i < params.m_blocksPerKernel; ++i)
	{
		if (threadId < radixStride)
		{
			radixPrefixCount[threadId] = 0;
		}
		__syncthreads();

		int radix = radixStride - 1;
		int index = bashSize + threadId;
		if (index < input.m_size)
		{
			cachedItems[threadId] = input[index];
			radix = getRadix(cachedItems[threadId]);
		}
		atomicAdd(&radixPrefixCount[radix], 1);
		sortedRadix[threadId] = (threadId << 16) + radix;
		radixPrefixScan[threadId] = 0;

		int bit = 1;
		__syncthreads();
		for (int j = blockStride; j; j = j >> 1)
		{
			int keyReg = sortedRadix[threadId];
			int test = keyReg & bit;
			int mask = test ? 1 : 0;
			dstLocalOffset[threadId] = mask;
			radixPrefixScan[blockStride / 2 + threadId + 1] = mask ? 1 << 16 : 1;

			for (int k = 1; k < blockStride; k = k << 1)
			{
				__syncthreads();
				int a = radixPrefixScan[blockStride / 2 + threadId];
				int b = radixPrefixScan[blockStride / 2 + threadId - k];
				int sumReg = a + b;
				
				__syncthreads();
				radixPrefixScan[blockStride / 2 + threadId] = sumReg;
			}
			__syncthreads();

			int base = (radixPrefixScan[blockStride / 2 + blockStride] + radixPrefixScan[blockStride / 2 + blockStride - 1]) << 16;
			__syncthreads();

			int a = radixPrefixScan[blockStride / 2 + threadId] & 0xffff;
			int b = (radixPrefixScan[blockStride / 2 + threadId] + base) >> 16;
			dstLocalOffset[threadId] = dstLocalOffset[threadId] ? b : a;
			__syncthreads();

			int dstIndex = dstLocalOffset[threadId];
			sortedRadix[dstIndex] = keyReg;
			bit = bit << 1;
			__syncthreads();
		}

		if (threadId < radixStride)
		{
			radixPrefixScan[radixStride / 2 + threadId + 1] = radixPrefixCount[threadId];
		}
		__syncthreads();

		for (int k = 1; k < radixStride; k = k << 1)
		{
			int sum;
			__syncthreads();
			if (threadId < radixStride)
			{
				int a = radixPrefixScan[radixStride / 2 + threadId];
				int b = radixPrefixScan[radixStride / 2 + threadId - k];
				sum = a + b;
			}
			__syncthreads();
			if (threadId < radixStride)
			{
				radixPrefixScan[radixStride / 2 + threadId] = sum;
			}
		}
		__syncthreads();

		if (index < input.m_size)
		{
		//	int keyValue = sortedRadix[threadId];
		//	int keyHigh = keyValue >> 16;
		//	int keyLow = keyValue & 0xffff;
		//	int dstOffset1 = radixPrefixStart[keyHigh];
		//	int dstOffset0 = threadId - radixPrefixScan[radixStride / 2 + keyHigh];
		//	output[dstOffset0 + dstOffset1] = cachedItems[keyLow];
		}
		__syncthreads();

		if (threadId < radixStride)
		{
			radixPrefixStart[threadId] += radixPrefixCount[threadId];
		}
		bashSize += blockStride;
	}
}

#endif

template <class T, int exponentRadix, typename ndEvaluateRadix>
void ndCountingSort(ndCudaContextImplement* const context, const ndCudaDeviceBuffer<T>& src, ndCudaDeviceBuffer<T>& dst, ndEvaluateRadix evaluateRadix)
{
	ndAssessor<T> output(dst);
	const ndAssessor<T> input(src);
	ndAssessor<int> prefixScanBuffer(context->GetPrefixScanBuffer());
	ndKernelParams params(context->GetDevice(), D_DEVICE_SORT_BLOCK_SIZE, src.GetCount());

	ndAssert(src.GetCount() == dst.GetCount());
	ndAssert((1 << exponentRadix) <= D_DEVICE_MAX_RADIX_SIZE);

	int radixStride = 1 << exponentRadix;
	ndCudaCountItems << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, prefixScanBuffer, radixStride, evaluateRadix);
	ndCudaAddPrefix << <1, radixStride, 0 >> > (params, input, prefixScanBuffer, evaluateRadix);
	ndCudaMergeBuckets << <params.m_kernelCount, params.m_workGroupSize, 0 >> > (params, input, output, prefixScanBuffer, radixStride, evaluateRadix);
}

#endif