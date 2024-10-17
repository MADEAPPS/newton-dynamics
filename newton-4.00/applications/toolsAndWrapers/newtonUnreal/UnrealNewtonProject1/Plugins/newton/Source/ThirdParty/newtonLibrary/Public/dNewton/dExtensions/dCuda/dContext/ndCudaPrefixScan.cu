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

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include "ndCudaContext.h"
#include "ndCudaSceneInfo.h"
#include "ndCudaIntrinsics.h"

#include "ndCudaSort.cuh"
#include "ndCudaPrefixScan.cuh"


#if 0
#define D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE	1024

__global__ void ndCudaHillisSteeleSanityCheck(ndCudaSceneInfo& info)
{
	const unsigned index = threadIdx.x + blockIdx.x * blockDim.x;
	const unsigned* histogram = info.m_histogram.m_array;

	if ((index >= 1) && (index < info.m_histogram.m_size))
	{
		unsigned item0 = histogram[index - 1];
		unsigned item1 = histogram[index - 0];
		if (info.m_frameIsValid && (item0 > item1))
		{
			//printf("block(%d) id:%d (%d %d)\n", blockIdx.x, threadIdx.x, item0, item1);
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
		}
	}
}

__global__ void ndCudaHillisSteeleInternal(ndCudaSceneInfo& info)
{
	__shared__  unsigned cacheBuffer[D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE / 2 + D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE + 1];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId0 = threadIdx.x;
	const unsigned threadId1 = D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE / 2 + threadId0;
	cacheBuffer[threadId0] = 0;
	__syncthreads();
	
	cacheBuffer[threadId1] = 0;
	unsigned* histogram = info.m_histogram.m_array;
	const unsigned index = threadId0 + blockDim.x * blockId;
	if (index < info.m_histogram.m_size)
	{
		cacheBuffer[threadId1] = histogram[index];
	}
	__syncthreads();

	for (int i = 1; i < D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE; i = i << 1)
	{
		int sum = cacheBuffer[threadId1] + cacheBuffer[threadId1 - i];
		__syncthreads();
		cacheBuffer[threadId1] = sum;
		__syncthreads();
	}

	if (index < info.m_histogram.m_size)
	{
		histogram[index] = cacheBuffer[threadId1];
	}
}

__global__ void ndCudaHillisSteeleAddBlocksInternal(ndCudaSceneInfo& info)
{
	const unsigned threadId = threadIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned superBlockCount = (itemsCount + D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE - 1) / D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE;
	
	unsigned* histogram = info.m_histogram.m_array;

	unsigned offset = D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE;
	for (int i = 1; i < superBlockCount; i++)
	{
		const unsigned value = histogram[offset - 1];
		histogram[offset + threadId] += value;
		offset += D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE;
		__syncthreads();
	}
}



/*
__global__ void ndCudaHillisSteelePaddBufferInternal(ndCudaSceneInfo& info)
{
	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned blockStart = blockDim.x * ((itemsCount + blockDim.x - 1) / blockDim.x);

	const unsigned blockOffset = blockId * blockDim.x;
	unsigned* histogram = info.m_histogram.m_array;

	if (blockOffset >= blockStart)
	{
		histogram[blockOffset + threadId] = 0;
	}
}

__global__ void ndCudaHillisSteelePrefixScanAddBlocksInternal(ndCudaSceneInfo& info, int bit)
{
	const unsigned blockId = blockIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockDim.x;
	const unsigned alignedItemsCount = prefixScanSuperBlockAlign * ((itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign);
	const unsigned blocks = ((alignedItemsCount + blockDim.x - 1) / blockDim.x);
	if (blockId < blocks)
	{
		const unsigned power = 1 << (bit + 1);
		const unsigned blockFrac = blockId & (power - 1);
		if (blockFrac >= (power >> 1))
		{
			const unsigned threadId = threadIdx.x;
			const unsigned dstIndex = blockDim.x * blockId;
			const unsigned srcIndex = blockDim.x * (blockId - blockFrac + (power >> 1)) - 1;

			unsigned* histogram = info.m_histogram.m_array;
			const unsigned value = histogram[srcIndex];
			histogram[dstIndex + threadId] += value;
		}
	}
}

__global__ void ndCudaHillisSteelePrefixScanAddBlocksFinalInternal(ndCudaSceneInfo& info)
{
	const unsigned blockId = blockIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockDim.x;
	const unsigned alignedItemsCount = prefixScanSuperBlockAlign * ((itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign);
	const unsigned blocks = ((alignedItemsCount + blockDim.x - 1) / blockDim.x);
	if (blockId < blocks)
	{
		const unsigned power = 1 << D_PREFIX_SCAN_PASSES_BITS;
		const unsigned blockFrac = blockId & (power - 1);
		if (blockFrac >= (power >> 1))
		{
			const unsigned threadId = threadIdx.x;
			const unsigned dstIndex = blockDim.x * blockId;
			const unsigned srcIndex = blockDim.x * (blockId - blockFrac + (power >> 1)) - 1;

			unsigned* histogram = info.m_histogram.m_array;
			const unsigned value = histogram[srcIndex];
			histogram[dstIndex + threadId] += value;

			if (blockFrac == (power - 1))
			{
				__syncthreads();
				if (threadId == (blockDim.x - 1))
				{
					const unsigned dstBlock = blockId / D_PREFIX_SCAN_PASSES;
					const unsigned sum = histogram[blockId * blockDim.x + threadId];
					histogram[alignedItemsCount + dstBlock] = sum;
				}
			}
		}
	}
}

__global__ void ndCudaHillisSteeleAddSupeBlocksInternal(ndCudaSceneInfo& info)
{
	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	const unsigned itemsCount = info.m_histogram.m_size;
	const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockDim.x;
	const unsigned superBlockCount = (itemsCount + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign;

	unsigned* histogram = info.m_histogram.m_array;
	unsigned offset = blockId * blockDim.x + prefixScanSuperBlockAlign;
	const unsigned superBlockOffset = superBlockCount * prefixScanSuperBlockAlign;

	unsigned value = histogram[superBlockOffset];
	for (int i = 1; i < superBlockCount; i++)
	{
		histogram[offset + threadId] += value;
		value += histogram[superBlockOffset + i];
		offset += prefixScanSuperBlockAlign;
	}
}

__global__ void ndCudaHillisSteelePrefixScan(ndCudaSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned threads = info.m_histogram.m_size;
		//const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * blockSize;
		//const unsigned superBlocks = (threads + prefixScanSuperBlockAlign - 1) / prefixScanSuperBlockAlign;
		//const unsigned histogramBlocks = D_PREFIX_SCAN_PASSES * superBlocks;
		const unsigned blocks = (threads + D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE - 1) / D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE;
		if (blocks * D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE > info.m_histogram.m_capacity)
		{
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
			info.m_histogram.m_size = blocks * D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE;
			return;
		}

		ndCudaHillisSteeleInternal << <blocks, D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE, 0 >> > (info);
		//ndCudaHillisSteelePaddBufferInternal << <D_PREFIX_SCAN_PASSES, blockSize, 0 >> > (info);
		//for (int i = 0; i < (D_PREFIX_SCAN_PASSES_BITS - 1); i++)
		//{
		//	ndCudaHillisSteelePrefixScanAddBlocksInternal << <histogramBlocks, blockSize, 0 >> > (info, i);
		//}
		//ndCudaHillisSteelePrefixScanAddBlocksFinalInternal << <histogramBlocks, blockSize, 0 >> > (info);
		//ndCudaHillisSteeleAddSupeBlocksInternal << <D_PREFIX_SCAN_PASSES, blockSize, 0 >> > (info);

		if (blocks > 1)
		{
			ndCudaHillisSteeleInternal << <1, D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE, 0 >> > (info);
		}

#ifdef _DEBUG
		ndCudaHillisSteeleSanityCheck << <blocks, D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE, 0 >> > (info);
#endif
	}
}
*/



__global__ void ndCudaHillisSteelePrefixScan(ndCudaSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned threads = info.m_histogram.m_size;
		const unsigned blocks = (threads + D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE - 1) / D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE;
		if (blocks * D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE > info.m_histogram.m_capacity)
		{
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
			info.m_histogram.m_size = blocks * D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE;
			return;
		}

		ndCudaHillisSteeleInternal << <blocks, D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE, 0 >> > (info);
		if (blocks > 1)
		{
			ndCudaHillisSteeleAddBlocksInternal << <1, D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE, 0 >> > (info);
		}

		#ifdef _DEBUG
		ndCudaHillisSteeleSanityCheck << <blocks, D_HILL_STEELE_PREFIX_SCAN_BLOCK_SIZE, 0 >> > (info);
		#endif
	}
}

#endif


//optimized single block Hillis-Steele prefix scan sum
__global__ void ndCudaAddPrefix(const ndKernelParams params, ndAssessor<int> scanBuffer)
{
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
		int count = scanBuffer[offset + threadId];
		scanBuffer[offset + threadId] = sum;
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
	scanBuffer[offset + threadId] = localPrefixScan[threadId];
}

