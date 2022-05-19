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
#include <ndNewtonStdafx.h>

#include "cuIntrinsics.h"
#include "cuPrefixScan.h"
#include "ndCudaContext.h"
#include "cuSortBodyAabbCells.h"

#define D_SIMPLE_PREFIX_SCAN

//static __global__ void cuTest0(const cuSceneInfo& info, int digit)
//{
//}
//
//__global__ void cuTest1(const cuSceneInfo& info, int digit, cudaStream_t stream)
//{
//	cuTest0 << <1, 1, 0, stream >> > (info, digit);
//}

//void BitonicSort(int* arr, int D_THREADS_PER_BLOCK)
//{
//	//for (int i = 0; i < D_THREADS_PER_BLOCK * 4; i++)
//	//{
//	//	arr[i] = (D_THREADS_PER_BLOCK << D_THREADS_PER_BLOCK_BITS) + i;
//	//}
//	//arr[0] = (0 << D_THREADS_PER_BLOCK_BITS) + 0;
//	//arr[1] = (1 << D_THREADS_PER_BLOCK_BITS) + 1;
//	//arr[2] = (0 << D_THREADS_PER_BLOCK_BITS) + 2;
//	//arr[3] = (1 << D_THREADS_PER_BLOCK_BITS) + 3;
//
//	int zzzz = 0;
//	for (int k = 2; k <= D_THREADS_PER_BLOCK; k *= 2)
//	{
//		for (int j = k / 2; j > 0; j /= 2)
//		{
//			zzzz++;
//			for (int i = 0; i < D_THREADS_PER_BLOCK; i++)
//			{
//				const int l = i ^ j;
//				if (l > i)
//				{
//					const int mask0 = (-(i & k)) >> 31;
//					const int mask1 = (-(arr[i] > arr[l])) >> 31;
//					const int mask = mask0 ^ mask1;
//					const int a = arr[i];
//					const int b = arr[l];
//					arr[i] = b & mask | a & ~mask;
//					arr[l] = a & mask | b & ~mask;
//				}
//			}
//		}
//	}
//	zzzz *= 1;
//}

inline unsigned __device__ cuCountingSortEvaluateGridCellKey(const cuBodyAabbCell& cell, int digit)
{
	const unsigned key = cell.m_key;
	const unsigned mask = (1 << D_AABB_GRID_CELL_BITS) - 1;
	const unsigned value = mask & (key >> (digit * D_AABB_GRID_CELL_BITS));
	return value;
};

__global__ void cuCountingSortCountGridCells(const cuSceneInfo& info, int digit)
{
	__shared__  unsigned cacheBuffer[1<<D_AABB_GRID_CELL_BITS];
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned cellCount = info.m_bodyAabbCell.m_size - 1;
		const unsigned blocks = (cellCount + blockDim.x - 1) / blockDim.x;
		if (blockId < blocks)
		{
			const unsigned threadId = threadIdx.x;

			cacheBuffer[threadId] = 0;
			unsigned* histogram = info.m_histogram.m_array;
			const cuBodyAabbCell* src = (digit & 1) ? info.m_bodyAabbCell.m_array : info.m_bodyAabbCellScrath.m_array;
			
			const unsigned index = threadId + blockDim.x * blockId;
			if (index < cellCount)
			{
				const unsigned key = cuCountingSortEvaluateGridCellKey(src[index], digit);
				atomicAdd(&cacheBuffer[key], 1);
			}
			__syncthreads();

			const unsigned dstBase = blockDim.x * blockId;
			histogram[dstBase + threadId] = cacheBuffer[threadId];
		}
	}
}

__global__ void cuCountingSortShuffleGridCells(const cuSceneInfo& info, int digit)
{
	__shared__  long long cachedCells[1 << D_AABB_GRID_CELL_BITS];
	__shared__  unsigned cacheSortedKey[1 << D_AABB_GRID_CELL_BITS];
	__shared__  unsigned cacheBaseOffset[1 << D_AABB_GRID_CELL_BITS];
	__shared__  unsigned cacheKeyPrefix[(1 << D_AABB_GRID_CELL_BITS) / 2 + (1 << D_AABB_GRID_CELL_BITS) + 1];
	__shared__  unsigned cacheItemCount[(1 << D_AABB_GRID_CELL_BITS) / 2 + (1 << D_AABB_GRID_CELL_BITS) + 1];

	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned cellCount = info.m_bodyAabbCell.m_size - 1;
		const unsigned blocks = (cellCount + blockDim.x - 1) / blockDim.x;
		if (blockId < blocks)
		{
			const unsigned threadId = threadIdx.x;
			const unsigned* histogram = info.m_histogram.m_array;
			const cuBodyAabbCell* src = (digit & 1) ? info.m_bodyAabbCell.m_array : info.m_bodyAabbCellScrath.m_array;
			cuBodyAabbCell* dst = (digit & 1) ? info.m_bodyAabbCellScrath.m_array : info.m_bodyAabbCell.m_array;

			const unsigned index = threadId + blockDim.x * blockId;
			const unsigned lastRoadOffset = blocks * (1 << D_AABB_GRID_CELL_BITS);
			
			cacheSortedKey[threadId] = ((1<< D_AABB_GRID_CELL_BITS) << 16) | threadId;
			if (index < cellCount)
			{
				cuBodyAabbCell cell;
				const long long value = src[index].m_value;
				cell.m_value = value;
				cachedCells[threadId] = value;
				const unsigned key = cuCountingSortEvaluateGridCellKey(cell, digit);
				cacheSortedKey[threadId] = (key << 16) | threadId;
			}
			__syncthreads();

			cacheKeyPrefix[threadId] = 0;
			cacheItemCount[threadId] = 0;

			const unsigned prefixBase = (1 << D_AABB_GRID_CELL_BITS) / 2;
			const unsigned srcOffset = blockId * (1 << D_AABB_GRID_CELL_BITS);
			cacheBaseOffset[threadId] = histogram[srcOffset + threadId];
			cacheKeyPrefix[prefixBase + 1 + threadId] = histogram[lastRoadOffset + threadId];
			cacheItemCount[prefixBase + 1 + threadId] = histogram[srcOffset + (1 << D_AABB_GRID_CELL_BITS) + threadId] - cacheBaseOffset[threadId];
						
			const int threadId0 = threadId;
			for (int k = 2; k <= (1 << D_AABB_GRID_CELL_BITS); k *= 2)
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

			for (int i = 1; i < (1 << D_AABB_GRID_CELL_BITS); i = i << 1)
			{
				const unsigned prefixSum = cacheKeyPrefix[prefixBase + threadId] + cacheKeyPrefix[prefixBase - i + threadId];
				const unsigned countSum = cacheItemCount[prefixBase + threadId] + cacheItemCount[prefixBase - i + threadId];
				__syncthreads();
				cacheKeyPrefix[prefixBase + threadId] = prefixSum;
				cacheItemCount[prefixBase + threadId] = countSum;
				__syncthreads();
			}
			
			if (index < cellCount)
			{
				const unsigned keyValue = cacheSortedKey[threadId];
				const unsigned key = keyValue >> 16;
				const unsigned threadIdBase = cacheItemCount[prefixBase + key];

				const unsigned dstOffset0 = threadId - threadIdBase;
				const unsigned dstOffset1 = cacheKeyPrefix[prefixBase + key] + cacheBaseOffset[key];
				dst[dstOffset0 + dstOffset1].m_value = cachedCells[keyValue & 0xffff];
			}
		}
	}
}

#ifdef D_SIMPLE_PREFIX_SCAN

__global__ void cuCountingSortBodyCellsPrefixScan(const cuSceneInfo& info, unsigned histogramGridBlockSize)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned blockCount = (itemsCount + histogramGridBlockSize - 1) / histogramGridBlockSize;
		
		unsigned offset = blockId * blockDim.x;
		unsigned* histogram = info.m_histogram.m_array;

		unsigned sum = 0;
		for (int i = 0; i < blockCount; i++)
		{
			const unsigned count = histogram[offset + threadId];
			histogram[offset + threadId] = sum;
			sum += count;
			offset += histogramGridBlockSize;
		}
		histogram[offset + threadId] = sum;
	}
}

#else
__global__ void cuCountingSortHillisSteelePaddBuffer(const cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned threadId = threadIdx.x;

		unsigned* histogram = info.m_histogram.m_array;
		if (blockId >= D_PREFIX_SCAN_PASSES)
		{
			histogram[threadId] = 0;
		}
		else
		{
			const unsigned cellCount = blockDim.x + info.m_bodyAabbCell.m_size - 1;
			const unsigned blocks = (cellCount + blockDim.x - 1) / blockDim.x;
			const unsigned blocksFrac = blocks & (D_PREFIX_SCAN_PASSES - 1);
			if (blockId >= blocksFrac)
			{
				const unsigned superBlockBase = (blocks + D_PREFIX_SCAN_PASSES - 1) / D_PREFIX_SCAN_PASSES - 1;
				const unsigned offset = (superBlockBase * D_PREFIX_SCAN_PASSES + blockId) * blockDim.x;

				histogram[offset + threadId] = 0;
			}
		}
	}
}

__global__ void cuCountingSortHillisSteelePrefixScanAddBlocks(cuSceneInfo& info, int bit)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned superBlockSize = D_PREFIX_SCAN_PASSES * blockDim.x;
		const unsigned alignedSuperBlockBase = superBlockSize * ((itemsCount + superBlockSize - 1) / superBlockSize);
		const unsigned blocks = ((alignedSuperBlockBase + blockDim.x - 1) / blockDim.x);
		if (blockId < blocks)
		{
			const unsigned power = 1 << (bit + 1);
			const unsigned blockFrac = blockId & (power - 1);
			if (blockFrac >= (power >> 1))
			{
				const unsigned threadId = threadIdx.x;
				const unsigned dstIndex = blockDim.x * blockId;
				const unsigned srcIndex = blockDim.x * (blockId - blockFrac + (power >> 1) - 1);

				unsigned* histogram = info.m_histogram.m_array;
				const unsigned value = histogram[srcIndex + threadId];
				histogram[dstIndex + threadId] += value;
			}
		}
	}
}

__global__ void cuCountingSortHillisSteelePrefixScanAddBlocksFinal(cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned superBlockSize = D_PREFIX_SCAN_PASSES * blockDim.x;
		const unsigned alignedSuperBlockBase = superBlockSize * ((itemsCount + superBlockSize - 1) / superBlockSize);
		const unsigned blocks = ((alignedSuperBlockBase + blockDim.x - 1) / blockDim.x);
		if (blockId < blocks)
		{
			const unsigned power = 1 << D_PREFIX_SCAN_PASSES_BITS;
			const unsigned blockFrac = blockId & (power - 1);
			if (blockFrac >= (power >> 1))
			{
				const unsigned threadId = threadIdx.x;
				const unsigned dstIndex = blockDim.x * blockId;
				const unsigned srcIndex = blockDim.x * (blockId - blockFrac + (power >> 1) - 1);

				unsigned* histogram = info.m_histogram.m_array;
				const unsigned value = histogram[srcIndex + threadId];
				histogram[dstIndex + threadId] += value;

				if (blockFrac == (power - 1))
				{
					const unsigned blockSumBase = alignedSuperBlockBase + blockDim.x * (blockId/D_PREFIX_SCAN_PASSES);
					histogram[blockSumBase + threadId] = value;
				}
			}
		}
	}
}

__global__ void cuCountingSortBodyCellsPrefixScan(const cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned threadId = threadIdx.x;
		const unsigned itemsCount = info.m_histogram.m_size;
		const unsigned superBlockSize = D_PREFIX_SCAN_PASSES * blockDim.x;
		const unsigned superBlockCount = (itemsCount + superBlockSize - 1) / superBlockSize;
		const unsigned alignedSuperBlockBase = superBlockSize * superBlockCount;
		
		unsigned offset = blockId * blockDim.x + superBlockSize;
		unsigned* histogram = info.m_histogram.m_array;
		//__shared__  unsigned superBlockOffset_;
		//__shared__  unsigned prefixScanSuperBlockAlign_;
		//superBlockOffset_ = superBlockOffset;
		//prefixScanSuperBlockAlign_ = prefixScanSuperBlockAlign;
		//__syncthreads();

		unsigned value = 0;
		unsigned blockSumBase = alignedSuperBlockBase;
		for (int i = 1; i < superBlockCount; i++)
		{
			value += histogram[blockSumBase + threadId];
			blockSumBase += blockDim.x;

			histogram[offset + threadId] += value;
			offset += superBlockSize;
		}
	}
}

__global__ void cuCountingSortCaculatePrefixOffset(const cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		//const int blockId = blockIdx.x;
		//const int cellCount = info.m_bodyAabbCell.m_size - 1;
		//const int blocks = (cellCount + D_AABB_GRID_CELL_SORT_BLOCK_SIZE - 1) / D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
		//
		//const int threadId = threadIdx.x;
		//unsigned* histogram = info.m_histogram.m_array;
		//
		//int src = blockId * D_AABB_GRID_CELL_DIGIT_BLOCK_SIZE;
		//int dst = blocks * D_AABB_GRID_CELL_SORT_BLOCK_SIZE + src;
		//int sum = histogram[2 * blocks * D_AABB_GRID_CELL_SORT_BLOCK_SIZE + src + threadId];
		//
		//for (int i = 0; i < blocks; i++)
		//{
		//	histogram[dst + threadId] = sum;
		//	sum += histogram[src + threadId];
		//	dst += D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
		//	src += D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
		//}
	}
}
#endif


static void CountingSortBodyCells(ndCudaContext* const context, int digit)
{
	cuSceneInfo* const sceneInfo = context->m_sceneInfoCpu;
	cudaStream_t stream = context->m_solverComputeStream;
	cuSceneInfo* const infoGpu = context->m_sceneInfoGpu;

	const unsigned histogramGridBlockSize = (1 << D_AABB_GRID_CELL_BITS);
	const unsigned blocks = (sceneInfo->m_bodyAabbCell.m_capacity + histogramGridBlockSize - 1) / histogramGridBlockSize;

	dAssert(blocks <= context->m_blocksPerKernelCall);

	cuCountingSortCountGridCells << <blocks, histogramGridBlockSize, 0, stream >> > (*infoGpu, digit);

#ifdef D_SIMPLE_PREFIX_SCAN
	const unsigned prefixScansBlocks = histogramGridBlockSize / D_THREADS_PER_BLOCK;
	cuCountingSortBodyCellsPrefixScan << <prefixScansBlocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, histogramGridBlockSize);
#else
	cuCountingSortHillisSteelePaddBuffer << <D_PREFIX_SCAN_PASSES + 1, histogramGridBlockSize, 0, stream >> > (*infoGpu);
	for (ndInt32 i = 0; i < (D_PREFIX_SCAN_PASSES_BITS - 1); i++)
	{
		cuCountingSortHillisSteelePrefixScanAddBlocks << <blocks, histogramGridBlockSize, 0, stream >> > (*infoGpu, i);
	}
	cuCountingSortHillisSteelePrefixScanAddBlocksFinal << <blocks, histogramGridBlockSize, 0, stream >> > (*infoGpu);
	cuCountingSortBodyCellsPrefixScan << <D_PREFIX_SCAN_PASSES, histogramGridBlockSize, 0, stream >> > (*infoGpu);
	//cuCountingSortCaculatePrefixOffset << <radixBlock, D_AABB_GRID_CELL_DIGIT_BLOCK_SIZE, 0, stream >> > (*infoGpu);
#endif

	cuCountingSortShuffleGridCells << <blocks, histogramGridBlockSize, 0, stream >> > (*infoGpu, digit);
}

void CudaBodyAabbCellSortBuffer(ndCudaContext* const context)
{
	//BitonicSort();

	dAssert(context->m_bodyAabbCell.GetCount() <= context->m_histogram.GetCount());
	dAssert(context->m_bodyAabbCell.GetCount() == context->m_bodyAabbCellScrath.GetCount());
	
	CountingSortBodyCells(context, 0);
	CountingSortBodyCells(context, 1);
	CountingSortBodyCells(context, 2);

	
	////auto CountJointBodyPairs = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	//auto XXX = ndMakeObject::ndFunction([]()
	//{
	//
	//});
	//
	////auto square = []()
	////{
	////	//return i * i;
	////};
	//
	//xxxxxxxxx<>(XXX);
}
