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

#include "cuIntrisics.h"
#include "cuPrefixScan.h"
#include "ndCudaContext.h"
#include "cuSortBodyAabbCells.h"

//#define D_AABB_GRID_CELL_DIGIT_BLOCK_SIZE	256
//#define D_AABB_GRID_CELL_SORT_BLOCK_SIZE	(1<<D_AABB_GRID_CELL_BITS)

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
//	for (int i = 0; i < D_THREADS_PER_BLOCK * 4; i++)
//	{
//		arr[i] = (D_THREADS_PER_BLOCK << D_THREADS_PER_BLOCK_BITS) + i;
//	}
//
//	arr[0] = (0 << D_THREADS_PER_BLOCK_BITS) + 0;
//	arr[1] = (1 << D_THREADS_PER_BLOCK_BITS) + 1;
//	arr[2] = (0 << D_THREADS_PER_BLOCK_BITS) + 2;
//	arr[3] = (1 << D_THREADS_PER_BLOCK_BITS) + 3;
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
//	arr[D_THREADS_PER_BLOCK - 1] = 0;
//}

inline unsigned __device__ cuCountingSortEvaluateGridCellKey(const cuBodyAabbCell& cell, int digit)
{
	unsigned key = cell.m_key;
	unsigned mask = (1 << D_AABB_GRID_CELL_BITS) - 1;
	unsigned value = mask & (key >> (digit * D_AABB_GRID_CELL_BITS));
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
			
			unsigned dstBase = blockDim.x * blockId + blockDim.x;
			histogram[dstBase + threadId] = cacheBuffer[threadId];
		}
	}
}

__global__ void cuCountingSortShuffleGridCells(const cuSceneInfo& info, int digit)
{
	//__shared__  long long cachedCells[D_AABB_GRID_CELL_SORT_BLOCK_SIZE];
	//__shared__  unsigned cacheSortedKey[D_AABB_GRID_CELL_SORT_BLOCK_SIZE];
	//__shared__  int cacheBufferAdress[D_AABB_GRID_CELL_SORT_BLOCK_SIZE];
	//__shared__  int cacheKeyPrefix[D_AABB_GRID_CELL_SORT_BLOCK_SIZE / 2 + D_AABB_GRID_CELL_SORT_BLOCK_SIZE + 1];
	//
	//if (info.m_frameIsValid)
	//{
	//	const int blockId = blockIdx.x;
	//	const int cellCount = info.m_bodyAabbCell.m_size - 1;
	//
	//	const int blocks = (cellCount + D_AABB_GRID_CELL_SORT_BLOCK_SIZE - 1) / D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
	//	if (blockId < blocks)
	//	{
	//		const int threadId = threadIdx.x;
	//		const unsigned* histogram = info.m_histogram.m_array;
	//
	//		const int index = threadId + blockDim.x * blockId;
	//		const cuBodyAabbCell* src = (digit & 1) ? info.m_bodyAabbCell.m_array : info.m_bodyAabbCellScrath.m_array;
	//		cuBodyAabbCell* dst = (digit & 1) ? info.m_bodyAabbCellScrath.m_array : info.m_bodyAabbCell.m_array;
	//
	//		const int srcOffset = blockId * D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
	//
	//		cacheBufferAdress[threadId] = histogram[(blocks * D_AABB_GRID_CELL_SORT_BLOCK_SIZE) + srcOffset + threadId];
	//
	//		cacheSortedKey[threadId] = ((1<< D_AABB_GRID_CELL_BITS) << 16) | threadId;
	//		if (index < cellCount)
	//		{
	//			cuBodyAabbCell cell;
	//			long long value = src[index].m_value;
	//			cell.m_value = value;
	//			cachedCells[threadId] = value;
	//			unsigned key = cuCountingSortEvaluateGridCellKey(cell, digit);
	//			cacheSortedKey[threadId] = (key << 16) | threadId;
	//		}
	//		__syncthreads();
	//
	//		cacheKeyPrefix[threadId] = 0;
	//		const int prefixBase = D_AABB_GRID_CELL_SORT_BLOCK_SIZE / 2;
	//		cacheKeyPrefix[prefixBase + 1 + threadId] = histogram[srcOffset + threadId];
	//					
	//		for (int k = 2; k <= D_AABB_GRID_CELL_SORT_BLOCK_SIZE; k *= 2)
	//		{
	//			for (int j = k / 2; j > 0; j /= 2)
	//			{
	//				const int threadId1 = threadId ^ j;
	//				if (threadId1 > threadId)
	//				{
	//					const int a = cacheSortedKey[threadId];
	//					const int b = cacheSortedKey[threadId1];
	//					const int mask0 = (-(threadId & k)) >> 31;
	//					const int mask1 = (-(a > b)) >> 31;
	//					const int mask = mask0 ^ mask1;
	//					cacheSortedKey[threadId] = b & mask | a & ~mask;
	//					cacheSortedKey[threadId1] = a & mask | b & ~mask;
	//				}
	//				__syncthreads();
	//			}
	//		}
	//		
	//		for (int i = 1; i < D_AABB_GRID_CELL_SORT_BLOCK_SIZE; i = i << 1)
	//		{
	//			const int sum = cacheKeyPrefix[prefixBase + threadId] + cacheKeyPrefix[prefixBase - i + threadId];
	//			__syncthreads();
	//			cacheKeyPrefix[prefixBase + threadId] = sum;
	//			__syncthreads();
	//		}
	//
	//		if (index < cellCount)
	//		{
	//			const unsigned keyValue = cacheSortedKey[threadId];
	//			const unsigned key = keyValue >> 16;
	//			const unsigned dstOffset = threadId + cacheBufferAdress[key] - cacheKeyPrefix[prefixBase + key];
	//			dst[dstOffset].m_value = cachedCells[keyValue & 0xffff];
	//		}
	//	}
	//}
}

__global__ void cuCountingSortAddSubPrefix(const cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned blockId = blockIdx.x;
		const unsigned threadId = threadIdx.x;
		const unsigned cellCount = info.m_bodyAabbCell.m_size - 1;

		if (blockId >= D_PREFIX_SCAN_PASSES)
		{

		}
		else
		{
			//const unsigned blocks = 1 + (cellCount + D_AABB_GRID_CELL_SORT_BLOCK_SIZE - 1) / D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
			//
			//unsigned* histogram = info.m_histogram.m_array;
			//
			//unsigned sum = 0;
			//unsigned start = blockId * D_AABB_GRID_CELL_DIGIT_BLOCK_SIZE;
			//for (int i = 1; i < blocks; i++)
			//{
			//	const unsigned value = histogram[start + threadId];
			//	histogram[start + threadId] = sum;
			//	sum += value;
			//	start += D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
			//}
			////const int dstIndex = 2 * blocks * D_AABB_GRID_CELL_SORT_BLOCK_SIZE + blockId * D_AABB_GRID_CELL_DIGIT_BLOCK_SIZE;
			////histogram[dstIndex + threadId] = sum;
			//histogram[start + threadId] = sum;
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

__global__ void cuCountingSortBodyCellsPrefixScan(const cuSceneInfo& info)
{
	//__shared__  int cacheBuffer[D_AABB_GRID_CELL_SORT_BLOCK_SIZE / 2 + D_AABB_GRID_CELL_SORT_BLOCK_SIZE + 1];
	//
	//if (info.m_frameIsValid)
	//{
	//	const unsigned threadId = threadIdx.x;
	//	unsigned* histogram = info.m_histogram.m_array;
	//	const unsigned cellCount = info.m_bodyAabbCell.m_size - 1;
	//	const unsigned blocks = (cellCount + D_AABB_GRID_CELL_SORT_BLOCK_SIZE - 1) / D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
	//	const unsigned histogramBase = 2 * blocks * D_AABB_GRID_CELL_SORT_BLOCK_SIZE;
	//
	//	const unsigned cacheStart = D_AABB_GRID_CELL_SORT_BLOCK_SIZE / 2;
	//
	//	cacheBuffer[threadId] = 0;
	//	cacheBuffer[cacheStart + threadId + 1] = histogram[histogramBase + threadId];
	//
	//	__syncthreads();
	//	for (int i = 1; i < D_AABB_GRID_CELL_SORT_BLOCK_SIZE; i = i << 1)
	//	{
	//		const unsigned sum = cacheBuffer[cacheStart + threadId] + cacheBuffer[cacheStart - i + threadId];
	//		__syncthreads();
	//		cacheBuffer[cacheStart + threadId] = sum;
	//		__syncthreads();
	//	}
	//	histogram[histogramBase + threadId] = cacheBuffer[cacheStart + threadId];
	//}
}

static void CountingSortBodyCells(ndCudaContext* context, int digit)
{
	cuSceneInfo* const sceneInfo = context->m_sceneInfoCpu;
	cudaStream_t stream = context->m_solverComputeStream;
	cuSceneInfo* const infoGpu = context->m_sceneInfoGpu;
	//ndInt32 radixBlock = D_AABB_GRID_CELL_SORT_BLOCK_SIZE / D_AABB_GRID_CELL_DIGIT_BLOCK_SIZE;

	const unsigned histogramGridBlockSize = (1 << D_AABB_GRID_CELL_BITS);
	const unsigned blocks = (sceneInfo->m_bodyAabbCell.m_capacity + histogramGridBlockSize - 1) / histogramGridBlockSize;
	//const unsigned histogramSuperGridBlockSize = histogramGridBlockSize * D_PREFIX_SCAN_PASSES;
	//const unsigned newCapacity = 2 * histogramGridBlockSize + histogramSuperGridBlockSize * ((cellCount + histogramSuperGridBlockSize - 1) / histogramSuperGridBlockSize);

	cuCountingSortCountGridCells << <blocks, histogramGridBlockSize, 0, stream >> > (*infoGpu, digit);
	cuCountingSortAddSubPrefix << <D_PREFIX_SCAN_PASSES + 1, histogramGridBlockSize, 0, stream >> > (*infoGpu);
	//cuCountingSortBodyCellsPrefixScan << <1, D_AABB_GRID_CELL_SORT_BLOCK_SIZE, 0, stream >> > (*infoGpu);
	//cuCountingSortCaculatePrefixOffset << <radixBlock, D_AABB_GRID_CELL_DIGIT_BLOCK_SIZE, 0, stream >> > (*infoGpu);
	//cuCountingSortShuffleGridCells << <blocks, D_AABB_GRID_CELL_SORT_BLOCK_SIZE, 0, stream >> > (*infoGpu, digit);
}

void CudaBodyAabbCellSortBuffer(ndCudaContext* const context)
{
	//BitonicSort();

	dAssert(context->m_bodyAabbCell.GetCount() <= context->m_histogram.GetCount());
	dAssert(context->m_bodyAabbCell.GetCount() == context->m_bodyAabbCellScrath.GetCount());
	
	CountingSortBodyCells(context, 0);
	//CountingSortBodyCells(context, 1);
	//CountingSortBodyCells(context, 2);
}
