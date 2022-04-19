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

#include "ndCudaContext.h"
#include "cuSortBodyAabbCells.h"

//#define D_USE_PARALLEL_PREFIX_SCAN

//__global__ void cuTest0(const cuSceneInfo& info, int digit)
//{
//
//}
//
//__global__ void cuTest1(const cuSceneInfo& info, int digit, cudaStream_t stream)
//{
//	cuTest0 << <1, 1, 0, stream >> > (info, digit);
//}


inline bool __device__ cuCountingSortIsThisGridCellDigitValid(const cuSceneInfo& info, int digit)
{
	bool isEven = (digit & 1) ? false : true;
	bool hasUpperByteHash = (&info.m_hasUpperByteHash.x)[digit / 4] ? true : false;
	bool test = isEven | hasUpperByteHash;
	return test;
}

inline unsigned __device__ cuCountingSortEvaluateGridCellKey(const cuBodyAabbCell& dataElement, int digit)
{
	return dataElement.m_bytes[digit];
};

__global__ void cuCountSortCountGridCells(const cuSceneInfo& info, int digit)
{
	__shared__  int cacheBuffer[D_THREADS_PER_BLOCK];
	if (info.m_frameIsValid)
	{
		bool test = cuCountingSortIsThisGridCellDigitValid(info, digit);
		if (test)
		{
			const int cellCount = info.m_bodyAabbCell.m_size - 1;
			const int blocks = (cellCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
			if (blockIdx.x < blocks)
			{
				int threadIndex = threadIdx.x;
				cacheBuffer[threadIndex] = 0;

				int index = threadIndex + blockDim.x * blockIdx.x;

				int* histogram = info.m_histogram.m_array;
				const cuBodyAabbCell* src = (digit & 1) ? info.m_bodyAabbCellScrath.m_array : info.m_bodyAabbCell.m_array;

				__syncthreads();
				if (index < cellCount)
				{
					unsigned key = cuCountingSortEvaluateGridCellKey(src[index], digit);
					atomicAdd(&cacheBuffer[key], 1);
				}
				__syncthreads();
				histogram[index] = cacheBuffer[threadIndex];
			}
		}
	}
}

__global__ void cuCountingSortBodyCellsItems(const cuSceneInfo& info, int digit)
{
	__shared__  int cacheKey[D_THREADS_PER_BLOCK];
	__shared__  int cacheBufferCount[D_THREADS_PER_BLOCK];
	__shared__  int cacheBufferAdress[D_THREADS_PER_BLOCK];

	if (info.m_frameIsValid)
	{
		int threadIndex = threadIdx.x;
		int index = threadIndex + blockDim.x * blockIdx.x;
		const int cellCount = info.m_bodyAabbCell.m_size - 1;
		bool test = cuCountingSortIsThisGridCellDigitValid(info, digit);
		if (test)
		{
			const int blocks = (cellCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
			if (blockIdx.x < blocks)
			{
				int* histogram = info.m_histogram.m_array;
				cuBodyAabbCell* dst = (digit & 1) ? info.m_bodyAabbCell.m_array : info.m_bodyAabbCellScrath.m_array;
				const cuBodyAabbCell* src = (digit & 1) ? info.m_bodyAabbCellScrath.m_array : info.m_bodyAabbCell.m_array;

				cacheKey[threadIndex] = 0;
				cacheBufferCount[threadIndex] = histogram[index];
				if (index < cellCount)
				{
					const cuBodyAabbCell entry = src[index];
					cacheKey[threadIndex] = cuCountingSortEvaluateGridCellKey(entry, digit);
					__syncthreads();

					if (threadIndex == 0)
					{
						for (int i = 0; i < D_THREADS_PER_BLOCK; i++)
						{
							const int key = cacheKey[i];
							const int dstIndex = cacheBufferCount[key];
							cacheBufferAdress[i] = dstIndex;
							cacheBufferCount[key] = dstIndex + 1;
						}
					}
					__syncthreads();
					int dstIndex = cacheBufferAdress[threadIndex];
					dst[dstIndex] = entry;
				}
			}
		}
		else
		{
			if (index < cellCount)
			{
				cuBodyAabbCell* dst = info.m_bodyAabbCell.m_array;
				const cuBodyAabbCell* src = info.m_bodyAabbCellScrath.m_array;
				dst[index] = src[index];
			}
		}
	}
}

#ifdef D_USE_PARALLEL_PREFIX_SCAN
__global__ void cuCountingSortClearLastSuperBlock(const cuSceneInfo& info, int digit)
{
	if (info.m_frameIsValid)
	{
		bool test = cuCountingSortIsThisGridCellDigitValid(info, digit);
		if (test)
		{
			const int cellCount = info.m_bodyAabbCell.m_size - 1;
			const int blocks = (cellCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
			const int superBlocks = (blocks + D_COUNT_SORT_SUPER_BLOCK - 1) / D_COUNT_SORT_SUPER_BLOCK;

			const int offset = threadIdx.x + (superBlocks - 1) * D_COUNT_SORT_SUPER_BLOCK * D_THREADS_PER_BLOCK;
			int* histogram = info.m_histogram.m_array;
			for (int i = 0; i < D_COUNT_SORT_SUPER_BLOCK; i++)
			{
				histogram[offset + i * D_THREADS_PER_BLOCK] = 0;
			}
		}
	}
}

__global__ void cuCountingSortAddSuperBlock(const cuSceneInfo& info, int digit)
{
	if (info.m_frameIsValid)
	{
		bool test = cuCountingSortIsThisGridCellDigitValid(info, digit);
		if (test)
		{
			const int cellCount = info.m_bodyAabbCell.m_size - 1;
			const int blocks = (cellCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
			const int superBlocks = (blocks + D_COUNT_SORT_SUPER_BLOCK - 1) / D_COUNT_SORT_SUPER_BLOCK;
			if (blockIdx.x < superBlocks)
			{
				const int threadId = threadIdx.x;
				const int superBlockSize = D_COUNT_SORT_SUPER_BLOCK * D_THREADS_PER_BLOCK;
				const int offsetIn = threadId + blockIdx.x * superBlockSize;
				int* histogram = info.m_histogram.m_array;

				int sum = 0;
				for (int i = 0; i < D_COUNT_SORT_SUPER_BLOCK; i++)
				{
					sum += histogram[offsetIn + i * D_THREADS_PER_BLOCK];
				}
				const int offset = threadId + superBlocks * superBlockSize;
				histogram[offset] = sum;
			}
		}
	}
}

__global__ void cuCountingSortBodyCellsPrefixScan(const cuSceneInfo& info, int digit)
{
	__shared__  int cacheBuffer[2 * D_THREADS_PER_BLOCK + 1];

	if (info.m_frameIsValid)
	{
		bool test = cuCountingSortIsThisGridCellDigitValid(info, digit);
		if (test)
		{
			int threadId = threadIdx.x;
			int threadId1 = threadId + D_THREADS_PER_BLOCK;

			int sum = 0;
			cacheBuffer[threadId] = 0;
			if (threadId == 0)
			{
				cacheBuffer[threadId1] = 0;
			}

			int* histogram = info.m_histogram.m_array;
			const int cellCount = info.m_bodyAabbCell.m_size - 1;
			const int blocks = (cellCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
			const int superBlocks = (blocks + D_COUNT_SORT_SUPER_BLOCK - 1) / D_COUNT_SORT_SUPER_BLOCK;
			const int superBlockOffset = threadId + superBlocks * D_COUNT_SORT_SUPER_BLOCK * D_THREADS_PER_BLOCK;
			for (int i = 0; i < superBlocks; i++)
			{
				sum += histogram[superBlockOffset + i * D_THREADS_PER_BLOCK];
			}
			cacheBuffer[threadId1 + 1] = sum;
			__syncthreads();

			for (int i = 1; i < D_THREADS_PER_BLOCK; i = i << 1)
			{
				int sum = cacheBuffer[threadId1] + cacheBuffer[threadId1 - i];
				__syncthreads();
				cacheBuffer[threadId1] = sum;
				__syncthreads();
			}

			sum = cacheBuffer[threadId1];
			for (int i = 0; i < blocks; i++)
			{
				int j = i * D_THREADS_PER_BLOCK + threadId;
				int partialSum = histogram[j];
				histogram[j] = sum;
				sum += partialSum;
			}
		}
	}
}

#else

__global__ void cuCountingSortBodyCellsPrefixScan(const cuSceneInfo& info, int digit)
{
	__shared__  int cacheBuffer[2 * D_THREADS_PER_BLOCK + 1];

	if (info.m_frameIsValid)
	{
		bool test = cuCountingSortIsThisGridCellDigitValid(info, digit);
		if (test)
		{
			int threadId = threadIdx.x;
			int threadId1 = threadId + D_THREADS_PER_BLOCK;

			int sum = 0;
			cacheBuffer[threadId] = 0;
			if (threadId == 0)
			{
				cacheBuffer[threadId1] = 0;
			}

			int* histogram = info.m_histogram.m_array;
			const int cellCount = info.m_bodyAabbCell.m_size - 1;
			const int blocks = (cellCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
			for (int i = 0; i < blocks; i++)
			{
				sum += histogram[i * D_THREADS_PER_BLOCK + threadId];
			}
			cacheBuffer[threadId1 + 1] = sum;
			__syncthreads();

			for (int i = 1; i < D_THREADS_PER_BLOCK; i = i << 1)
			{
				int sum = cacheBuffer[threadId1] + cacheBuffer[threadId1 - i];
				__syncthreads();
				cacheBuffer[threadId1] = sum;
				__syncthreads();
			}

			sum = cacheBuffer[threadId1];
			for (int i = 0; i < blocks; i++)
			{
				int j = i * D_THREADS_PER_BLOCK + threadId;
				int partialSum = histogram[j];
				histogram[j] = sum;
				sum += partialSum;
			}
		}
	}
}
#endif

static bool CountingSortBodyCellsSanityCheck(ndCudaContext* const context)
{
	cuSceneInfo info;
	cudaError_t cudaStatus;
	cudaDeviceSynchronize();
	cudaStatus = cudaMemcpy(&info, context->m_sceneInfoGpu, sizeof(cuSceneInfo), cudaMemcpyDeviceToHost);
	dAssert(cudaStatus == cudaSuccess);

	if (info.m_frameIsValid)
	{
		static ndArray<cuBodyAabbCell> data;
		int size = info.m_bodyAabbCell.m_size - 1;
		data.SetCount(size);
		cudaStatus = cudaMemcpy(&data[0], info.m_bodyAabbCell.m_array, size * sizeof(cuBodyAabbCell), cudaMemcpyDeviceToHost);
		dAssert(cudaStatus == cudaSuccess);

		for (int i = 1; i < size; i++)
		{
			cuBodyAabbCell key0(data[i - 1]);
			cuBodyAabbCell key1(data[i - 0]);
			bool zTest0 = key0.m_z < key1.m_z;
			bool zTest1 = key0.m_z == key1.m_z;
			bool yTest0 = key0.m_y < key1.m_y;
			bool yTest1 = key0.m_y == key1.m_y;
			bool xTest = key0.m_x <= key1.m_x;
			bool test = zTest0 | (zTest1 & (yTest0 | (yTest1 & xTest)));
			//test = xTest;
			//test = key0.m_y <= key1.m_y;
			//test = yTest0 | (yTest1 & xTest);
			dAssert(test);
		}
	}
	return true;
}

static void CountingSortBodyCells(ndCudaContext* context, int digit)
{
	cuSceneInfo* const sceneInfo = context->m_sceneInfoCpu;
	ndInt32 blocks = (sceneInfo->m_bodyAabbCell.m_capacity + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	if (blocks)
	{
		cudaStream_t stream = context->m_solverComputeStream;
		cuSceneInfo* const infoGpu = context->m_sceneInfoGpu;

		#ifdef D_USE_PARALLEL_PREFIX_SCAN
		ndInt32 superBlocks = (blocks + D_COUNT_SORT_SUPER_BLOCK - 1) / D_COUNT_SORT_SUPER_BLOCK;

		cuCountingSortClearLastSuperBlock << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuCountSortCountGridCells << <blocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuCountingSortAddSuperBlock << <superBlocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuCountingSortBodyCellsPrefixScan << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuCountingSortBodyCellsItems << <blocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);

		#else
		cuCountSortCountGridCells << <blocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuCountingSortBodyCellsPrefixScan << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuCountingSortBodyCellsItems << <blocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		#endif
	}
}

void CudaSortBodyAabbCells(ndCudaContext* const context)
{
	dAssert(context->m_bodyAabbCell.GetCount() <= context->m_histogram.GetCount());
	dAssert(context->m_bodyAabbCell.GetCount() == context->m_bodyAabbCellTmp.GetCount());

	for (int i = 0; i < 3; i++)
	{
		CountingSortBodyCells(context, i * 4 + 0);
		CountingSortBodyCells(context, i * 4 + 1);
	}

	dAssert(CountingSortBodyCellsSanityCheck(context));
}
