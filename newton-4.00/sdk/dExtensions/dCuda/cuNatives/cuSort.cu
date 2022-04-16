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
#include "cuSort.h"
#include "ndCudaContext.h"


//__global__ void cuTest0(const cuSceneInfo& info, int digit)
//{
//
//}
//
//__global__ void cuTest1(const cuSceneInfo& info, int digit, cudaStream_t stream)
//{
//	cuTest0 << <1, 1, 0, stream >> > (info, digit);
//}



inline bool __device__ cuIsThisGridHashDigitValid(const cuSceneInfo& info, int digit)
{
	bool isEven = (digit & 1) ? false : true;
	bool hasUpperByteHash = (&info.m_hasUpperByteHash.x)[digit / 4] ? true : false;
	bool test = isEven | hasUpperByteHash;
	return test;
}

inline unsigned __device__ cuEvaluateGridHashKey(const cuAabbGridHash& dataElement, int digit)
{
	return dataElement.m_bytes[digit];
};

__global__ void cuGridHashPrefixScan(const cuSceneInfo& info, int digit)
{
	__shared__  int cacheBuffer[2 * D_THREADS_PER_BLOCK + 1];

	if (info.m_frameIsValid)
	{
		bool test = cuIsThisGridHashDigitValid(info, digit);
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
			const int blocks = (info.m_histogram.m_size + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
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

__global__ void cuCountGridHashKeys(const cuSceneInfo& info, int digit)
{
	__shared__  int cacheBuffer[D_THREADS_PER_BLOCK];
	if (info.m_frameIsValid)
	{
		bool test = cuIsThisGridHashDigitValid(info, digit);
		if (test)
		{
			int threadIndex = threadIdx.x;
			cacheBuffer[threadIndex] = 0;

			int index = threadIndex + blockDim.x * blockIdx.x;

			int* histogram = info.m_histogram.m_array;
			const cuAabbGridHash* src = (digit & 1) ? info.m_hashArrayScrath.m_array : info.m_hashArray.m_array;
		
			__syncthreads();
			int gridCount = info.m_hashArray.m_size - 1;
			if (index < gridCount)
			{
				unsigned key = cuEvaluateGridHashKey(src[index], digit);
				atomicAdd(&cacheBuffer[key], 1);
			}
			__syncthreads();
			histogram[index] = cacheBuffer[threadIndex];
		}
	}
}

__global__ void cuSortGridHashItems(const cuSceneInfo& info, int digit)
{
	__shared__  int cacheKey[D_THREADS_PER_BLOCK];
	__shared__  int cacheBufferCount[D_THREADS_PER_BLOCK];
	__shared__  int cacheBufferAdress[D_THREADS_PER_BLOCK];

	if (info.m_frameIsValid)
	{
		int threadIndex = threadIdx.x;
		int index = threadIndex + blockDim.x * blockIdx.x;
		const int gridCount = info.m_hashArray.m_size - 1;
		bool test = cuIsThisGridHashDigitValid(info, digit);
		if (test)
		{
			int* histogram = info.m_histogram.m_array;
			cuAabbGridHash* dst = (digit & 1) ? info.m_hashArray.m_array: info.m_hashArrayScrath.m_array;
			const cuAabbGridHash* src = (digit & 1) ? info.m_hashArrayScrath.m_array : info.m_hashArray.m_array;

			cacheKey[threadIndex] = 0;
			if (index < gridCount)
			{
				const cuAabbGridHash entry = src[index];
				cacheBufferCount[threadIndex] = histogram[index];
				cacheKey[threadIndex] = cuEvaluateGridHashKey(entry, digit);
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
		else
		{
			if (index < gridCount)
			{
				cuAabbGridHash* dst = info.m_hashArray.m_array;
				const cuAabbGridHash* src = info.m_hashArrayScrath.m_array;
				dst[index] = src[index];
			}
		}
	}
}

static bool GridHashSanityCheck(ndCudaContext* const context)
{
	cuSceneInfo info;
	cudaError_t cudaStatus;
	cudaDeviceSynchronize();
	cudaStatus = cudaMemcpy(&info, context->m_sceneInfoGpu, sizeof(cuSceneInfo), cudaMemcpyDeviceToHost);
	dAssert(cudaStatus == cudaSuccess);

	if (info.m_frameIsValid)
	{
		ndArray<cuAabbGridHash> data;
		int size = info.m_hashArray.m_size;
		data.SetCount(size);
		cudaStatus = cudaMemcpy(&data[0], info.m_hashArray.m_array, size * sizeof(cuAabbGridHash), cudaMemcpyDeviceToHost);
		dAssert(cudaStatus == cudaSuccess);

		for (int i = 1; i < size; i++)
		{
			cuAabbGridHash key0(data[i - 1]);
			cuAabbGridHash key1(data[i - 0]);
			bool zTest0 = key0.m_z < key1.m_z;
			bool zTest1 = key0.m_z == key1.m_z;
			bool yTest0 = key0.m_y < key1.m_y;
			bool yTest1 = key0.m_y == key1.m_y;
			bool xTest = key0.m_x <= key1.m_x;
			//bool test = zTest0 | (zTest1 & (yTest0 | (yTest1 & xTest)));
			bool test = xTest;
			dAssert(test);
		}
	}
	return true;
}

static void SortGridHash(ndCudaContext* context, int digit)
{
	cudaStream_t stream = context->m_stream0;
	cuSceneInfo* const infoGpu = context->m_sceneInfoGpu;
	cuSceneInfo* const sceneInfo = context->m_sceneInfoCpu;
	ndInt32 blocks = (sceneInfo->m_histogram.m_size + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	if (blocks)
	{
		cuCountGridHashKeys << <blocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuGridHashPrefixScan << <1, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
		cuSortGridHashItems << <blocks, D_THREADS_PER_BLOCK, 0, stream >> > (*infoGpu, digit);
	}
}

void CudaSortGridHash(ndCudaContext* const context)
{
	dAssert(context->m_gridHash.GetCount() == context->m_gridHashTmp.GetCount());
	dAssert(context->m_histogram.GetCount() == context->m_gridHashTmp.GetCount());

	for (int i = 0; i < 1; i++)
	{
		SortGridHash(context, i * 4 + 0);
		SortGridHash(context, i * 4 + 1);
	}
	dAssert(GridHashSanityCheck(context));
}
