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

inline bool __device__ HasUpperDigit(const cuSceneInfo& info, int digit)
{
	bool hasUpperByteHash = (&info.m_hasUpperByteHash.x)[digit / 4] ? true : false;
	bool test = !(digit & 1) | hasUpperByteHash;
	return test;
}

template <typename Predicate>
__global__ void CudaSortHistogram(Predicate EvaluateKey, const cuSceneInfo& info, const cuAabbGridHash* src, int* histogram, int size, int digit)
{
	__shared__  int cacheBuffer[D_THREADS_PER_BLOCK];

	bool test = HasUpperDigit(info, digit);
	if (test)
	{
		int threadIndex = threadIdx.x;
		int index = threadIndex + blockDim.x * blockIdx.x;

		cacheBuffer[threadIndex] = 0;
		__syncthreads();
		if (index < size)
		{
			int key = EvaluateKey(src[index], digit);
			atomicAdd(&cacheBuffer[key], 1);
		}
		__syncthreads();
		histogram[index] = cacheBuffer[threadIndex];
	}
}

template <typename Predicate>
__global__ void CudaSortItems(Predicate EvaluateKey, const cuSceneInfo& info, const cuAabbGridHash* src, cuAabbGridHash* dst, int* histogram, int size, int digit)
{
	__shared__  int cacheKey[D_THREADS_PER_BLOCK];
	__shared__  int cacheBufferCount[D_THREADS_PER_BLOCK];
	__shared__  int cacheBufferAdress[D_THREADS_PER_BLOCK];

	int threadIndex = threadIdx.x;
	int index = threadIndex + blockDim.x * blockIdx.x;

	cacheKey[threadIndex] = 0;
	if (index < size)
	{
		bool test = HasUpperDigit(info, digit);
		if (test)
		{
			const cuAabbGridHash entry = src[index];
			cacheBufferCount[threadIndex] = histogram[index];
			cacheKey[threadIndex] = EvaluateKey(entry, digit);
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
		else
		{
			dst[index] = src[index];
		}
	}
}

template <typename Predicate>
__global__ void CudaSortPrefixScans(Predicate PrefixScan, const cuSceneInfo& info, int* histogram, int size, int digit)
{
	bool test = HasUpperDigit(info, digit);
	if (test)
	{
		PrefixScan(histogram, size);
	}
}

CudaCountingSort::CudaCountingSort(cuSceneInfo* info, cudaStream_t stream)
	:m_info(info)
	,m_stream(stream)
{
}

bool CudaCountingSort::SanityCheck(const cuAabbGridHash* const src)
{
	//cuSceneInfo info;
	//cudaError_t cudaStatus;
	//ndArray<cuAabbGridHash> data;
	//
	//cudaDeviceSynchronize();
	//cudaStatus = cudaMemcpy(&info, m_info, sizeof(cuSceneInfo), cudaMemcpyDeviceToHost);
	//dAssert(cudaStatus == cudaSuccess);
	//
	//data.SetCount(m_size);
	//cudaStatus = cudaMemcpy(&data[0], src, m_size * sizeof(cuAabbGridHash), cudaMemcpyDeviceToHost);
	//dAssert(cudaStatus == cudaSuccess);
	//
	//for (int i = 1; i < m_size; i++)
	//{
	//	cuAabbGridHash key0(data[i - 1]);
	//	cuAabbGridHash key1(data[i - 0]);
	//	bool zTest0 = key0.m_z < key1.m_z;
	//	bool zTest1 = key0.m_z == key1.m_z;
	//	bool yTest0 = key0.m_y < key1.m_y;
	//	bool yTest1 = key0.m_y == key1.m_y;
	//	bool xTest = key0.m_x <= key1.m_x;
	//	bool test = zTest0 | (zTest1 & (yTest0 | (yTest1 & xTest)));
	//	dAssert(test);
	//}
	return true;
}

void CudaCountingSort::Sort(const cuAabbGridHash* const src, cuAabbGridHash* const dst, int digit)
{
	auto EvaluateKey = [] __device__(const cuAabbGridHash & dataElement, int digit)
	{
		return dataElement.m_bytes[digit];
	};

	auto PrefixScanSum = [] __device__(int* histogram, int size)
	{
		__shared__  int cacheBuffer[2 * D_THREADS_PER_BLOCK + 1];

		int threadId = threadIdx.x;
		int threadId1 = threadId + D_THREADS_PER_BLOCK;

		int sum = 0;
		cacheBuffer[threadId] = 0;
		if (threadId == 0)
		{
			cacheBuffer[threadId1] = 0;
		}
		const int blocks = size / D_THREADS_PER_BLOCK;
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
	};

	//m_size(size)
	//m_blocks((m_size + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK)
	//ndInt32 hashBlocksCount = (infoGpu->m_scan.m_size + 8 * D_THREADS_PER_BLOCK) / D_THREADS_PER_BLOCK; 
	 
	//CudaSortHistogram << <m_blocks, D_THREADS_PER_BLOCK, 0, m_stream >> > (EvaluateKey, *m_info, src, m_histogram, m_size, digit);
	//CudaSortPrefixScans << <1, D_THREADS_PER_BLOCK, 0, m_stream >> > (PrefixScanSum, *m_info, m_histogram, m_size, digit);
	//CudaSortItems << <m_blocks, D_THREADS_PER_BLOCK, 0, m_stream >> > (EvaluateKey, *m_info, src, dst, m_histogram, m_size, digit);
}

void CudaCountingSort::Sort()
{
	//cuAabbGridHash* const src, cuAabbGridHash* const dst
	//for (int i = 0; i < 3; i++)
	//{
	//	Sort(src, dst, i * 4 + 0);
	//	Sort(dst, src, i * 4 + 1);
	//}
	//dAssert(SanityCheck(src));
}
