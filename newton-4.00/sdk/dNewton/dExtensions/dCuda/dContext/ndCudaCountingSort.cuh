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

#ifndef __CU_COUNTIN_SORT_H__
#define __CU_COUNTIN_SORT_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include "ndCudaContext.h"
#include "ndCudaSceneInfo.h"
#include "ndCudaIntrinsics.h"

#define D_COUNTING_SORT_MAX_BLOCK_SIZE  1024

template <typename Buffer, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountItemsInternal(const Buffer* src, unsigned* histogram, unsigned size, SortKeyPredicate sortKey)
{
	__shared__  unsigned cacheBuffer[D_COUNTING_SORT_MAX_BLOCK_SIZE];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	
	cacheBuffer[threadId] = 0;
	__syncthreads();
	const unsigned index = threadId + blockDim.x * blockId;
	if (index < size)
	{
		const unsigned key = sortKey(src[index].m_key);
		atomicAdd(&cacheBuffer[key], 1);
	}
	__syncthreads();
	
	const unsigned dstBase = blockDim.x * blockId;
	histogram[dstBase + threadId] = cacheBuffer[threadId];
}

__global__ void ndCudaCountingCellsPrefixScanInternal(unsigned* histogram, unsigned blockCount)
{
	unsigned sum = 0;
	unsigned offset = 0;

	//printf("%d %d %d %d\n", histogram[0], histogram[1], histogram[2], histogram[3]);

	const unsigned threadId = threadIdx.x;
	for (int i = 0; i < blockCount; i++)
	{
		const unsigned count = histogram[offset + threadId];
		histogram[offset + threadId] = sum;
		sum += count;
		offset += blockDim.x;
	}
	histogram[offset + threadId] = sum;
}


inline unsigned __device__ ndCudaCountingSortCalculateScanPrefixSize(unsigned items, unsigned keySize)
{
	unsigned blocks = (items + D_COUNTING_SORT_MAX_BLOCK_SIZE - 1) / D_COUNTING_SORT_MAX_BLOCK_SIZE;
	return keySize * (blocks + 2);
}

template <typename Buffer, typename SortKeyPredicate>
__global__ void ndCudaCountingSort(const Buffer* src, Buffer* dst, unsigned* histogram, unsigned size, SortKeyPredicate sortKey, const unsigned keySize)
{
	const unsigned blocks = (size + D_COUNTING_SORT_MAX_BLOCK_SIZE -1 ) / D_COUNTING_SORT_MAX_BLOCK_SIZE;

	printf("%d %d %d %d\n", histogram[0], histogram[1], histogram[2], histogram[3]);
	ndCudaCountingSortCountItemsInternal << <blocks, D_COUNTING_SORT_MAX_BLOCK_SIZE, 0 >> > (src, histogram, size, sortKey);
	ndCudaCountingCellsPrefixScanInternal << <1, keySize, 0 >> > (histogram, blocks);
}


#endif