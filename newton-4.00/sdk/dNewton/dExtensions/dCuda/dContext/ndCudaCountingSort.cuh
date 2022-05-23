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

#define D_COUNTING_SORT_MAX_BLOCK_SIZE  256
#define D_COUNTING_SORT_MAX_KEY_SIZE	1024

template <typename Buffer, typename SortKeyPredicate>
__global__ void ndCudaCountingSortCountItems(const Buffer* src, unsigned* histogram, unsigned size, SortKeyPredicate sortKey)
{
	__shared__  unsigned cacheBuffer[D_COUNTING_SORT_MAX_KEY_SIZE];

	const unsigned blockId = blockIdx.x;
	const unsigned threadId = threadIdx.x;
	
	// this is wroung;
	cacheBuffer[threadId] = 0;
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

template <typename Buffer, typename SortKeyPredicate>
__global__ void ndCudaCountingSort(const Buffer* src, Buffer* dst, unsigned* prefixScanBuffer, unsigned size, SortKeyPredicate sortKey, const unsigned keySize)
{
	unsigned blocks = (size + D_COUNTING_SORT_MAX_BLOCK_SIZE -1 ) / D_COUNTING_SORT_MAX_BLOCK_SIZE;
	ndCudaCountingSortCountItems << <blocks, D_COUNTING_SORT_MAX_BLOCK_SIZE, 0 >> > (src, prefixScanBuffer, size, sortKey);
}


#endif