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

#include "ndCudaStdafx.h"
#include "ndCudaUtils.h"
#include "ndCudaDevice.h"
#include "ndCudaSort.cuh"
#include "ndCudaContext.h"
#include "ndCudaPrefixScan.cuh"
#include "ndCudaContextImplement.h"
#include "ndCudaContextImplementInternal.cuh"

#if 0
#define D_CUDA_SCENE_GRID_SIZE		8.0f
#define D_CUDA_SCENE_INV_GRID_SIZE	(1.0f/D_CUDA_SCENE_GRID_SIZE) 

static ndCudaBoundingBox __device__ g_boundingBoxBuffer[1024 * 1024 / D_THREADS_PER_BLOCK];

__global__ void ndCudaInitBodyArrayInternal(ndCudaSceneInfo& info)
{
	__shared__  ndCudaBoundingBox cacheAabb[D_THREADS_PER_BLOCK];

	const unsigned threadId = threadIdx.x;
	const unsigned index = threadId + blockDim.x * blockIdx.x;
	const unsigned bodyCount = info.m_bodyArray.m_size - 1;
	if (index < bodyCount)
	{
		ndCudaBodyProxy* bodyArray = info.m_bodyArray.m_array;
		ndCudaBodyProxy& body = bodyArray[index];

		// calculate shape global Matrix
		body.m_globalSphapeRotation = body.m_localRotation * body.m_rotation;
		ndCudaMatrix3x3 matrix(body.m_globalSphapeRotation.GetMatrix3x3());
		body.m_globalSphapePosition = matrix.RotateVector(body.m_localPosition) + body.m_posit;

		matrix.m_front = matrix.m_front.Scale(body.m_scale.x);
		matrix.m_up = matrix.m_up.Scale(body.m_scale.y);
		matrix.m_right = matrix.m_right.Scale(body.m_scale.z);
		matrix = body.m_alignRotation.GetMatrix3x3() * matrix;

		const ndCudaVector origin(matrix.RotateVector(body.m_obbOrigin) + body.m_globalSphapePosition);
		const ndCudaVector size(matrix.m_front.Abs().Scale(body.m_obbSize.x) + matrix.m_up.Abs().Scale(body.m_obbSize.y) + matrix.m_right.Abs().Scale(body.m_obbSize.z));

		const ndCudaVector padding(1.0f / 16.0f);
		const ndCudaVector minBox(origin - size - padding);
		const ndCudaVector maxBox(origin + size + padding);

		// save aabb and calculate bonding box for this thread block
		body.m_minAabb = minBox;
		body.m_maxAabb = maxBox;
		cacheAabb[threadId].m_min = minBox;
		cacheAabb[threadId].m_max = maxBox;
	}

	const unsigned lastBlock = bodyCount / D_THREADS_PER_BLOCK;
	if (lastBlock == blockIdx.x)
	{
		__syncthreads();
		const unsigned lastId = bodyCount - D_THREADS_PER_BLOCK * lastBlock;
		const ndCudaBoundingBox box(cacheAabb[0]);
		if (threadId >= lastId)
		{
			cacheAabb[threadId] = box;
		}
	}
	__syncthreads();

	for (int i = D_THREADS_PER_BLOCK / 2; i; i = i >> 1)
	{
		if (threadId < i)
		{
			cacheAabb[threadId].m_min = cacheAabb[threadId].m_min.Min(cacheAabb[threadId + i].m_min);
			cacheAabb[threadId].m_max = cacheAabb[threadId].m_max.Max(cacheAabb[threadId + i].m_max);
		}
		__syncthreads();
	}

	if (threadId == 0)
	{
		g_boundingBoxBuffer[blockIdx.x].m_min = cacheAabb[0].m_min;
		g_boundingBoxBuffer[blockIdx.x].m_max = cacheAabb[0].m_max;
	}
};

__global__ void ndCudaMergeAabbInternal(ndCudaSceneInfo& info)
{
	__shared__  ndCudaBoundingBox cacheAabb[D_THREADS_PER_BLOCK];
	const unsigned threadId = threadIdx.x;
	const unsigned boxCount = ((info.m_bodyArray.m_size - 1) + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	const unsigned aabbBlocks = boxCount / D_THREADS_PER_BLOCK;
	const unsigned boxLastRow = boxCount - aabbBlocks * D_THREADS_PER_BLOCK;

	cacheAabb[threadId] = g_boundingBoxBuffer[0];
	if (threadId < boxLastRow)
	{
		cacheAabb[threadId] = g_boundingBoxBuffer[aabbBlocks * D_THREADS_PER_BLOCK + threadId];
	}
	__syncthreads();

	unsigned base = 0;
	for (int i = 0; i < aabbBlocks; i++)
	{
		cacheAabb[threadId].m_min = cacheAabb[threadId].m_min.Min(cacheAabb[base + threadId].m_min);
		cacheAabb[threadId].m_max = cacheAabb[threadId].m_max.Min(cacheAabb[base + threadId].m_max);
		base += D_THREADS_PER_BLOCK;
	}
	__syncthreads();

	for (int i = D_THREADS_PER_BLOCK / 2; i; i = i >> 1)
	{
		if (threadId < i)
		{
			cacheAabb[threadId].m_min = cacheAabb[threadId].m_min.Min(cacheAabb[threadId + i].m_min);
			cacheAabb[threadId].m_max = cacheAabb[threadId].m_max.Max(cacheAabb[threadId + i].m_max);
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		ndCudaVector minBox((cacheAabb[0].m_min.Scale(D_CUDA_SCENE_INV_GRID_SIZE).Floor()).Scale(D_CUDA_SCENE_GRID_SIZE));
		ndCudaVector maxBox((cacheAabb[0].m_max.Scale(D_CUDA_SCENE_INV_GRID_SIZE).Floor()).Scale(D_CUDA_SCENE_GRID_SIZE) + ndCudaVector(D_CUDA_SCENE_GRID_SIZE));
		minBox.w = 0.0f;
		maxBox.w = 0.0f;
		info.m_worldBox.m_min = minBox;
		info.m_worldBox.m_max = maxBox;
	}
};

__global__ void ndCudaGenerateGridsInternal(ndCudaSceneInfo& info)
{
	const unsigned threadId = threadIdx.x;
	const unsigned index = threadId + blockDim.x * blockIdx.x;
	const unsigned bodyCount = info.m_bodyArray.m_size - 1;
	if (index < bodyCount)
	{
		const unsigned* histogram = info.m_histogram.m_array;
		const ndCudaBodyProxy* bodyArray = info.m_bodyArray.m_array;
		ndCudaBodyAabbCell* hashArray = info.m_bodyAabbCell.m_array;

		const ndCudaVector minBox(info.m_worldBox.m_min);
		const ndCudaVector bodyBoxMin(bodyArray[index].m_minAabb);
		const ndCudaVector bodyBoxMax(bodyArray[index].m_maxAabb);

		const int x0 = __float2int_rd((bodyBoxMin.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE);
		const int y0 = __float2int_rd((bodyBoxMin.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE);
		const int z0 = __float2int_rd((bodyBoxMin.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE);
		const int x1 = __float2int_rd((bodyBoxMax.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		const int y1 = __float2int_rd((bodyBoxMax.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		const int z1 = __float2int_rd((bodyBoxMax.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;

		ndCudaBodyAabbCell hash;

		hash.m_key = 0;
		hash.m_id = index;
		unsigned start = index ? histogram[index - 1] : 0;
		for (int z = z0; z < z1; z++)
		{
			hash.m_z = z;
			for (int y = y0; y < y1; y++)
			{
				hash.m_y = y;
				for (int x = x0; x < x1; x++)
				{
					hash.m_x = x;
					hashArray[start].m_value = hash.m_value;
					start++;
				}
			}
		}
	}
};

__global__ void ndCudaCountAabbInternal(ndCudaSceneInfo& info)
{
	const unsigned blockId = blockIdx.x;
	const unsigned bodyCount = info.m_bodyArray.m_size - 1;
	const unsigned threadId = threadIdx.x;
	const unsigned index = threadId + blockDim.x * blockId;

	unsigned* histogram = info.m_histogram.m_array;
	const ndCudaBodyProxy* bodyArray = info.m_bodyArray.m_array;
	if (index < bodyCount)
	{
		const ndCudaVector minBox(info.m_worldBox.m_min);
		const ndCudaVector bodyBoxMin(bodyArray[index].m_minAabb);
		const ndCudaVector bodyBoxMax(bodyArray[index].m_maxAabb);

		const int x0 = __float2int_rd((bodyBoxMin.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE);
		const int y0 = __float2int_rd((bodyBoxMin.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE);
		const int z0 = __float2int_rd((bodyBoxMin.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE);
		const int x1 = __float2int_rd((bodyBoxMax.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		const int y1 = __float2int_rd((bodyBoxMax.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		const int z1 = __float2int_rd((bodyBoxMax.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		const int count = (z1 - z0) * (y1 - y0) * (x1 - x0);
		histogram[index] = count;
	}
};

//auto CalculateBodyPairsCount = [] __device__(cuSceneInfo & info)
__global__ void ndCudaCalculateBodyPairsCountInternal(ndCudaSceneInfo& info)
{
	//__shared__  unsigned cacheBuffer[D_THREADS_PER_BLOCK / 2 + D_THREADS_PER_BLOCK];
	//
	//const unsigned blockId = blockIdx.x;
	//const unsigned cellCount = info.m_bodyAabbCell.m_size - 1;
	//const unsigned blocks = (cellCount + blockDim.x - 1) / blockDim.x;
	//if (blockId < blocks)
	//{
	//	const unsigned threadId = threadIdx.x;
	//	const unsigned threadId1 = D_THREADS_PER_BLOCK / 2 + threadId;
	//	int index = threadId + blockDim.x * blockIdx.x;
	//
	//	cacheBuffer[threadId] = 0;
	//	cacheBuffer[threadId1] = 0;
	//	if (index < cellCount)
	//	{
	//		const cuBodyAabbCell* hashArray = info.m_bodyAabbCell.m_array;
	//
	//		unsigned count = 0;
	//		const cuBodyAabbCell& cell = hashArray[index];
	//
	//		for (int i = index + 1; cell.m_key == hashArray[i].m_key; i++)
	//		{
	//			count++;
	//		}
	//		cacheBuffer[threadId1] = count;
	//	}
	//
	//	__syncthreads();
	//	for (int i = 1; i < D_THREADS_PER_BLOCK; i = i << 1)
	//	{
	//		int sum = cacheBuffer[threadId1] + cacheBuffer[threadId1 - i];
	//		__syncthreads();
	//		cacheBuffer[threadId1] = sum;
	//		__syncthreads();
	//	}
	//
	//	if (index < cellCount)
	//	{
	//		unsigned* scan = info.m_histogram.m_array;
	//		const unsigned prefixScanSuperBlockAlign = D_PREFIX_SCAN_PASSES * D_THREADS_PER_BLOCK;
	//		const unsigned offset = (cellCount + prefixScanSuperBlockAlign) & (-prefixScanSuperBlockAlign);
	//		scan[offset + index] = cacheBuffer[threadId1];
	//	}
	//}
};

__global__ void ndCudaGetBodyTransformsInternal (ndCudaSceneInfo & info)
{
	int index = threadIdx.x + blockDim.x * blockIdx.x;
	if (index < (info.m_bodyArray.m_size - 1))
	{
		const ndCudaBodyProxy* src = info.m_bodyArray.m_array;
		ndCudaSpatialVector* dst = (info.m_frameCount & 1) ? info.m_transformBuffer0.m_array : info.m_transformBuffer1.m_array;

		dst[index].m_linear = src[index].m_posit;
		dst[index].m_angular = src[index].m_rotation;
	}
};

__global__ void ndCudaBeginFrame(ndCudaSceneInfo& info)
{
	long long start;
	asm volatile("mov.u64 %0, %%globaltimer;" : "=l"(start));
	info.m_startFrameTime = start;
}

__global__ void ndCudaEndFrame(ndCudaSceneInfo& info, int frameCount)
{
	long long end;
	asm volatile("mov.u64 %0, %%globaltimer;" : "=l"(end));
	info.m_frameTimeInNanosecunds = end - info.m_startFrameTime;

	//printf("gpu frame:%d ms:%lld\n", info.m_frameCount, info.m_frameTimeInNanosecunds / 1000000);
	info.m_frameCount = frameCount;
}

__global__ void ndCudaInitBodyArray(ndCudaSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned bodyCount = info.m_bodyArray.m_size - 1;
		const unsigned blocksCount = (bodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		ndCudaInitBodyArrayInternal << <blocksCount, D_THREADS_PER_BLOCK, 0 >> > (info);
		#ifdef _DEBUG
			if (blocksCount > D_THREADS_PER_BLOCK * 16)
			{
				printf("function: %sn too many block to ruun in one block\n", __FUNCTION__);
			}
		#endif
		ndCudaMergeAabbInternal << <1, D_THREADS_PER_BLOCK, 0 >> > (info);

		m_device->SyncDevice();
		info.m_histogram.m_size = bodyCount;
		if (bodyCount > info.m_histogram.m_capacity)
		{
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
			return;
		}
		ndCudaCountAabbInternal << <blocksCount, D_THREADS_PER_BLOCK, 0 >> > (info);
	}
}

__global__ void ndCudaGenerateGrids(ndCudaSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned bodyCount = info.m_bodyArray.m_size - 1;
		const unsigned cellCount = info.m_histogram.m_array[bodyCount - 1];
		if ((cellCount + D_THREADS_PER_BLOCK) > info.m_bodyAabbCell.m_capacity)
		{
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
			info.m_bodyAabbCell.m_size = cellCount + D_THREADS_PER_BLOCK;
			info.m_bodyAabbCellScratch.m_size = cellCount + D_THREADS_PER_BLOCK;
			printf("skipping frame %d  function %s  line %d\n", info.m_frameCount, __FUNCTION__, __LINE__);
			return;
		}

		ndCudaBodyAabbCell cell;
		ndCudaBodyAabbCell* cellArray = info.m_bodyAabbCell.m_array;
		ndCudaBodyAabbCell* cellArrayScratch = info.m_bodyAabbCellScratch.m_array;

		cell.m_value = 0;
		cell.m_x = unsigned(-1);
		cell.m_y = unsigned(-1);
		cell.m_z = unsigned(-1);
		cell.m_id = unsigned(-1);
		cellArray[cellCount].m_value = cell.m_value;
		cellArrayScratch[cellCount].m_value = cell.m_value;

		const unsigned blocksCount = (cellCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;

		info.m_bodyAabbCell.m_size = cellCount + 1;
		info.m_bodyAabbCellScratch.m_size = cellCount + 1;
		ndCudaGenerateGridsInternal << <blocksCount, D_THREADS_PER_BLOCK, 0 >> > (info);

		const unsigned newCapacity = ndCudaCountingSortCalculateScanPrefixSize(cellCount + 2, D_THREADS_PER_BLOCK);
		if (newCapacity > info.m_histogram.m_capacity)
		{
			cuInvalidateFrame(info, __FUNCTION__, __LINE__);
			info.m_histogram.m_size = newCapacity;
		}
	}
}

__global__ void ndCudaCalculateBodyPairsCount(ndCudaSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		//printf("unsorted: %s\n", __FUNCTION__);
		//const ndCudaBodyAabbCell* dst = info.m_bodyAabbCellScrath.m_array;
		//for (int i = 0; i < info.m_bodyAabbCellScrath.m_size; i++)
		//{
		//	ndCudaBodyAabbCell cell = dst[i];
		//	printf("x(%d) y(%d) z(%d) id(%d)  %llx\n", cell.m_x, cell.m_y, cell.m_z, cell.m_id, cell.m_value);
		//}
		//printf("\n");
		//
		//printf("sorted: %s\n", __FUNCTION__);
		//const ndCudaBodyAabbCell* src = info.m_bodyAabbCell.m_array;
		//for (int i = 0; i < info.m_bodyAabbCell.m_size; i++)
		//{
		//	const ndCudaBodyAabbCell cell = src[i];
		//	printf("x(%d) y(%d) z(%d) id(%d)  %llx\n", cell.m_x, cell.m_y, cell.m_z, cell.m_id, cell.m_value);
		//}
		//printf("\n");
		//
		//printf("\n");
		//printf("%s %d\n", __FUNCTION__, info.m_frameCount);
	}
};

__global__ void ndCudaGetBodyTransforms(ndCudaSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		const unsigned threads = info.m_bodyArray.m_size - 1;
		const unsigned blocks = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		ndCudaGetBodyTransformsInternal << <blocks, D_THREADS_PER_BLOCK, 0 >> > (info);
	}
}

__global__ void ndCudaInitTransforms(ndCudaSceneInfo& info)
{
	int index = threadIdx.x + blockDim.x * blockIdx.x;
	if (index < info.m_bodyArray.m_size)
	{
		const ndCudaBodyProxy* src = info.m_bodyArray.m_array;
		ndCudaSpatialVector* dst0 = info.m_transformBuffer0.m_array;
		ndCudaSpatialVector* dst1 = info.m_transformBuffer1.m_array;

		dst0[index].m_linear = src[index].m_posit;
		dst0[index].m_angular = src[index].m_rotation;
		dst1[index].m_linear = src[index].m_posit;
		dst1[index].m_angular = src[index].m_rotation;
	}
}

__global__ void ndCudaGenerateSceneGraph(ndCudaSceneInfo& info)
{
}
#endif