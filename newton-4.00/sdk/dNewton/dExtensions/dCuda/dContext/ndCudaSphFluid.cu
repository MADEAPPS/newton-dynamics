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
#include "ndCudaDevice.h"
#include "ndCudaContext.h"
#include "ndCudaSphFluid.h"
#include "ndCudaContextImplement.h"

#define D_MAX_LOCAL_SIZE 512

ndCudaSphFluid::Image::Image(const ndSphFluidInitInfo& info)
	:ndSphFluidInitInfo(info)
{
}

void ndCudaSphFluid::Image::Init(ndCudaSphFluid& fluid)
{
	m_errorCode = ndErrorCode(m_context->m_device);
	m_errorCode.Set(0);
	m_param = ndKernelParams (m_context->m_device, m_context->m_device->m_workGroupSize, fluid.m_points.GetCount());
	
	fluid.m_pointsAabb.SetCount(m_param.m_kernelCount + 32);
	if (m_context->m_implement->m_sortPrefixBuffer.GetCount() < m_param.m_itemCount * 2)
	{
		m_context->m_implement->m_sortPrefixBuffer.SetCount(m_param.m_itemCount * 2);
	}
	fluid.m_hashGridMap.SetCount(m_param.m_itemCount * 4 + 1024);
	m_activeHashGridMapSize = fluid.m_hashGridMap.GetCount();
	
	m_points = ndAssessor<ndCudaVector>(fluid.m_points);
	m_hashGridMap = ndAssessor<ndGridHash>(fluid.m_hashGridMap);
	m_pointsAabb = ndAssessor<ndSphFluidAabb>(fluid.m_pointsAabb);
	m_gridScans = ndAssessor<int>(m_context->m_implement->m_sortPrefixBuffer);
}

__global__ void ndCalculateAabb(ndCudaSphFluid::Image* fluid)
{
	__shared__  float box_x0[D_MAX_LOCAL_SIZE];
	__shared__  float box_y0[D_MAX_LOCAL_SIZE];
	__shared__  float box_z0[D_MAX_LOCAL_SIZE];
	__shared__  float box_x1[D_MAX_LOCAL_SIZE];
	__shared__  float box_y1[D_MAX_LOCAL_SIZE];
	__shared__  float box_z1[D_MAX_LOCAL_SIZE];

	int threadId = threadIdx.x;
	int blockSride = blockDim.x;

	if (threadId < fluid->m_param.m_kernelCount)
	{
		box_x0[threadId] = fluid->m_pointsAabb[threadId].m_min.x;
		box_y0[threadId] = fluid->m_pointsAabb[threadId].m_min.y;
		box_z0[threadId] = fluid->m_pointsAabb[threadId].m_min.z;
		box_x1[threadId] = fluid->m_pointsAabb[threadId].m_max.x;
		box_y1[threadId] = fluid->m_pointsAabb[threadId].m_max.y;
		box_z1[threadId] = fluid->m_pointsAabb[threadId].m_max.z;
	}
	else
	{
		box_x0[threadId] = fluid->m_pointsAabb[0].m_min.x;
		box_y0[threadId] = fluid->m_pointsAabb[0].m_min.y;
		box_z0[threadId] = fluid->m_pointsAabb[0].m_min.z;
		box_x1[threadId] = fluid->m_pointsAabb[0].m_max.x;
		box_y1[threadId] = fluid->m_pointsAabb[0].m_max.y;
		box_z1[threadId] = fluid->m_pointsAabb[0].m_max.z;
	}

	for (int i = blockSride / 2; i > 0; i = i >> 1)
	{
		if (threadId < i)
		{
			float x0 = box_x0[threadId];
			float y0 = box_y0[threadId];
			float z0 = box_z0[threadId];
			float x1 = box_x0[i + threadId];
			float y1 = box_y0[i + threadId];
			float z1 = box_z0[i + threadId];
			box_x0[threadId] = x0 < x1 ? x0 : x1;
			box_y0[threadId] = y0 < y1 ? y0 : y1;
			box_z0[threadId] = z0 < z1 ? z0 : z1;

			x0 = box_x1[threadId];
			y0 = box_y1[threadId];
			z0 = box_z1[threadId];
			x1 = box_x1[i + threadId];
			y1 = box_y1[i + threadId];
			z1 = box_z1[i + threadId];
			box_x1[threadId] = x0 > x1 ? x0 : x1;
			box_y1[threadId] = y0 > y1 ? y0 : y1;
			box_z1[threadId] = z0 > z1 ? z0 : z1;
		}
		__syncthreads();
	}

	if (threadId == 0)
	{
		ndSphFluidAabb box;

		box.m_min = ndCudaVector(box_x0[0], box_y0[0], box_z0[0], 0.0f);
		box.m_max = ndCudaVector(box_x1[0], box_y1[0], box_z1[0], 0.0f);
		ndCudaVector grid(fluid->m_gridSize);
		ndCudaVector invGrid(1.0f / fluid->m_gridSize);

		// add one grid padding to the aabb
		box.m_min = box.m_min - grid;
		box.m_max = box.m_max + grid + grid;

		// quantize the aabb to integers of the gird size
		box.m_min = grid * (box.m_min * invGrid).Floor();
		box.m_max = grid * (box.m_max * invGrid).Floor();
	
		// make sure the w component is zero.
		//m_box0 = box.m_min & ndVector::m_triplexMask;
		//m_box1 = box.m_max & ndVector::m_triplexMask;
		box.m_min.w = 0.0f;
		box.m_max.w = 0.0f;
		fluid->m_aabb = box;

		//ndWorkingBuffers& data = *m_workingBuffers;
		//ndInt32 numberOfGrid = ndInt32((box.m_max.m_x - box.m_min.m_x) * invGrid.m_x + ndFloat32(1.0f));
		//data.SetWorldToGridMapping(numberOfGrid, m_box1.m_x, m_box0.m_x);
	}
}

__global__ void ndCalculateBlockAabb(ndCudaSphFluid::Image* fluid)
{
	__shared__  float box_x0[D_MAX_LOCAL_SIZE];
	__shared__  float box_y0[D_MAX_LOCAL_SIZE];
	__shared__  float box_z0[D_MAX_LOCAL_SIZE];
	__shared__  float box_x1[D_MAX_LOCAL_SIZE];
	__shared__  float box_y1[D_MAX_LOCAL_SIZE];
	__shared__  float box_z1[D_MAX_LOCAL_SIZE];

	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blockSride = blockDim.x;

	float xMin = 1.0e15f;
	float yMin = 1.0e15f;
	float zMin = 1.0e15f;
	float xMax = -1.0e15f;
	float yMax = -1.0e15f;
	float zMax = -1.0e15f;

	int base = blockSride * fluid->m_param.m_blocksPerKernel * blockId;
	for (int i = 0; i < fluid->m_param.m_blocksPerKernel; ++i)
	{
		int index = base + threadId;
		ndCudaVector point(index < fluid->m_points.m_size ? fluid->m_points[index] : fluid->m_points[0]);

		xMin = point.x < xMin ? point.x : xMin;
		yMin = point.y < yMin ? point.y : yMin;
		zMin = point.z < zMin ? point.z : zMin;

		xMax = point.x > xMax ? point.x : xMax;
		yMax = point.y > yMax ? point.y : yMax;
		zMax = point.z > zMax ? point.z : zMax;

		base += blockSride;
	}

	box_x0[threadId] = xMin;
	box_y0[threadId] = yMin;
	box_z0[threadId] = zMin;

	box_x1[threadId] = xMax;
	box_y1[threadId] = yMax;
	box_z1[threadId] = zMax;

	for (int i = fluid->m_param.m_workGroupSize / 2; i > 0; i = i >> 1)
	{
		if (threadId < i)
		{
			float x0 = box_x0[threadId];
			float y0 = box_y0[threadId];
			float z0 = box_z0[threadId];
			float x1 = box_x0[i + threadId];
			float y1 = box_y0[i + threadId];
			float z1 = box_z0[i + threadId];
			box_x0[threadId] = x0 < x1 ? x0 : x1;
			box_y0[threadId] = y0 < y1 ? y0 : y1;
			box_z0[threadId] = z0 < z1 ? z0 : z1;
	
			x0 = box_x1[threadId];
			y0 = box_y1[threadId];
			z0 = box_z1[threadId];
			x1 = box_x1[i + threadId];
			y1 = box_y1[i + threadId];
			z1 = box_z1[i + threadId];
			box_x1[threadId] = x0 > x1 ? x0 : x1;
			box_y1[threadId] = y0 > y1 ? y0 : y1;
			box_z1[threadId] = z0 > z1 ? z0 : z1;
		}
		__syncthreads();
	}
	
	if (threadId == 0)
	{
		fluid->m_pointsAabb[blockId].m_min = ndCudaVector(box_x0[0], box_y0[0], box_z0[0], 0.0f);
		fluid->m_pointsAabb[blockId].m_max = ndCudaVector(box_x1[0], box_y1[0], box_z1[0], 0.0f);
	}
}

__global__ void ndCountGrids(ndCudaSphFluid::Image* fluid)
{
	__shared__  float scans[D_MAX_LOCAL_SIZE/2 + D_MAX_LOCAL_SIZE + 1];

	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int halfBlockSride = blockSride / 2;
	int base = blockSride * fluid->m_param.m_blocksPerKernel * blockId;

	const ndCudaVector origin(fluid->m_aabb.m_min);
	const ndCudaVector box(fluid->m_gridSize * 0.5f * 0.99f);
	const ndCudaVector invGridSize(1.0f / fluid->m_gridSize);

	int sumAccumulator = 0;
	for (int i = 0; i < fluid->m_param.m_blocksPerKernel; ++i)
	{
		int index = base + threadId;

		if (threadId < halfBlockSride)
		{
			scans[threadId] = 0;
		}

		if (index < fluid->m_points.m_size)
		{
			const ndCudaVector posit(fluid->m_points[index]);
			const ndCudaVector r(posit - origin);
			const ndCudaVector p(r - origin);
			const ndCudaVector p0((r - box) * invGridSize);
			const ndCudaVector p1((r + box) * invGridSize);
			
			const ndCudaSphFluid::ndGridHash box0Hash(p0, index);
			const ndCudaSphFluid::ndGridHash box1Hash(p1, index);
			const ndCudaSphFluid::ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);
			
			const unsigned code = unsigned(codeHash.m_z * 2 + codeHash.m_x);
			scans[halfBlockSride + threadId] = fluid->m_neighborgInfo.m_counter[code];
		}
		else
		{
			scans[halfBlockSride + threadId] = 0;
		}

		for (int j = 1; j < blockSride; j = j << 1)
		{
			__syncthreads();
			int sum = scans[halfBlockSride + threadId] + scans[halfBlockSride + threadId - j];
			__syncthreads();
			scans[halfBlockSride + threadId] = sum;
		}
		__syncthreads();
		fluid->m_gridScans[index] = scans[halfBlockSride + threadId] + sumAccumulator;
		sumAccumulator += scans[halfBlockSride + blockSride - 1];

		base += blockSride;
	}
	__syncthreads();

	if (threadId == 0)
	{
		int offset = blockSride * fluid->m_param.m_blocksPerKernel * fluid->m_param.m_kernelCount;
		fluid->m_gridScans[offset + blockId] = sumAccumulator;
	}
}

__global__ void ndPrefixScanSum(ndCudaSphFluid::Image* fluid, int kernelStride)
{
	__shared__  float scanSum[D_MAX_LOCAL_SIZE / 2 + D_MAX_LOCAL_SIZE + 1];

	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int halfKernelStride = kernelStride / 2;
	int scanSize = fluid->m_param.m_workGroupSize * fluid->m_param.m_blocksPerKernel * fluid->m_param.m_kernelCount;

	if (kernelStride > 1)
	{
		if (threadId < halfKernelStride)
		{
			scanSum[threadId] = 0;
		}

		scanSum[halfKernelStride + threadId] = fluid->m_gridScans[scanSize + threadId];
		for (int j = 1; j < kernelStride; j = j << 1)
		{
			int sum;
			__syncthreads();
			if (threadId < kernelStride)
			{
				sum = scanSum[halfKernelStride + threadId] + scanSum[halfKernelStride + threadId - j];
			}
			__syncthreads();
			if (threadId < kernelStride)
			{
				scanSum[halfKernelStride + threadId] = sum;
			}
		}

		int base = blockSride * blockId;
		int itemsPerBlock = fluid->m_param.m_workGroupSize * fluid->m_param.m_blocksPerKernel;
		for (int i = 0; i < fluid->m_param.m_kernelCount; ++i)
		{
			int index = base + threadId;
			float sumAcc = scanSum[halfKernelStride + i - 1];
			fluid->m_gridScans[index] = fluid->m_gridScans[index] + sumAcc;
			base += itemsPerBlock;
		}
		__syncthreads();

		if ((blockId == (gridDim.x - 1) && (threadId == 0)))
		{
			int activeHashGridMapSize = scanSum[halfKernelStride + kernelStride - 1];
			fluid->m_hashGridMap[activeHashGridMapSize].m_gridHash = uint64_t (- 1);
			fluid->m_activeHashGridMapSize = activeHashGridMapSize;

			//fluid->m_errorCode.Set(-2);
		}
	}
	else
	{
		if ((blockId == (gridDim.x - 1) && (threadId == 0)))
		{
			int activeHashGridMapSize = fluid->m_gridScans[scanSize];
			fluid->m_hashGridMap[activeHashGridMapSize].m_gridHash = uint64_t(-1);
			fluid->m_activeHashGridMapSize = activeHashGridMapSize;

			//fluid->m_errorCode.Set(-2);
		}
	}
}

__global__ void ndCreateGrids(ndCudaSphFluid::Image* fluid)
{
	int blockId = blockIdx.x;
	int threadId = threadIdx.x;
	int blockSride = blockDim.x;
	int base = blockSride * fluid->m_param.m_blocksPerKernel * blockId;

	const ndCudaVector origin(fluid->m_aabb.m_min);
	const ndCudaVector box(fluid->m_gridSize * 0.5f * 0.99f);
	const ndCudaVector invGridSize(1.0f / fluid->m_gridSize);

	__shared__  float scanStart;
	if (threadId == 0)
	{
		scanStart = fluid->m_gridScans[0];
	}
	__syncthreads();

	for (int i = 0; i < fluid->m_param.m_blocksPerKernel; ++i)
	{
		int index = base + threadId;
		if (index < fluid->m_points.m_size)
		{
			//const ndVector r(posit[i] - origin);
			//const ndVector p(r * invGridSize);
			//const ndGridHash hashKey(p, i);
			const ndCudaVector posit(fluid->m_points[index]);
			const ndCudaVector r(posit - origin);
			const ndCudaVector p(r - origin);
			//const ndCudaSphFluid::ndGridHash boxHash(p, i);

			//const ndVector p0((r - box) * invGridSize);
			//const ndVector p1((r + box) * invGridSize);
			//ndGridHash box0Hash(p0, i);
			//const ndGridHash box1Hash(p1, i);
			//const ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);
			const ndCudaVector p0((r - box) * invGridSize);
			const ndCudaVector p1((r + box) * invGridSize);
			const ndCudaSphFluid::ndGridHash box0Hash(p0, index);
			const ndCudaSphFluid::ndGridHash box1Hash(p1, index);
			const ndCudaSphFluid::ndGridHash codeHash(box1Hash.m_gridHash - box0Hash.m_gridHash);

			//const ndInt32 base = scans[i];
			//const ndInt32 count = scans[i + 1] - base;
			//const ndInt32 code = ndInt32(codeHash.m_z * 2 + codeHash.m_x);
			const int base = fluid->m_gridScans[index] - scanStart;
			const unsigned code = unsigned(codeHash.m_z * 2 + codeHash.m_x);
			const ndCudaSphFluid::ndGridHash* const neigborgh = &fluid->m_neighborgInfo.m_neighborDirs[code][0];
			//ndAssert(count == neiborghood.m_counter[code]);
			const int count = fluid->m_neighborgInfo.m_counter[code];
			
			const ndCudaSphFluid::ndGridHash hashKey(p, index);
			for (int j = 0; j < count; ++j)
			{
				//ndGridHash quadrand(box0Hash);
				ndCudaSphFluid::ndGridHash quadrand(box0Hash);
				//quadrand.m_gridHash += neigborgh[j].m_gridHash;
				 quadrand.m_gridHash += neigborgh[j].m_gridHash;
				//quadrand.m_cellType = ndGridType(quadrand.m_gridHash == hashKey.m_gridHash);
				quadrand.m_cellType = ndCudaSphFluid::ndGridType(quadrand.m_gridHash == hashKey.m_gridHash);
				//ndAssert(quadrand.m_cellType == ((quadrand.m_gridHash == hashKey.m_gridHash) ? ndHomeGrid : ndAdjacentGrid));
				//dst[base + j] = quadrand;
				fluid->m_hashGridMap[base + j] = quadrand;
			}
		}
		base += blockSride;
	}
}


ndCudaSphFluid::ndCudaSphFluid(const ndSphFluidInitInfo& info)
	:m_imageCpu(info)
	,m_imageGpu(nullptr)
	,m_points()
	,m_pointsAabb()
{
	m_imageCpu.m_cudaStatus = cudaMalloc((void**)&m_imageGpu, sizeof (Image));
	ndAssert(m_imageCpu.m_cudaStatus == cudaSuccess);
}

ndCudaSphFluid::~ndCudaSphFluid()
{
	m_imageCpu.m_cudaStatus = cudaFree(m_imageGpu);
	ndAssert(m_imageCpu.m_cudaStatus == cudaSuccess);
}

void ndCudaSphFluid::MemCpy(const double* const src, int strideInItems, int items)
{
	ndAssert(0);
}

void GetPositions(double* const dst, int strideInItems, int items)
{
	ndAssert(0);
}

void ndCudaSphFluid::MemCpy(const float* const src, int strideInItems, int items)
{
	m_points.SetCount(items);

	if (strideInItems == sizeof(ndCudaVector) / sizeof(float))
	{
		const ndCudaVector* const srcPtr = (ndCudaVector*)src;
		m_points.ReadData(srcPtr, items);
	}
	else
	{
		ndAssert(0);
	}

	InitBuffers();
}

void ndCudaSphFluid::GetPositions(float* const dst, int strideInItems, int items)
{
	if (strideInItems == sizeof(ndCudaVector) / sizeof(float))
	{
		ndCudaVector* const dstPtr = (ndCudaVector*)dst;
		m_points.WriteData(dstPtr, items);
	}
	else
	{
		ndAssert(0);
	}
}

void ndCudaSphFluid::InitBuffers()
{
	m_imageCpu.Init(*this);
	m_imageCpu.m_cudaStatus = cudaMemcpy(m_imageGpu, &m_imageCpu, sizeof (Image), cudaMemcpyHostToDevice);
	ndAssert(m_imageCpu.m_cudaStatus == cudaSuccess);
}

void ndCudaSphFluid::CaculateAabb()
{
	int power = 1;
	while (power < m_imageCpu.m_param.m_kernelCount)
	{
		power *= 2;
	}
	ndCalculateBlockAabb << <m_imageCpu.m_param.m_kernelCount, m_imageCpu.m_param.m_workGroupSize, 0 >> > (m_imageGpu);
	ndCalculateAabb << <1, power, 0 >> > (m_imageGpu);
}

bool ndCudaSphFluid::TraceHashes()
{
#if 0
	char imageBuff[sizeof(Image) + 256];
	Image* image = (Image*)&imageBuff;
	m_imageCpu.m_cudaStatus = cudaMemcpy(image, m_imageGpu, sizeof(Image), cudaMemcpyDeviceToHost);
	cudaDeviceSynchronize();

	ndCudaHostBuffer<ndGridHash> buffer;
	buffer.SetCount(image->m_activeHashGridMapSize + 4000);
	buffer.ReadData(&m_hashGridMap[0], image->m_activeHashGridMapSize);
	for (int i = 0; i < image->m_activeHashGridMapSize; i++)
	{
		cuTrace(("id(%d)\tx(%d)\tz(%d)\n", buffer[i].m_particleIndex, buffer[i].m_x, buffer[i].m_z));
	}
#endif

	return true;
}


void ndCudaSphFluid::Update(float timestep)
{
	ndAssert(m_imageCpu.m_errorCode.Get() == 0);
	CaculateAabb();
	CreateGrids();

	ndAssert (TraceHashes());

#if 0
	char xxxxx[sizeof(Image) + 256];
	Image* xxxxxxx = (Image*)&xxxxx;
	m_imageCpu.m_cudaStatus = cudaMemcpy(xxxxxxx, m_imageGpu, sizeof(Image), cudaMemcpyDeviceToHost);
	cudaDeviceSynchronize();
	ndAssert(m_imageCpu.m_cudaStatus == cudaSuccess);
	ndCudaHostBuffer<int> scans;
	scans.SetCount(xxxxxxx->m_param.m_itemCount + 4000);
	scans.ReadData(&xxxxxxx->m_gridScans[0], scans.GetCount());
	scans.SetCount(xxxxxxx->m_param.m_itemCount);
#endif
}

void ndCudaSphFluid::CreateGrids()
{
	//data.m_gridScans.SetCount(m_posit.GetCount() + 1);
	//data.m_gridScans[m_posit.GetCount()] = 0;
	//threadPool->ParallelExecute(CountGrids);
	//data.m_hashGridMap.SetCount(gridCount);
	//threadPool->ParallelExecute(CreateGrids);

	int power = 1;
	while (power < m_imageCpu.m_param.m_kernelCount)
	{
		power *= 2;
	}
	ndCountGrids << <m_imageCpu.m_param.m_kernelCount, m_imageCpu.m_param.m_workGroupSize, 0 >> > (m_imageGpu);
	ndPrefixScanSum << <m_imageCpu.m_param.m_blocksPerKernel * 2, m_imageCpu.m_param.m_workGroupSize / 2, 0 >> > (m_imageGpu, power);
	ndCreateGrids << <m_imageCpu.m_param.m_kernelCount, m_imageCpu.m_param.m_workGroupSize, 0 >> > (m_imageGpu);
	
	//data.m_hashGridMapScratchBuffer.SetCount(gridCount);
}