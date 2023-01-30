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
#include "ndCudaContext.h"
#include "ndCudaPrefixScan.cuh"
#include "ndCudaCountingSort.cuh"
#include "ndCudaContextImplement.h"
#include "ndCudaContextImplementInternal.cuh"

ndCudaContextImplement::ndCudaContextImplement(ndCudaDevice* const device)
	:m_device(device)
	//,m_sceneInfoGpu(nullptr)
	//,m_sceneInfoCpu(nullptr)
	//,m_histogram()
	//,m_bodyBuffer()
	//,m_sceneGraph()
	//,m_bodyAabbCell()
	//,m_bodyAabbCellScratch()
	//,m_transformBuffer0()
	//,m_transformBuffer1()
	//,m_transformBufferCpu()
	//,m_solverMemCpuStream(0)
	//,m_solverComputeStream(0)
	//,m_timeInSeconds(0.0f)
	//,m_frameCounter(0)
{
	//cudaError_t cudaStatus;
	//cudaStatus = cudaStreamCreate(&m_solverMemCpuStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaStreamCreate(&m_solverComputeStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaMalloc((void**)&m_sceneInfoGpu, sizeof(ndCudaSceneInfo));
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaMallocHost((void**)&m_sceneInfoCpu, sizeof(ndCudaSceneInfo));
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//if (cudaStatus != cudaSuccess)
	//{
	//	ndAssert(0);
	//}
	//
	//*m_sceneInfoCpu = ndCudaSceneInfo();

	m_sortPrefixBuffer.SetCount(m_sortPrefixBuffer.GetCapacity());


	// ***********************************
	//m_src.SetCount(1000);
	m_src.SetCount(3 * 256 + 1);
	//m_src.SetCount(1000000);
	for (int i = 0; i < m_src.GetCount(); ++i)
	{
		//m_src[i] = rand() % 256;
		m_src[i] = i % 256;
	}

	m_scan0.SetCount(1024 * 256);
	m_scan1.SetCount(1024 * 256);
	m_dst0.SetCount(m_src.GetCount());
	m_dst1.SetCount(m_src.GetCount());
	m_buf0.SetCount(m_src.GetCount());
	m_buf1.SetCount(m_src.GetCount());
	m_buf0.ReadData(&m_src[0], m_src.GetCount());

	for (int i = 0; i < m_dst1.GetCount(); ++i)
	{
		m_dst1[i] = -2;
	}
	m_buf1.ReadData(&m_dst1[0], m_dst1.GetCount());

	class GetKey
	{
		public:
		int GetRadix(int item) const
		{
			return item & 0xff;
		};
	};

	ndCountingSort<int, GetKey, 8>(m_src, m_dst0, m_scan0);
	for (int i = 1; i < m_dst0.GetCount(); ++i)
	{
		int a = m_dst0[i - 1];
		int b = m_dst0[i];
		ndAssert(a <= b);
	}
}

ndCudaContextImplement::~ndCudaContextImplement()
{
	ndAssert(m_device);
	//cudaError_t cudaStatus;
	//cudaStatus = cudaFreeHost(m_sceneInfoCpu);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaFree(m_sceneInfoGpu);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaStreamDestroy(m_solverComputeStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaStreamDestroy(m_solverMemCpuStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//if (cudaStatus != cudaSuccess)
	//{
	//	ndAssert(0);
	//}
}

const char* ndCudaContextImplement::GetStringId() const
{
	return m_device->m_prop.name;
}

int ndCudaContextImplement::GetComputeUnits() const
{
	return m_device->GetComputeUnits();
}

#if 0
float ndCudaContextImplement::GetTimeInSeconds() const
{
	return float (m_timeInSeconds);
}


void ndCudaContextImplement::End()
{
	m_frameCounter = m_frameCounter + 1;
	ndCudaSceneInfo* const gpuInfo = m_sceneInfoGpu;
	ndCudaEndFrame << < 1, 1, 0, m_solverComputeStream >> > (*gpuInfo, m_frameCounter);
}

ndCudaSpatialVector* ndCudaContextImplement::GetTransformBuffer()
{
	return &m_transformBufferCpu[0];
}

void ndCudaContextImplement::ResizeBuffers(int cpuBodyCount)
{
	ndCudaDeviceBuffer<unsigned>& histogram = m_histogram;
	ndCudaDeviceBuffer<ndCudaBodyProxy>& bodyBuffer = m_bodyBuffer;
	ndCudaDeviceBuffer<ndCudaSceneNode>& sceneGraph = m_sceneGraph;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell>& bodyAabbCell0 = m_bodyAabbCell;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell>& bodyAabbCell1 = m_bodyAabbCellScratch;
	ndCudaDeviceBuffer<ndCudaSpatialVector>& transformBuffer0 = m_transformBuffer0;
	ndCudaDeviceBuffer<ndCudaSpatialVector>& transformBuffer1 = m_transformBuffer1;
	
	histogram.SetCount(cpuBodyCount);
	bodyBuffer.SetCount(cpuBodyCount);
	sceneGraph.SetCount(cpuBodyCount * 2);
	bodyAabbCell0.SetCount(cpuBodyCount);
	bodyAabbCell1.SetCount(cpuBodyCount);
	transformBuffer0.SetCount(cpuBodyCount);
	transformBuffer1.SetCount(cpuBodyCount);

	ndCudaHostBuffer<ndCudaSpatialVector>& transformBufferCpu = m_transformBufferCpu;
	transformBufferCpu.SetCount(cpuBodyCount);
}

void ndCudaContextImplement::LoadBodyData(const ndCudaBodyProxy* const src, int cpuBodyCount)
{
	cudaDeviceSynchronize();
		
	ndCudaSceneInfo info;
	info.m_histogram = ndCudaBuffer<unsigned>(m_histogram);
	info.m_bodyArray = ndCudaBuffer<ndCudaBodyProxy>(m_bodyBuffer);
	info.m_sceneGraph = ndCudaBuffer<ndCudaSceneNode>(m_sceneGraph);
	info.m_bodyAabbCell = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCell);
	info.m_bodyAabbCellScratch = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCellScratch);
	info.m_transformBuffer0 = ndCudaBuffer<ndCudaSpatialVector>(m_transformBuffer0);
	info.m_transformBuffer1 = ndCudaBuffer<ndCudaSpatialVector>(m_transformBuffer1);
	
	*m_sceneInfoCpu = info;
	cudaError_t cudaStatus = cudaMemcpy(m_sceneInfoGpu, &info, sizeof(ndCudaSceneInfo), cudaMemcpyHostToDevice);
	ndAssert(cudaStatus == cudaSuccess);

	const int blocksCount = (cpuBodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	m_bodyBuffer.ReadData(src, cpuBodyCount);
	ndCudaInitTransforms << <blocksCount, D_THREADS_PER_BLOCK, 0, m_solverComputeStream >> > (*m_sceneInfoCpu);
	ndCudaGenerateSceneGraph << <1, 1, 0, m_solverComputeStream >> > (*m_sceneInfoCpu);
	
	cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

void ndCudaContextImplement::ValidateContextBuffers()
{
	ndCudaSceneInfo* const sceneInfo = m_sceneInfoCpu;
	if (!sceneInfo->m_frameIsValid)
	{
		cudaDeviceSynchronize();

		if (sceneInfo->m_histogram.m_size > sceneInfo->m_histogram.m_capacity)
		{
			sceneInfo->m_frameIsValid = 1;
			m_histogram.SetCount(sceneInfo->m_histogram.m_size);
			sceneInfo->m_histogram = ndCudaBuffer<unsigned>(m_histogram);
		}

		if (sceneInfo->m_bodyAabbCell.m_size > sceneInfo->m_bodyAabbCell.m_capacity)
		{
			sceneInfo->m_frameIsValid = 1;
			m_bodyAabbCell.SetCount(sceneInfo->m_bodyAabbCell.m_size);
			m_bodyAabbCellScratch.SetCount(sceneInfo->m_bodyAabbCell.m_size);
			sceneInfo->m_bodyAabbCell = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCell);
			sceneInfo->m_bodyAabbCellScratch = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCellScratch);
		}

		if (!sceneInfo->m_frameCount)
		{
			sceneInfo->m_frameIsValid = 1;
		}

		ndAssert(sceneInfo->m_frameIsValid);
		cudaError_t cudaStatus = cudaMemcpy(m_sceneInfoGpu, sceneInfo, sizeof(ndCudaSceneInfo), cudaMemcpyHostToDevice);
		ndAssert(cudaStatus == cudaSuccess);
		if (cudaStatus != cudaSuccess)
		{
			ndAssert(0);
		}
		cudaDeviceSynchronize();
	}
}

void ndCudaContextImplement::UpdateTransform()
{
	ndCudaSceneInfo* const infoGpu = m_sceneInfoGpu;
	ndCudaGetBodyTransforms << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
}

void ndCudaContextImplement::InitBodyArray()
{
	//auto CompactMovingBodies = ndMakeObject::ndFunction([this, &scans](int threadIndex, int threadCount)
	//{
	//	const ndArray<ndBodyKinematic*>& activeBodyArray = GetActiveBodyArray();
	//	ndBodyKinematic** const sceneBodyArray = &m_sceneBodyArray[0];
	//
	//	const ndArray<ndBodyKinematic*>& view = m_bodyList.m_view;
	//	int* const scan = &scans[threadIndex][0];
	//
	//	const ndStartEnd startEnd(view.GetCount(), threadIndex, threadCount);
	//	for (int i = startEnd.m_start; i < startEnd.m_end; ++i)
	//	{
	//		ndBodyKinematic* const body = activeBodyArray[i];
	//		const int key = body->m_sceneEquilibrium;
	//		const int index = scan[key];
	//		sceneBodyArray[index] = body;
	//		scan[key] ++;
	//	}
	//});
	//ParallelExecute(BuildBodyArray);
	//int sum = 0;
	//int threadCount = GetThreadCount();
	//for (int j = 0; j < 2; j++)
	//{
	//	for (int i = 0; i < threadCount; ++i)
	//	{
	//		const int count = scans[i][j];
	//		scans[i][j] = sum;
	//		sum += count;
	//	}
	//}
	//
	//int movingBodyCount = scans[0][1] - scans[0][0];
	//m_sceneBodyArray.SetCount(m_bodyList.GetCount());
	//if (movingBodyCount)
	//{
	//	ParallelExecute(CompactMovingBodies);
	//}
	//
	//m_sceneBodyArray.SetCount(movingBodyCount);
	//
	//ndBodyKinematic* const sentinelBody = m_sentinelBody;
	//sentinelBody->PrepareStep(GetActiveBodyArray().GetCount() - 1);
	//
	//sentinelBody->m_isStatic = 1;
	//sentinelBody->m_autoSleep = 1;
	//sentinelBody->m_equilibrium = 1;
	//sentinelBody->m_equilibrium0 = 1;
	//sentinelBody->m_isJointFence0 = 1;
	//sentinelBody->m_isJointFence1 = 1;
	//sentinelBody->m_isConstrained = 0;
	//sentinelBody->m_sceneEquilibrium = 1;
	//sentinelBody->m_weigh = ndFloat32(0.0f);

	auto GetItemsCount = [] __device__(const ndCudaSceneInfo & info)
	{
		return info.m_bodyAabbCell.m_size - 1;
	};

	auto GetSrcBuffer = [] __device__(const ndCudaSceneInfo &info)
	{
		return &info.m_bodyAabbCell.m_array[0].m_value;
	};

	auto GetDstBuffer = [] __device__(const ndCudaSceneInfo & info)
	{
		return &info.m_bodyAabbCellScratch.m_array[0].m_value;
	};

	auto GetSortKey_x = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return key & 0xff;
	};

	auto GetSortKey_y = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return (key>>8) & 0xff;
	};
	
	auto GetSortKey_z = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return (key >> 16) & 0xff;
	};
	
	auto GetSortKey_w = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return (key >> 24) & 0xff;
	};

	long long dommyType = 0;
	ndCudaSceneInfo* const infoGpu = m_sceneInfoGpu;

	ndCudaInitBodyArray << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
	ndCudaHillisSteelePrefixScan << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
	ndCudaGenerateGrids << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetSrcBuffer, GetDstBuffer, GetItemsCount, GetSortKey_x, 256);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetDstBuffer, GetSrcBuffer, GetItemsCount, GetSortKey_y, 256);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetSrcBuffer, GetDstBuffer, GetItemsCount, GetSortKey_z, 256);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetDstBuffer, GetSrcBuffer, GetItemsCount, GetSortKey_w, 256);
	ndCudaCalculateBodyPairsCount << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
}

#endif

void ndCudaContextImplement::Begin()
{
	//ndAssert(0);
	cudaDeviceSynchronize();
	//// get the scene info from the update	
	//ndCudaSceneInfo* const gpuInfo = m_sceneInfoGpu;
	//ndCudaSceneInfo* const cpuInfo = m_sceneInfoCpu;
	//
	//cudaError_t cudaStatus = cudaMemcpyAsync(cpuInfo, gpuInfo, sizeof(ndCudaSceneInfo), cudaMemcpyDeviceToHost, m_solverMemCpuStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//if (cudaStatus != cudaSuccess)
	//{
	//	ndAssert(0);
	//}
	//
	//m_timeInSeconds = double(cpuInfo->m_frameTimeInNanosecunds) * double(1.0e-9f);
	////printf("cpu frame:%d ms:%lld\n", cpuInfo->m_frameCount, cpuInfo->m_frameTimeInNanosecunds/1000000);
	//
	//const int frameCounter = m_frameCounter;
	//if (frameCounter)
	//{
	//	ndCudaHostBuffer<ndCudaSpatialVector>& cpuBuffer = m_transformBufferCpu;
	//	ndCudaDeviceBuffer<ndCudaSpatialVector>& gpuBuffer = (frameCounter & 1) ? m_transformBuffer1 : m_transformBuffer0;
	//	gpuBuffer.WriteData(&cpuBuffer[0], cpuBuffer.GetCount() - 1, m_solverMemCpuStream);
	//}
	//
	//ndCudaBeginFrame << < 1, 1, 0, m_solverComputeStream >> > (*gpuInfo);

	auto GetRadix = []  __device__(int item)
	{
		return item & 0xff;
	};
	ndCountingSort<int, 8>(this, m_buf0, m_buf1, m_sortPrefixBuffer, GetRadix);
	cudaDeviceSynchronize();

	m_sortPrefixBuffer.WriteData(&m_scan1[0], m_scan1.GetCount());
	m_buf1.WriteData(&m_dst1[0], m_dst1.GetCount());

	for (int i = 1; i < m_dst1.GetCount(); ++i)
	{
		int a = m_dst1[i - 1];
		int b = m_dst1[i];
		ndAssert(a <= b);
	}
}
