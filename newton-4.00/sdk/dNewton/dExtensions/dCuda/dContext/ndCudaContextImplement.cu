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
#include "ndCudaContext.h"
#include "ndCudaContextImplement.h"


__global__ void CudaEndFrame(ndCudaSceneInfo& info, int frameCount)
{
	info.m_frameCount = frameCount;
}

ndCudaContextImplement::ndCudaContextImplement()
	:m_sceneInfoGpu(nullptr)
	,m_sceneInfoCpu(nullptr)
	,m_histogram()
	,m_bodyBufferGpu()
	,m_bodyAabbCell()
	,m_bodyAabbCellScrath()
	,m_boundingBoxGpu()
	,m_transformBufferCpu0()
	,m_transformBufferCpu1()
	,m_transformBufferGpu0()
	,m_transformBufferGpu1()
	,m_solverMemCpyStream(0)
	,m_solverComputeStream(0)
	,m_frameCounter(0)
{
	cudaError_t cudaStatus;
	cudaStatus = cudaStreamCreate(&m_solverMemCpyStream);
	dAssert(cudaStatus == cudaSuccess);
	
	cudaStatus = cudaStreamCreate(&m_solverComputeStream);
	dAssert(cudaStatus == cudaSuccess);
	
	cudaStatus = cudaMalloc((void**)&m_sceneInfoGpu, sizeof(ndCudaSceneInfo));
	dAssert(cudaStatus == cudaSuccess);
	
	cudaStatus = cudaMallocHost((void**)&m_sceneInfoCpu, sizeof(ndCudaSceneInfo));
	dAssert(cudaStatus == cudaSuccess);
	
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
	
	*m_sceneInfoCpu = ndCudaSceneInfo();
}

ndCudaContextImplement::~ndCudaContextImplement()
{
	cudaError_t cudaStatus;
	
	cudaStatus = cudaFreeHost(m_sceneInfoCpu);
	dAssert(cudaStatus == cudaSuccess);
	
	cudaStatus = cudaFree(m_sceneInfoGpu);
	dAssert(cudaStatus == cudaSuccess);
	
	cudaStatus = cudaStreamDestroy(m_solverComputeStream);
	dAssert(cudaStatus == cudaSuccess);
	
	cudaStatus = cudaStreamDestroy(m_solverMemCpyStream);
	dAssert(cudaStatus == cudaSuccess);
	
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
}

/*
void ndCudaContextImplement::SwapBuffers()
{
	//m_transformBufferCpu0.Swap(m_transformBufferCpu1);
}
*/

void ndCudaContextImplement::Begin()
{
	cudaDeviceSynchronize();

	const ndInt32 frameCounter = m_frameCounter;

	// get the scene info from the update	
	ndCudaSceneInfo* const gpuInfo = m_sceneInfoGpu;
	ndCudaSceneInfo* const cpuInfo = m_sceneInfoCpu;

	cudaError_t cudaStatus = cudaMemcpyAsync(cpuInfo, gpuInfo, sizeof(ndCudaSceneInfo), cudaMemcpyDeviceToHost, m_solverMemCpyStream);
	dAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}

	CudaEndFrame << < 1, 1, 0, m_solverComputeStream >> > (*gpuInfo, frameCounter);
	if (frameCounter)
	{
		ndCudaHostBuffer<ndCudaSpatialVector>& cpuBuffer = m_transformBufferCpu0;
		ndCudaDeviceBuffer<ndCudaSpatialVector>& gpuBuffer = (frameCounter & 1) ? m_transformBufferGpu1 : m_transformBufferGpu0;
		gpuBuffer.WriteData(&cpuBuffer[0], cpuBuffer.GetCount() - 1, m_solverMemCpyStream);
	}
}

ndCudaSpatialVector* ndCudaContextImplement::GetTransformBuffer0()
{
	return &m_transformBufferCpu0[0];
}

ndCudaSpatialVector* ndCudaContextImplement::GetTransformBuffer1()
{
	return &m_transformBufferCpu1[0];
}

void ndCudaContextImplement::ResizeBuffers(int cpuBodyCount)
{
	const ndInt32 gpuBodyCount = D_THREADS_PER_BLOCK * ((cpuBodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK);
	
	ndCudaDeviceBuffer<unsigned>& histogramGpu = m_histogram;
	ndCudaDeviceBuffer<ndCudaBodyProxy>& bodyBufferGpu = m_bodyBufferGpu;
	ndCudaDeviceBuffer<ndCudaBoundingBox>& boundingBoxGpu = m_boundingBoxGpu;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell>& bodyAabbCellGpu0 = m_bodyAabbCell;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell>& bodyAabbCellGpu1 = m_bodyAabbCellScrath;
	ndCudaHostBuffer<ndCudaSpatialVector>& transformBufferCpu0 = m_transformBufferCpu0;
	ndCudaHostBuffer<ndCudaSpatialVector>& transformBufferCpu1 = m_transformBufferCpu1;
	ndCudaDeviceBuffer<ndCudaSpatialVector>& transformBufferGpu0 = m_transformBufferGpu0;
	ndCudaDeviceBuffer<ndCudaSpatialVector>& transformBufferGpu1 = m_transformBufferGpu1;
	
	histogramGpu.SetCount(cpuBodyCount);
	bodyBufferGpu.SetCount(cpuBodyCount);
	bodyAabbCellGpu0.SetCount(cpuBodyCount);
	bodyAabbCellGpu1.SetCount(cpuBodyCount);
	transformBufferGpu0.SetCount(cpuBodyCount);
	transformBufferGpu1.SetCount(cpuBodyCount);
	transformBufferCpu0.SetCount(cpuBodyCount);
	transformBufferCpu1.SetCount(cpuBodyCount);
	boundingBoxGpu.SetCount(gpuBodyCount / D_THREADS_PER_BLOCK);
}

void ndCudaContextImplement::LoadBodyData(int cpuBodyCount)
{
	//auto InitTransforms = [] __device__(const cuSceneInfo & info)
	//{
	//	int index = threadIdx.x + blockDim.x * blockIdx.x;
	//	if (index < info.m_bodyArray.m_size)
	//	{
	//		cuBodyProxy* src = info.m_bodyArray.m_array;
	//		cuSpatialVector* dst0 = info.m_transformBuffer0.m_array;
	//		cuSpatialVector* dst1 = info.m_transformBuffer1.m_array;
	//
	//		dst0[index].m_linear = src[index].m_posit;
	//		dst0[index].m_angular = src[index].m_rotation;
	//		dst1[index].m_linear = src[index].m_posit;
	//		dst1[index].m_angular = src[index].m_rotation;
	//	}
	//};
	
	cudaDeviceSynchronize();
		
	ndCudaSceneInfo info;
	info.m_histogram = ndCudaBuffer<unsigned>(m_histogram);
	info.m_bodyArray = ndCudaBuffer<ndCudaBodyProxy>(m_bodyBufferGpu);
	info.m_bodyAabbArray = ndCudaBuffer<ndCudaBoundingBox>(m_boundingBoxGpu);
	info.m_bodyAabbCell = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCell);
	info.m_bodyAabbCellScrath = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCellScrath);
	info.m_transformBuffer0 = ndCudaBuffer<ndCudaSpatialVector>(m_transformBufferGpu0);
	info.m_transformBuffer1 = ndCudaBuffer<ndCudaSpatialVector>(m_transformBufferGpu0);
	
	*m_sceneInfoCpu = info;
	cudaError_t cudaStatus = cudaMemcpy(m_sceneInfoGpu, &info, sizeof(ndCudaSceneInfo), cudaMemcpyHostToDevice);
	dAssert(cudaStatus == cudaSuccess);

	const ndInt32 blocksCount = (cpuBodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	const ndInt32 gpuBodyCount = D_THREADS_PER_BLOCK * ((cpuBodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK);
	
	//bodyBufferGpu.ReadData(&bodyBufferCpu[0], cpuBodyCount);
	//CudaInitTransforms << <blocksCount, D_THREADS_PER_BLOCK, 0, 0 >> > (InitTransforms, *m_context->m_sceneInfoCpu);
	
	cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
}