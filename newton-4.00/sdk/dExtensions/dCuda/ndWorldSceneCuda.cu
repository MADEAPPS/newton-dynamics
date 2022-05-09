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

#include <ndWorld.h>
#include <ndModel.h>
#include <ndWorldScene.h>
#include <ndBodyDynamic.h>
#include <ndSkeletonList.h>
#include <ndDynamicsUpdate.h>
#include <ndBodyParticleSet.h>
#include <ndDynamicsUpdateSoa.h>
#include <ndJointBilateralConstraint.h>

#include "cuQuat.h"
#include "cuVector.h"
#include "cuMatrix3x3.h"
#include "cuPrefixScan.h"
#include "ndCudaContext.h"
#include "ndWorldSceneCuda.h"
#include "cuSortBodyAabbCells.h"

#define D_CUDA_SCENE_GRID_SIZE		8.0f
#define D_CUDA_SCENE_INV_GRID_SIZE	(1.0f/D_CUDA_SCENE_GRID_SIZE) 

__global__ void CudaEndFrame(cuSceneInfo& info, int frameCount)
{
	info.m_frameCount = frameCount;
}

template <typename Predicate>
__global__ void CudaInitBodyArray(Predicate UpdateBodyScene, cuSceneInfo& info)
{
	UpdateBodyScene(info);
}

template <typename Predicate>
__global__ void CudaMergeAabb(Predicate ReducedAabb, cuSceneInfo& info)
{
	ReducedAabb(info);
}

template <typename Predicate>
__global__ void CudaCountAabb(Predicate CountAabb, cuSceneInfo& info)
{
	CountAabb(info);
}

template <typename Predicate>
__global__ void CudaGenerateGridHash(Predicate GenerateHash, cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		GenerateHash(info);
	}
}

template <typename Predicate>
__global__ void CudaEndGridHash(Predicate EndGridHash, cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		EndGridHash(info);
	}
}

template <typename Predicate>
__global__ void CudaGetBodyTransforms(Predicate GetTransform, cuSceneInfo& info, int frameCount)
{
	GetTransform(info, frameCount);
}

template <typename Predicate>
__global__ void CudaInitTransforms(Predicate InitTransforms, cuSceneInfo& info)
{
	InitTransforms(info);
}

template <typename Predicate>
__global__ void CudaBodyCalculatePairsCount(Predicate CalculatePairsCount, cuSceneInfo& info)
{
	if (info.m_frameIsValid)
	{
		CalculatePairsCount(info);
	}
}

ndWorldSceneCuda::ndWorldSceneCuda(const ndWorldScene& src)
	:ndWorldScene(src)
	,m_context(ndCudaContext::CreateContext())
{
	m_bodyListChanged = 1;
}

ndWorldSceneCuda::~ndWorldSceneCuda()
{
	if (m_context)
	{
		delete m_context;
	}
}

bool ndWorldSceneCuda::IsValid() const
{
	return m_context ? true : false;
}

void ndWorldSceneCuda::Begin()
{
	ndWorldScene::Begin();
	cudaDeviceSynchronize();

	cudaStream_t stream = m_context->m_solverMemCpyStream;
	const ndInt32 frameCounter = m_context->m_frameCounter;

	// get the scene info from the update	
	cuSceneInfo* const gpuInfo = m_context->m_sceneInfoGpu;
	cuSceneInfo* const cpuInfo = m_context->m_sceneInfoCpu;
	
	cudaError_t cudaStatus = cudaMemcpyAsync(cpuInfo, gpuInfo, sizeof(cuSceneInfo), cudaMemcpyDeviceToHost, stream);
	dAssert(cudaStatus == cudaSuccess);
	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}

	CudaEndFrame << < 1, 1, 0, m_context->m_solverComputeStream >> > (*gpuInfo, frameCounter);
	if (frameCounter)
	{
		cuHostBuffer<cuSpatialVector>& cpuBuffer = m_context->m_transformBufferCpu0;
		cuDeviceBuffer<cuSpatialVector>& gpuBuffer = (frameCounter & 1) ? m_context->m_transformBufferGpu1 : m_context->m_transformBufferGpu0;
		gpuBuffer.WriteData(&cpuBuffer[0], cpuBuffer.GetCount() - 1, stream);
	}
}

void ndWorldSceneCuda::End()
{
	m_context->m_frameCounter = m_context->m_frameCounter + 1;
	m_context->SwapBuffers();
	ndWorldScene::End();
}

//void ndWorldSceneCuda::FindCollidingPairs(ndBodyKinematic* const body)
void ndWorldSceneCuda::FindCollidingPairs(ndBodyKinematic* const)
{
	dAssert(0);
}

void ndWorldSceneCuda::FindCollidingPairs()
{
	//ndWorldScene::FindCollidingPairs();
}

//void ndWorldSceneCuda::CalculateContacts(ndInt32 threadIndex, ndContact* const contact)
void ndWorldSceneCuda::CalculateContacts(ndInt32, ndContact* const)
{
	dAssert(0);
}

void ndWorldSceneCuda::CalculateContacts()
{
	//ndWorldScene::CalculateContacts();
}

void ndWorldSceneCuda::LoadBodyData()
{
	auto CopyBodies = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndVector minBox(ndFloat32(1.0e15f));
		const ndVector maxBox(ndFloat32(-1.0e15f));

		ndArray<cuBodyProxy>& data = m_context->m_bodyBufferCpu;
		cuHostBuffer<cuSpatialVector>& transformBufferCpu0 = m_context->m_transformBufferCpu0;
		cuHostBuffer<cuSpatialVector>& transformBufferCpu1 = m_context->m_transformBufferCpu1;

		ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
		const ndStartEnd startEnd(bodyArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			cuSpatialVector transform;
			ndBodyKinematic* const body = bodyArray[i];
			cuBodyProxy& proxi = data[i];

			// Get thansform and velocity
			proxi.m_mass = body->GetMassMatrix();
			proxi.m_rotation = cuQuat(body->GetRotation());
			proxi.m_posit = body->GetGlobalGetCentreOfMass();
			proxi.m_invIntertia = body->GetInvInertia();
			proxi.m_dampCoef = body->GetCachedDamping();
			proxi.m_veloc = body->GetVelocity();
			proxi.m_omega = body->GetOmega();

			// Get scene manager data
			const ndShapeInstance& collision = body->GetCollisionShape();
			const ndShape* const shape = collision.GetShape();

			proxi.m_minAabb = minBox;
			proxi.m_maxAabb = maxBox;
			proxi.m_obbSize = shape->GetObbSize();
			proxi.m_obbOrigin = shape->GetObbOrigin();
			proxi.m_scale = collision.GetScale();
			proxi.m_localPosition = collision.GetLocalMatrix().m_posit;
			proxi.m_localRotation = cuQuat(ndQuaternion(collision.GetLocalMatrix()));
			proxi.m_alignRotation = cuQuat(ndQuaternion(collision.GetAlignmentMatrix()));

			transform.m_angular = cuQuat(body->GetRotation());
			transform.m_linear = body->GetGlobalGetCentreOfMass();
			transformBufferCpu0[i] = transform;
			transformBufferCpu1[i] = transform;
		}
	});

	auto InitTransforms = [] __device__(const cuSceneInfo & info)
	{
		int index = threadIdx.x + blockDim.x * blockIdx.x;
		if (index < info.m_bodyArray.m_size)
		{
			cuBodyProxy* src = info.m_bodyArray.m_array;
			cuSpatialVector* dst0 = info.m_transformBuffer0.m_array;
			cuSpatialVector* dst1 = info.m_transformBuffer1.m_array;

			dst0[index].m_linear = src[index].m_posit;
			dst0[index].m_angular = src[index].m_rotation;
			dst1[index].m_linear = src[index].m_posit;
			dst1[index].m_angular = src[index].m_rotation;
		}
	};

	cudaDeviceSynchronize();

	const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();

	const ndInt32 cpuBodyCount = bodyArray.GetCount();
	const ndInt32 blocksCount = (cpuBodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	const ndInt32 gpuBodyCount = (D_THREADS_PER_BLOCK * ((cpuBodyCount + D_THREADS_PER_BLOCK - 1)) / D_THREADS_PER_BLOCK);

	ndArray<cuBodyProxy>& bodyBufferCpu = m_context->m_bodyBufferCpu;
	bodyBufferCpu.SetCount(cpuBodyCount);

	cuDeviceBuffer<unsigned>& histogramGpu = m_context->m_histogram;
	cuDeviceBuffer<cuBodyProxy>& bodyBufferGpu = m_context->m_bodyBufferGpu;
	cuDeviceBuffer<cuBoundingBox>& boundingBoxGpu = m_context->m_boundingBoxGpu;
	cuDeviceBuffer<cuBodyAabbCell>& bodyAabbCellGpu0 = m_context->m_bodyAabbCell;
	cuDeviceBuffer<cuBodyAabbCell>& bodyAabbCellGpu1 = m_context->m_bodyAabbCellTmp;
	cuHostBuffer<cuSpatialVector>& transformBufferCpu0 = m_context->m_transformBufferCpu0;
	cuHostBuffer<cuSpatialVector>& transformBufferCpu1 = m_context->m_transformBufferCpu1;
	cuDeviceBuffer<cuSpatialVector>& transformBufferGpu0 = m_context->m_transformBufferGpu0;
	cuDeviceBuffer<cuSpatialVector>& transformBufferGpu1 = m_context->m_transformBufferGpu1;

	histogramGpu.SetCount(cpuBodyCount);
	bodyBufferGpu.SetCount(cpuBodyCount);
	bodyAabbCellGpu0.SetCount(cpuBodyCount);
	bodyAabbCellGpu1.SetCount(cpuBodyCount);
	transformBufferGpu0.SetCount(cpuBodyCount);
	transformBufferGpu1.SetCount(cpuBodyCount);
	transformBufferCpu0.SetCount(cpuBodyCount);
	transformBufferCpu1.SetCount(cpuBodyCount);
	boundingBoxGpu.SetCount(gpuBodyCount / D_THREADS_PER_BLOCK);

	cuSceneInfo info;
	info.m_histogram = cuBuffer<unsigned>(histogramGpu);
	info.m_bodyArray = cuBuffer<cuBodyProxy>(bodyBufferGpu);
	info.m_bodyAabbArray = cuBuffer<cuBoundingBox>(boundingBoxGpu);
	info.m_bodyAabbCell = cuBuffer<cuBodyAabbCell>(bodyAabbCellGpu0);
	info.m_bodyAabbCellScrath = cuBuffer<cuBodyAabbCell>(bodyAabbCellGpu1);
	info.m_transformBuffer0 = cuBuffer<cuSpatialVector>(transformBufferGpu0);
	info.m_transformBuffer1 = cuBuffer<cuSpatialVector>(transformBufferGpu1);

	cudaError_t cudaStatus;
	ParallelExecute(CopyBodies);

	*m_context->m_sceneInfoCpu = info;
	cudaStatus = cudaMemcpy(m_context->m_sceneInfoGpu, &info, sizeof(cuSceneInfo), cudaMemcpyHostToDevice);
	dAssert(cudaStatus == cudaSuccess);

	bodyBufferGpu.ReadData(&bodyBufferCpu[0], cpuBodyCount);
	CudaInitTransforms << <blocksCount, D_THREADS_PER_BLOCK, 0, 0 >> > (InitTransforms, *m_context->m_sceneInfoCpu);

	cudaDeviceSynchronize();

	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
}

void ndWorldSceneCuda::GetBodyTransforms()
{
	D_TRACKTIME();

	auto GetTransform = [] __device__(const cuSceneInfo& info, int frameCount)
	{
		int index = threadIdx.x + blockDim.x * blockIdx.x;
		if (index < (info.m_bodyArray.m_size - 1))
		{
			cuBodyProxy* src = info.m_bodyArray.m_array;
			cuSpatialVector* dst = (frameCount & 1) ? info.m_transformBuffer0.m_array : info.m_transformBuffer1.m_array;

			dst[index].m_linear = src[index].m_posit;
			dst[index].m_angular = src[index].m_rotation;
		}
	};

	cudaStream_t stream = m_context->m_solverComputeStream;
	cuSceneInfo* const infoGpu = m_context->m_sceneInfoGpu;
	
	ndInt32 threads = m_context->m_bodyBufferGpu.GetCount() - 1;
	ndInt32 blocks = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	CudaGetBodyTransforms << <blocks, D_THREADS_PER_BLOCK, 0, stream >> > (GetTransform, *infoGpu, m_context->m_frameCounter);

	//cuHostBuffer<cuSpatialVector>& cpuBuffer = m_context->m_transformBufferCpu0;
	//cuDeviceBuffer<cuSpatialVector>& gpuBuffer = m_context->m_transformBufferGpu0;
	//gpuBuffer.WriteData(&cpuBuffer[0], cpuBuffer.GetCount() - 1, stream);
}

void ndWorldSceneCuda::UpdateTransform()
{
	D_TRACKTIME();

	GetBodyTransforms();
	auto SetTransform = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
		const cuSpatialVector* const data = &m_context->m_transformBufferCpu1[0];
		const ndStartEnd startEnd(bodyArray.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			const cuSpatialVector& transform = data[i];
			const ndVector position(transform.m_linear.x, transform.m_linear.y, transform.m_linear.z, ndFloat32(1.0f));
			const ndQuaternion rotation(ndVector(transform.m_angular.x, transform.m_angular.y, transform.m_angular.z, transform.m_angular.w));
			body->SetMatrixAndCentreOfMass(rotation, position);

			body->m_transformIsDirty = true;
			UpdateTransformNotify(threadIndex, body);
		}
	});
	ParallelExecute(SetTransform);
}

void ndWorldSceneCuda::UpdateBodyList()
{
	D_TRACKTIME();
	bool bodyListChanged = m_bodyListChanged;
	ndWorldScene::UpdateBodyList();
	if (bodyListChanged)
	{
		LoadBodyData();
	}
	
	cuSceneInfo* const sceneInfo = m_context->m_sceneInfoCpu;
	if (!sceneInfo->m_frameIsValid)
	{
		cudaDeviceSynchronize();
		sceneInfo->m_frameIsValid = 1;

		CudaBodyAabbCellResizeBuffers(m_context);

		cudaError_t cudaStatus = cudaMemcpy(m_context->m_sceneInfoGpu, sceneInfo, sizeof(cuSceneInfo), cudaMemcpyHostToDevice);
		dAssert(cudaStatus == cudaSuccess);
		if (cudaStatus != cudaSuccess)
		{
			dAssert(0);
		}
		cudaDeviceSynchronize();
	}
}

bool ndWorldSceneCuda::SanityCheckPrefix() const
{
	cuSceneInfo info;
	cudaError_t cudaStatus;

	cudaDeviceSynchronize();
	cudaStatus = cudaMemcpy(&info, m_context->m_sceneInfoGpu, sizeof(cuSceneInfo), cudaMemcpyDeviceToHost);
	dAssert(cudaStatus == cudaSuccess);

	if (info.m_frameIsValid)
	{
		static ndArray<unsigned> histogram;
		histogram.SetCount(info.m_histogram.m_size);

		cudaStatus = cudaMemcpy(&histogram[0], info.m_histogram.m_array, histogram.GetCount() * sizeof(unsigned), cudaMemcpyDeviceToHost);
		dAssert(cudaStatus == cudaSuccess);
		for (int i = 1; i < histogram.GetCount(); i++)
		{
			dAssert(histogram[i - 1] <= histogram[i]);
		}
	}

	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
	return true;
}

bool ndWorldSceneCuda::SanityCheckSortCells() const
{
	cuSceneInfo info;
	cudaError_t cudaStatus;

	cudaDeviceSynchronize();
	cudaStatus = cudaMemcpy(&info, m_context->m_sceneInfoGpu, sizeof(cuSceneInfo), cudaMemcpyDeviceToHost);
	dAssert(cudaStatus == cudaSuccess);

	if (info.m_frameIsValid)
	{
		static ndArray<cuBodyAabbCell> data;
		data.SetCount(info.m_bodyAabbCell.m_size - 1);
		cudaStatus = cudaMemcpy(&data[0], info.m_bodyAabbCell.m_array, data.GetCount() * sizeof(cuBodyAabbCell), cudaMemcpyDeviceToHost);
		dAssert(cudaStatus == cudaSuccess);

		for (int i = 1; i < data.GetCount(); i++)
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

	if (cudaStatus != cudaSuccess)
	{
		dAssert(0);
	}
	return true;
}


void ndWorldSceneCuda::InitBodyArray()
{
//	ndWorldScene::InitBodyArray();

	D_TRACKTIME();
	// this has to be recreated in gpu
	//ndInt32 scans[D_MAX_THREADS_COUNT][2];
	//auto BuildBodyArray = ndMakeObject::ndFunction([this, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	//{
	//	D_TRACKTIME();
	//	const ndArray<ndBodyKinematic*>& view = GetActiveBodyArray();
	//
	//	ndInt32* const scan = &scans[threadIndex][0];
	//	scan[0] = 0;
	//	scan[1] = 0;
	//
	//	const ndFloat32 timestep = m_timestep;
	//	const ndStartEnd startEnd(view.GetCount() - 1, threadIndex, threadCount);
	//	for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
	//	{
	//		ndBodyKinematic* const body = view[i];
	//		body->ApplyExternalForces(threadIndex, timestep);
	//
	//		body->PrepareStep(i);
	//		UpdateAabb(threadIndex, body);
	//
	//		const ndInt32 key = body->m_sceneEquilibrium;
	//		scan[key] ++;
	//	}
	//});
	
	//auto CompactMovingBodies = ndMakeObject::ndFunction([this, &scans](ndInt32 threadIndex, ndInt32 threadCount)
	//{
	//	D_TRACKTIME();
	//	const ndArray<ndBodyKinematic*>& activeBodyArray = GetActiveBodyArray();
	//	ndBodyKinematic** const sceneBodyArray = &m_sceneBodyArray[0];
	//
	//	const ndArray<ndBodyKinematic*>& view = m_bodyList.m_view;
	//	ndInt32* const scan = &scans[threadIndex][0];
	//
	//	const ndStartEnd startEnd(view.GetCount(), threadIndex, threadCount);
	//	for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
	//	{
	//		ndBodyKinematic* const body = activeBodyArray[i];
	//		const ndInt32 key = body->m_sceneEquilibrium;
	//		const ndInt32 index = scan[key];
	//		sceneBodyArray[index] = body;
	//		scan[key] ++;
	//	}
	//});
	
	//ParallelExecute(BuildBodyArray);
	//ndInt32 sum = 0;
	//ndInt32 threadCount = GetThreadCount();
	//for (ndInt32 j = 0; j < 2; j++)
	//{
	//	for (ndInt32 i = 0; i < threadCount; ++i)
	//	{
	//		const ndInt32 count = scans[i][j];
	//		scans[i][j] = sum;
	//		sum += count;
	//	}
	//}
	//
	//ndInt32 movingBodyCount = scans[0][1] - scans[0][0];
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

	auto CalcuateBodyAabb = [] __device__(cuSceneInfo& info)
	{
		__shared__  cuBoundingBox cacheAabb[D_THREADS_PER_BLOCK];

		int threadId = threadIdx.x;
		int index = threadId + blockDim.x * blockIdx.x;
		const int bodyCount = info.m_bodyArray.m_size - 1;
		if (index < bodyCount)
		{
			cuBodyProxy* bodyArray = info.m_bodyArray.m_array;
			cuBodyProxy& body = bodyArray[index];

			// calculate shape global Matrix
			body.m_globalSphapeRotation = body.m_localRotation * body.m_rotation;
			cuMatrix3x3 matrix(body.m_globalSphapeRotation.GetMatrix3x3());
			body.m_globalSphapePosition = matrix.RotateVector(body.m_localPosition) + body.m_posit;

			matrix.m_front = matrix.m_front.Scale(body.m_scale.x);
			matrix.m_up = matrix.m_up.Scale(body.m_scale.y);
			matrix.m_right = matrix.m_right.Scale(body.m_scale.z);
			matrix = body.m_alignRotation.GetMatrix3x3() * matrix;

			const cuVector origin(matrix.RotateVector(body.m_obbOrigin) + body.m_globalSphapePosition);
			const cuVector size(matrix.m_front.Abs().Scale(body.m_obbSize.x) + matrix.m_up.Abs().Scale(body.m_obbSize.y) + matrix.m_right.Abs().Scale(body.m_obbSize.z));

			const cuVector padding(1.0f / 16.0f);
			const cuVector minBox(origin - size - padding);
			const cuVector maxBox(origin + size + padding);

			// save aabb and calculate bonding box for this thread block
			body.m_minAabb = minBox;
			body.m_maxAabb = maxBox;
			cacheAabb[threadId].m_min = minBox;
			cacheAabb[threadId].m_max = maxBox;
		}

		const int lastBlock = bodyCount / D_THREADS_PER_BLOCK;
		if (lastBlock == blockIdx.x)
		{
			__syncthreads();
			const int lastId = bodyCount - D_THREADS_PER_BLOCK * lastBlock;
			const cuBoundingBox box(cacheAabb[0]);
			if (threadId >= lastId)
			{
				cacheAabb[threadId] = box;
			}
		}
		__syncthreads();

		cuBoundingBox* bBox = info.m_bodyAabbArray.m_array;
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
			bBox[blockIdx.x].m_min = cacheAabb[0].m_min;
			bBox[blockIdx.x].m_max = cacheAabb[0].m_max;
		}
	};

	auto ReducedAabb = [] __device__(cuSceneInfo& info)
	{
		__shared__  cuBoundingBox cacheAabb[D_THREADS_PER_BLOCK];

		const cuBoundingBox* bBoxOut = info.m_bodyAabbArray.m_array;

		const int threadId = threadIdx.x;
		const int boxCount = info.m_bodyAabbArray.m_size;
		const int aabbBlocks = boxCount / D_THREADS_PER_BLOCK;
		const int boxLastRow = boxCount - aabbBlocks * D_THREADS_PER_BLOCK;

		cacheAabb[threadId] = bBoxOut[0];
		if (threadId < boxLastRow)
		{
			cacheAabb[threadId] = bBoxOut[aabbBlocks * D_THREADS_PER_BLOCK + threadId];
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
			cuVector minBox((cacheAabb[0].m_min.Scale(D_CUDA_SCENE_INV_GRID_SIZE).Floor()).Scale(D_CUDA_SCENE_GRID_SIZE));
			cuVector maxBox((cacheAabb[0].m_max.Scale(D_CUDA_SCENE_INV_GRID_SIZE).Floor()).Scale(D_CUDA_SCENE_GRID_SIZE) + cuVector(D_CUDA_SCENE_GRID_SIZE));
			minBox.w = 0.0f;
			maxBox.w = 0.0f;
			info.m_worldBox.m_min = minBox;
			info.m_worldBox.m_max = maxBox;
		}
	};

	auto CountAabb = [] __device__(cuSceneInfo& info)
	{
		__shared__  unsigned cacheBuffer[D_THREADS_PER_BLOCK / 2 + D_THREADS_PER_BLOCK];

		const int blockId = blockIdx.x;
		const int bodyCount = info.m_bodyArray.m_size - 1;
		const int blocks = (bodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		if (blockId < blocks)
		{
			const int threadId = threadIdx.x;
			const int threadId1 = D_THREADS_PER_BLOCK / 2 + threadId;
			const int index = threadId + blockDim.x * blockId;

			cacheBuffer[threadId] = 0;
			cacheBuffer[threadId1] = 0;
			if (index < bodyCount)
			{
				cuBodyProxy* bodyArray = info.m_bodyArray.m_array;

				const cuVector minBox(info.m_worldBox.m_min);
				const cuVector bodyBoxMin(bodyArray[index].m_minAabb);
				const cuVector bodyBoxMax(bodyArray[index].m_maxAabb);

				const int x0 = __float2int_rd((bodyBoxMin.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE);
				const int y0 = __float2int_rd((bodyBoxMin.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE);
				const int z0 = __float2int_rd((bodyBoxMin.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE);
				const int x1 = __float2int_rd((bodyBoxMax.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
				const int y1 = __float2int_rd((bodyBoxMax.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
				const int z1 = __float2int_rd((bodyBoxMax.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
				const int count = (z1 - z0) * (y1 - y0) * (x1 - x0);
				cacheBuffer[threadId1] = count;
			}
			__syncthreads();

			for (int i = 1; i < D_THREADS_PER_BLOCK; i = i << 1)
			{
				int sum = cacheBuffer[threadId1] + cacheBuffer[threadId1 - i];
				__syncthreads();
				cacheBuffer[threadId1] = sum;
				__syncthreads();
			}

			unsigned* histogram = info.m_histogram.m_array;
			histogram[index] = cacheBuffer[threadId1];
			if (index == 0)
			{
				info.m_histogram.m_size = blocks * D_THREADS_PER_BLOCK;
			}
		}
	};

	auto GenerateHashGrids = [] __device__(const cuSceneInfo & info)
	{
		const int threadId = threadIdx.x;
		const int index = threadId + blockDim.x * blockIdx.x;
		const int bodyCount = info.m_bodyArray.m_size - 1;
		if (index < bodyCount)
		{
			const unsigned* histogram = info.m_histogram.m_array;
			const cuBodyProxy* bodyArray = info.m_bodyArray.m_array;
			cuBodyAabbCell* hashArray = info.m_bodyAabbCellScrath.m_array;

			const cuVector minBox(info.m_worldBox.m_min);
			const cuVector bodyBoxMin(bodyArray[index].m_minAabb);
			const cuVector bodyBoxMax(bodyArray[index].m_maxAabb);
			
			const int x0 = __float2int_rd((bodyBoxMin.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE);
			const int y0 = __float2int_rd((bodyBoxMin.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE);
			const int z0 = __float2int_rd((bodyBoxMin.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE);
			const int x1 = __float2int_rd((bodyBoxMax.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
			const int y1 = __float2int_rd((bodyBoxMax.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
			const int z1 = __float2int_rd((bodyBoxMax.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;

			cuBodyAabbCell hash;
			hash.m_id = index;
			hash.m_key = 0;
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
						hashArray[start] = hash;
						start++;
					}
				}
			}
		}
	};

	auto EndGridHash = [] __device__(cuSceneInfo& info)
	{
		cuBodyAabbCell* hashArray = info.m_bodyAabbCellScrath.m_array;
		const int index = info.m_bodyAabbCellScrath.m_size;

		cuBodyAabbCell hash;
		hash = hashArray[index - 1];
		hash.m_id = unsigned (-1);
		hash.m_z = hash.m_z + 1;
		hashArray[index] = hash;

		//info.m_histogram.m_size = index + 1;
		info.m_bodyAabbCell.m_size = index + 1;
		info.m_bodyAabbCellScrath.m_size = index + 1;
		if ((index + 1) >= info.m_bodyAabbCell.m_capacity)
		{
			info.m_frameIsValid = 0;
		}

		const int histogrameSize = D_PREFIX_SCAN_ALIGN____ * ((index + D_PREFIX_SCAN_ALIGN____) / D_PREFIX_SCAN_ALIGN____) + D_PREFIX_SCAN_ALIGN____;
		if (histogrameSize >= info.m_histogram.m_capacity)
		{
			info.m_frameIsValid = 0;
		}
	};

	auto CalculatePairsCount = [] __device__(cuSceneInfo & info)
	{
		__shared__  unsigned cacheBuffer[D_THREADS_PER_BLOCK / 2 + D_THREADS_PER_BLOCK];

		const int blockId = blockIdx.x;
		const int cellCount = info.m_bodyAabbCell.m_size - 1;
		const int blocks = (cellCount + blockDim.x - 1) / blockDim.x;
		if (blockId < blocks)
		{
			const int threadId = threadIdx.x;
			const int threadId1 = D_THREADS_PER_BLOCK / 2 + threadId;
			int index = threadId + blockDim.x * blockIdx.x;

			cacheBuffer[threadId] = 0;
			cacheBuffer[threadId1] = 0;
			if (index < cellCount)
			{
				const cuBodyAabbCell* hashArray = info.m_bodyAabbCell.m_array;

				unsigned count = 0;
				const cuBodyAabbCell& cell = hashArray[index];

				for (int i = index + 1; cell.m_key == hashArray[i].m_key; i++)
				{
					count++;
				}
				cacheBuffer[threadId1] = count;
			}

			__syncthreads();
			for (int i = 1; i < D_THREADS_PER_BLOCK; i = i << 1)
			{
				int sum = cacheBuffer[threadId1] + cacheBuffer[threadId1 - i];
				__syncthreads();
				cacheBuffer[threadId1] = sum;
				__syncthreads();
			}

			if (index < cellCount)
			{
				unsigned* scan = info.m_histogram.m_array;
				const unsigned offset = (cellCount + D_PREFIX_SCAN_ALIGN____) & (-D_PREFIX_SCAN_ALIGN____);
				scan[offset + index] = cacheBuffer[threadId1];
			}
		}
	};


	cudaStream_t stream = m_context->m_solverComputeStream;
	cuSceneInfo* const infoGpu = m_context->m_sceneInfoGpu;
	
	ndInt32 threads = m_context->m_bodyBufferGpu.GetCount() - 1;
	ndInt32 bodyBlocksCount = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;

	CudaInitBodyArray << <bodyBlocksCount, D_THREADS_PER_BLOCK, 0, stream >> > (CalcuateBodyAabb, *infoGpu);
	CudaMergeAabb << <1, D_THREADS_PER_BLOCK, 0, stream >> > (ReducedAabb, *infoGpu);
	CudaCountAabb << <bodyBlocksCount, D_THREADS_PER_BLOCK, 0, stream >> > (CountAabb, *infoGpu);
	CudaPrefixScan(m_context);
dAssert(SanityCheckPrefix());
	CudaGenerateGridHash << <bodyBlocksCount, D_THREADS_PER_BLOCK, 0, stream >> > (GenerateHashGrids, *infoGpu);
	CudaEndGridHash << <1, 1, 0, stream >> > (EndGridHash, *infoGpu);
	CudaBodyAabbCellSortBuffer(m_context);
dAssert(SanityCheckSortCells());

//	ndInt32 cellsBlocksCount = (m_context->m_bodyAabbCell.m_capacity + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
//	dAssert(cellsBlocksCount > 0);
//	CudaBodyCalculatePairsCount << <cellsBlocksCount, D_THREADS_PER_BLOCK, 0, stream >> > (CalculatePairsCount, *infoGpu);
//dAssert(SanityCheck());
}
