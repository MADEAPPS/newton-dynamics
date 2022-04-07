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

#include "ndCudaContext.h"
#include "ndWorldSceneCuda.h"

#define D_CUDA_SCENE_GRID_SIZE		8.0f
#define D_CUDA_SCENE_INV_GRID_SIZE	(1.0f/D_CUDA_SCENE_GRID_SIZE) 

template <typename Predicate>
__global__ void CudaAddBodyPadding(Predicate PaddLastBlock, cuBodyProxy* bodyArray, int blocksCount, int sentinelIndex)
{
	PaddLastBlock(bodyArray, blocksCount, sentinelIndex);
}

template <typename Predicate>
__global__ void CudaMergeAabb(Predicate ReducedAabb, ndGpuInfo* const info, cuBoundingBox* bBox, ndInt32 count)
{
	ReducedAabb(*info, bBox, count);
}

template <typename Predicate>
__global__ void CudaInitBodyArray(Predicate UpdateBodyScene, cuBodyProxy* bodyArray, cuBoundingBox* bBox)
{
	UpdateBodyScene(bodyArray, bBox);
}

template <typename Predicate>
__global__ void CudaGetBodyTransforms(Predicate GetTransform, const cuBodyProxy* const srcBuffer, cuSpatialVector* const dstBuffer, int size)
{
	int index = threadIdx.x + blockDim.x * blockIdx.x;
	if (index < size)
	{
		GetTransform(srcBuffer[index], dstBuffer[index]);
	}
}

template <typename Predicate>
__global__ void CudaCountAabb(Predicate CountAabb, ndGpuInfo* const info, cuBodyProxy* bodyArray, int* scan)
{
	CountAabb(*info, bodyArray, scan);
}

template <typename Predicate>
__global__ void CudaPrefixScanSum0(Predicate PrefixScan, int* scan)
{
	PrefixScan(scan);
}

template <typename Predicate>
__global__ void CudaPrefixScanSum1(Predicate PrefixScan, int* scan, int size)
{
	PrefixScan(scan, size);
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

void ndWorldSceneCuda::Sync()
{
	ndScene::Sync();

	//syncronize all streams before starting a new frame.
	//this is pretty horrendous function, need to find a beter method
	//cudaDeviceSynchronize();
	//cudaStreamSynchronize(m_context->m_stream0);
	m_context->SwapBuffers();
}

void ndWorldSceneCuda::FindCollidingPairs(ndBodyKinematic* const body)
{
	dAssert(0);
}

void ndWorldSceneCuda::FindCollidingPairs()
{
	//ndWorldScene::FindCollidingPairs();
}

void ndWorldSceneCuda::CalculateContacts(ndInt32 threadIndex, ndContact* const contact)
{
	dAssert(0);
}

void ndWorldSceneCuda::CalculateContacts()
{
	//ndWorldScene::CalculateContacts();
}

void ndWorldSceneCuda::LoadBodyData()
{
	auto UploadBodies = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndVector minBox(ndFloat32(1.0e15f));
		const ndVector maxBox(ndFloat32(-1.0e15f));

		ndArray<cuBodyProxy>& data = m_context->m_bodyBufferCpu;
		cuHostBuffer<cuSpatialVector>& transformBufferCpu0 = m_context->m_transformBufferCpu0;
		cuHostBuffer<cuSpatialVector>& transformBufferCpu1 = m_context->m_transformBufferCpu1;

		ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
		//const ndStartEnd startEnd(bodyArray.GetCount() - 1, threadIndex, threadCount);
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

	ndArray<cuBodyProxy>& bodyBufferCpu = m_context->m_bodyBufferCpu;
	cuDeviceBuffer<cuBodyProxy>& bodyBufferGpu = m_context->m_bodyBufferGpu;
	const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
	cuDeviceBuffer<int>& scan = m_context->m_scan;
	cuDeviceBuffer<cuBoundingBox>& boundingBoxGpu = m_context->m_boundingBoxGpu;
	cuDeviceBuffer<cuSpatialVector>& transformBufferGpu = m_context->m_transformBufferGpu;
	cuHostBuffer<cuSpatialVector>& transformBufferCpu0 = m_context->m_transformBufferCpu0;
	cuHostBuffer<cuSpatialVector>& transformBufferCpu1 = m_context->m_transformBufferCpu1;

	const ndInt32 cpuBodyCount = bodyArray.GetCount();
	const ndInt32 gpuBodyCount = D_THREADS_PER_BLOCK * ((cpuBodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK);
	
	bodyBufferCpu.SetCount(cpuBodyCount);
	bodyBufferGpu.SetCount(gpuBodyCount);
	transformBufferGpu.SetCount(cpuBodyCount);
	transformBufferCpu0.SetCount(cpuBodyCount);
	transformBufferCpu1.SetCount(cpuBodyCount);
	scan.SetCount(gpuBodyCount);
	boundingBoxGpu.SetCount(gpuBodyCount / D_THREADS_PER_BLOCK);

	ParallelExecute(UploadBodies);
	bodyBufferGpu.ReadData(&bodyBufferCpu[0], cpuBodyCount);
}

void ndWorldSceneCuda::GetBodyTransforms()
{
	D_TRACKTIME();

	auto GetTransform = [] __device__(const cuBodyProxy& body, cuSpatialVector& transform)
	{
		transform.m_linear = body.m_posit;
		transform.m_angular = body.m_rotation;
	};

	if (m_context->m_bodyBufferGpu.GetCount())
	{
		cuHostBuffer<cuSpatialVector>& cpuBuffer = m_context->m_transformBufferCpu0;
		cuDeviceBuffer<cuSpatialVector>& gpuBuffer = m_context->m_transformBufferGpu;

		ndInt32 threads = m_context->m_bodyBufferGpu.GetCount();
		ndInt32 blocks = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		cuBodyProxy* const bodiesGpu = &m_context->m_bodyBufferGpu[0];

		gpuBuffer.SetCount(threads);
		cpuBuffer.SetCount(threads);
		cudaStream_t stream = m_context->m_stream0;
		CudaGetBodyTransforms <<<blocks, D_THREADS_PER_BLOCK, 0, stream>>> (GetTransform, bodiesGpu, &gpuBuffer[0], threads);
		gpuBuffer.WriteData(&cpuBuffer[0], cpuBuffer.GetCount(), stream);
	}
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
	
	//ndScene::UpdateTransform();
}

void ndWorldSceneCuda::UpdateBodyList()
{
	D_TRACKTIME();
	bool bodyListChanged = m_bodyListChanged;
	ndWorldScene::UpdateBodyList();
	if (bodyListChanged)
	{
		LoadBodyData();
		cudaDeviceSynchronize();
	}
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

	auto ReducedAabb = [] __device__(ndGpuInfo& info, cuBoundingBox* bBoxOut, int index)
	{
		__shared__  cuBoundingBox aabb[D_THREADS_PER_BLOCK];

		aabb[threadIdx.x].m_min = bBoxOut[threadIdx.x].m_min;
		aabb[threadIdx.x].m_max = bBoxOut[threadIdx.x].m_max;
		__syncthreads();

		if (threadIdx.x >= index)
		{
			aabb[threadIdx.x] = aabb[index - 1];
		}
		__syncthreads();

		for (int i = D_THREADS_PER_BLOCK / 2; i; i = i >> 1)
		{
			if (threadIdx.x < i)
			{
				aabb[threadIdx.x].m_min = aabb[threadIdx.x].m_min.Min(aabb[threadIdx.x + i].m_min);
				aabb[threadIdx.x].m_max = aabb[threadIdx.x].m_max.Max(aabb[threadIdx.x + i].m_max);
			}
			__syncthreads();
		}

		if (threadIdx.x == 0)
		{
			cuVector minBox((aabb[0].m_min.Scale(D_CUDA_SCENE_INV_GRID_SIZE).Floor()).Scale(D_CUDA_SCENE_GRID_SIZE));
			cuVector maxBox((aabb[0].m_max.Scale(D_CUDA_SCENE_INV_GRID_SIZE).Floor()).Scale(D_CUDA_SCENE_GRID_SIZE) + cuVector(D_CUDA_SCENE_GRID_SIZE));
			minBox.w = 0.0f;
			maxBox.w = 0.0f;
			info.m_worldBox.m_min = minBox;
			info.m_worldBox.m_max = maxBox;
		}
	};

	auto PaddLastBodyBlock = [] __device__(cuBodyProxy* bodyArray, int blocksCount, int sentinelIndex)
	{
		int index = (blocksCount - 1) * blockDim.x + threadIdx.x;
		if (index == sentinelIndex)
		{
			bodyArray[sentinelIndex].m_posit = bodyArray[sentinelIndex - 1].m_posit;
			bodyArray[sentinelIndex].m_rotation = bodyArray[sentinelIndex - 1].m_rotation;
		}
		__syncthreads();

		if (index > sentinelIndex)
		{
			bodyArray[index].m_rotation = bodyArray[sentinelIndex].m_rotation;
			bodyArray[index].m_posit = bodyArray[sentinelIndex].m_posit;
			bodyArray[index].m_obbSize = bodyArray[sentinelIndex].m_obbSize;
			bodyArray[index].m_obbOrigin = bodyArray[sentinelIndex].m_obbOrigin;
			bodyArray[index].m_scale = bodyArray[sentinelIndex].m_scale;
			bodyArray[index].m_localPosition = bodyArray[sentinelIndex].m_localPosition;
			bodyArray[index].m_localRotation = bodyArray[sentinelIndex].m_localRotation;
			bodyArray[index].m_alignRotation = bodyArray[sentinelIndex].m_alignRotation;
		}
	};

	auto UpdateAabb = [] __device__(cuBodyProxy* bodyArray, cuBoundingBox* bBox)
	{
		__shared__  cuBoundingBox aabb[D_THREADS_PER_BLOCK];

		int index = threadIdx.x + blockDim.x * blockIdx.x;
		cuBodyProxy& body = bodyArray[index];

		// calculate shape global Matrix
		body.m_globalSphapeRotation = body.m_localRotation * body.m_rotation;
		cuMatrix3x3 matrix(body.m_globalSphapeRotation.GetMatrix3x3());
		body.m_globalSphapePosition = matrix.RotateVector(body.m_localPosition) + body.m_posit;

		// calculate world aabb
		//ndMatrix scaleMatrix;
		//scaleMatrix[0] = matrix[0].Scale(m_scale.m_x);
		//scaleMatrix[1] = matrix[1].Scale(m_scale.m_y);
		//scaleMatrix[2] = matrix[2].Scale(m_scale.m_z);
		//scaleMatrix[3] = matrix[3];
		//scaleMatrix = m_alignmentMatrix * scaleMatrix;
		matrix.m_front = matrix.m_front.Scale(body.m_scale.x);
		matrix.m_up    = matrix.m_up.Scale(body.m_scale.y);
		matrix.m_right = matrix.m_right.Scale(body.m_scale.z);
		matrix = body.m_alignRotation.GetMatrix3x3() * matrix;

		//const ndVector size0(m_shape->GetObbSize());
		//const ndVector size(scaleMatrix.m_front.Abs().Scale(size0.m_x) + scaleMatrix.m_up.Abs().Scale(size0.m_y) + scaleMatrix.m_right.Abs().Scale(size0.m_z));
		//const ndVector origin(scaleMatrix.TransformVector(m_shape->GetObbOrigin()));
		const cuVector origin(matrix.RotateVector(body.m_obbOrigin) + body.m_globalSphapePosition);
		const cuVector size(matrix.m_front.Abs().Scale(body.m_obbSize.x) + matrix.m_up.Abs().Scale(body.m_obbSize.y) + matrix.m_right.Abs().Scale(body.m_obbSize.z));

		//p0 = (origin - size - m_padding) & ndVector::m_triplexMask;
		//p1 = (origin + size + m_padding) & ndVector::m_triplexMask;
		const cuVector padding(1.0f / 16.0f);
		const cuVector minBox(origin - size - padding);
		const cuVector maxBox(origin + size + padding);


		int threadId = threadIdx.x;
		// save aabb and calculate bonding box for this thread block
		body.m_minAabb = minBox;
		body.m_maxAabb = maxBox;
		aabb[threadId].m_min = minBox;
		aabb[threadId].m_max = maxBox;
		__syncthreads();

		for (int i = D_THREADS_PER_BLOCK / 2; i; i = i >> 1)
		{
			if (threadId < i)
			{
				aabb[threadId].m_min = aabb[threadIdx.x].m_min.Min(aabb[threadId + i].m_min);
				aabb[threadId].m_max = aabb[threadIdx.x].m_max.Max(aabb[threadId + i].m_max);
			}
			__syncthreads();
		}

		if (threadId == 0)
		{
			bBox[blockIdx.x].m_min = aabb[0].m_min;
			bBox[blockIdx.x].m_max = aabb[0].m_max;
		}
	};

	auto CountAabb = [] __device__(const ndGpuInfo& info, const cuBodyProxy* bodyArray, int* scan)
	{
		__shared__  cuBoundingBox cacheAabb;
		if (threadIdx.x == 0)
		{
			cacheAabb.m_min = info.m_worldBox.m_min;
			cacheAabb.m_max = info.m_worldBox.m_max;
		}
		__syncthreads();

		const cuVector minBox(cacheAabb.m_min);
		//const cuVector maxBox(cacheAabb.m_max);

		int index = threadIdx.x + blockDim.x * blockIdx.x;
		const cuVector bodyBoxMin(bodyArray[index].m_minAabb);
		const cuVector bodyBoxMax(bodyArray[index].m_maxAabb);

		int x0 = __float2int_rd((bodyBoxMin.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE);
		int y0 = __float2int_rd((bodyBoxMin.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE);
		int z0 = __float2int_rd((bodyBoxMin.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE);
		int x1 = __float2int_rd((bodyBoxMax.x - minBox.x) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		int y1 = __float2int_rd((bodyBoxMax.y - minBox.y) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		int z1 = __float2int_rd((bodyBoxMax.z - minBox.z) * D_CUDA_SCENE_INV_GRID_SIZE) + 1;
		int count = (z1 - z0) * (y1 - y0) * (x1 - x0);
		scan[index] = count;
	};

	auto PrefixScanSum0 = [] __device__(int* scan)
	{
		__shared__  int cacheBuffer[2 * D_THREADS_PER_BLOCK];

		int index = threadIdx.x + blockDim.x * blockIdx.x;

		cacheBuffer[threadIdx.x] = 0;
		int threadId = threadIdx.x + D_THREADS_PER_BLOCK;
		cacheBuffer[threadId] = scan[index];
		__syncthreads();
		
		for (int i = 1; i < D_THREADS_PER_BLOCK; i = i << 1)
		{
			int sum = cacheBuffer[threadId] + cacheBuffer[threadId - i];
			__syncthreads();
			cacheBuffer[threadId] = sum;
			__syncthreads();
		}
		scan[index] = cacheBuffer[threadId];
	};

	auto PrefixScanSum1 = [] __device__(int* scan, int size)
	{
		int threadId = threadIdx.x;
		const int blocks = size / D_THREADS_PER_BLOCK;
		for (int i = 1; i < blocks; i ++)
		{
			int sum = scan[i * D_THREADS_PER_BLOCK - 1];
			__syncthreads();
			scan[i * D_THREADS_PER_BLOCK + threadId] += sum;
			__syncthreads();
		}
	};


	ndGpuInfo* const info = m_context->m_sceneInfo;
	cudaStream_t stream = m_context->m_stream0;
	ndInt32 threads = m_context->m_bodyBufferGpu.GetCount();
	ndInt32 blocksCount = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	dAssert(blocksCount < D_THREADS_PER_BLOCK);
	dAssert(blocksCount * D_THREADS_PER_BLOCK == threads);

	int* const scan = &m_context->m_scan[0];
	cuBodyProxy* const bodiesGpu = &m_context->m_bodyBufferGpu[0];
	cuBoundingBox* const bBoxGpu = &m_context->m_boundingBoxGpu[0];

	ndInt32 sentinelIndex = m_context->m_bodyBufferCpu.GetCount() - 1;
	CudaAddBodyPadding << <1, D_THREADS_PER_BLOCK, 0, stream >> > (PaddLastBodyBlock, bodiesGpu, blocksCount, sentinelIndex);
	CudaInitBodyArray << <blocksCount, D_THREADS_PER_BLOCK, 0, stream >> > (UpdateAabb, bodiesGpu, bBoxGpu);
	CudaMergeAabb << <1, D_THREADS_PER_BLOCK, 0, stream >> > (ReducedAabb, info, bBoxGpu, blocksCount);

	CudaCountAabb << <blocksCount, D_THREADS_PER_BLOCK, 0, stream >> > (CountAabb, info, bodiesGpu, scan);
	CudaPrefixScanSum0 << <blocksCount, D_THREADS_PER_BLOCK, 0, stream >> > (PrefixScanSum0, scan);
	CudaPrefixScanSum1 << <1, D_THREADS_PER_BLOCK, 0, stream >> > (PrefixScanSum1, scan, threads);
}
