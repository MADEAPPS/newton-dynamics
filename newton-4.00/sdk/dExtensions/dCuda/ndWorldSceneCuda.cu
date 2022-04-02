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
#include "ndCudaKernels.h"
#include "ndWorldSceneCuda.h"

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

		ndBodyBuffer& gpuBodyBuffer = m_context->m_bodyBuffer;
		ndArray<ndBodyProxy>& data = gpuBodyBuffer.m_dataView;

		cuHostBuffer<cuSpatialVector>& transformBufferCpu0 = m_context->m_transformBufferCpu0;
		cuHostBuffer<cuSpatialVector>& transformBufferCpu1 = m_context->m_transformBufferCpu1;

		ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
		const ndStartEnd startEnd(bodyArray.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			cuSpatialVector transform;
			ndBodyKinematic* const body = bodyArray[i];
			ndBodyProxy& proxi = data[i];

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
			proxi.m_obbSize = shape->GetObbSize();
			proxi.m_obbOrigin = shape->GetObbOrigin();
			proxi.m_minAabb = minBox;
			proxi.m_maxAabb = maxBox;

			transform.m_angular = cuQuat(body->GetRotation());
			transform.m_linear = body->GetGlobalGetCentreOfMass();
			transformBufferCpu0[i] = transform;
			transformBufferCpu1[i] = transform;
		}
	});

	ndBodyBuffer& gpuBodyBuffer = m_context->m_bodyBuffer;
	ndArray<ndBodyProxy>& data = gpuBodyBuffer.m_dataView;
	const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
	cuHostBuffer<cuSpatialVector>& transformBufferCpu0 = m_context->m_transformBufferCpu0;
	cuHostBuffer<cuSpatialVector>& transformBufferCpu1 = m_context->m_transformBufferCpu1;

	const ndInt32 bodyCount = bodyArray.GetCount();
	
	gpuBodyBuffer.SetCount(bodyCount);
	transformBufferCpu0.SetCount(bodyCount);
	transformBufferCpu1.SetCount(bodyCount);
	gpuBodyBuffer.m_dataView.SetCount(bodyCount);

	ParallelExecute(UploadBodies);
	gpuBodyBuffer.ReadData(&data[0], bodyCount);
}

void ndWorldSceneCuda::GetBodyTransforms()
{
	D_TRACKTIME();

	auto GetTransform = [] __device__(const ndBodyProxy& body, cuSpatialVector& transform)
	{
		transform.m_linear = body.m_posit;
		transform.m_angular = body.m_rotation;
	};

	if (m_context->m_bodyBuffer.GetCount())
	{
		cuHostBuffer<cuSpatialVector>& cpuBuffer = m_context->m_transformBufferCpu0;
		cuDeviceBuffer<cuSpatialVector>& gpuBuffer = m_context->m_transformBufferGpu;

		ndInt32 threads = m_context->m_bodyBuffer.GetCount();
		ndInt32 blocks = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		ndBodyProxy* bodies = &m_context->m_bodyBuffer[0];

		gpuBuffer.SetCount(threads);
		cpuBuffer.SetCount(threads);
		cudaStream_t stream = m_context->m_stream0;
		CudaKernel2 <<<blocks, D_THREADS_PER_BLOCK, 0, stream>>> (GetTransform, bodies, &gpuBuffer[0], threads);
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
}
