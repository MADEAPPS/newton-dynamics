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
}

ndWorldSceneCuda::~ndWorldSceneCuda()
{
	if (m_context)
	{
		delete m_context;
	}
}

void ndWorldSceneCuda::FindCollidingPairs(ndBodyKinematic* const body)
{
	dAssert(0);
}

void ndWorldSceneCuda::CalculateContacts(ndInt32 threadIndex, ndContact* const contact)
{
	dAssert(0);
}

void ndWorldSceneCuda::CalculateContacts()
{
	//ndWorldScene::CalculateContacts();
}

void ndWorldSceneCuda::FindCollidingPairs()
{
	//ndWorldScene::FindCollidingPairs();
}

void ndWorldSceneCuda::LoadBodyData()
{
	//const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
	ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
	const ndInt32 bodyCount = bodyArray.GetCount();
	
	ndBodyBuffer& gpuBodyBuffer = m_context->m_bodyBuffer;
	ndArray<ndBodyProxy>& data = gpuBodyBuffer.m_dataView;
	
	gpuBodyBuffer.SetCount(bodyCount);
	gpuBodyBuffer.m_dataView.SetCount(bodyCount);
	
	for (ndInt32 i = 0; i < bodyCount; i++)
	{
		ndBodyKinematic* const body = bodyArray[i];
		ndBodyProxy& proxi = data[i];
		proxi.BodyToProxy(body);
	}
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
		ndArray<cuSpatialVector>& cpuBuffer = m_context->m_transformBufferCpu;
		cuDeviceBuffer<cuSpatialVector>& gpuBuffer = m_context->m_transformBufferGpu;

		ndInt32 threads = m_context->m_bodyBuffer.GetCount();
		ndInt32 blocks = (threads + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
		ndBodyProxy* bodies = &m_context->m_bodyBuffer[0];

		gpuBuffer.SetCount(threads);
		cpuBuffer.SetCount(threads);
		CudaKernel2 << <blocks, D_THREADS_PER_BLOCK >> > (GetTransform, bodies, &gpuBuffer[0], threads);
		gpuBuffer.WriteData(&cpuBuffer[0], cpuBuffer.GetCount());
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
		const cuSpatialVector* const data = &m_context->m_transformBufferCpu[0];
		const ndStartEnd startEnd(bodyArray.GetCount() - 1, threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			const cuSpatialVector& transform = data[i];
			const ndVector position(transform.m_linear.x, transform.m_linear.y, transform.m_linear.z, ndFloat32(1.0f));
			const ndQuaternion rotation(ndVector(transform.m_angular.x, transform.m_angular.y, transform.m_angular.z, transform.m_angular.w));
			body->SetMatrixAndCentreOfMass(rotation, position);

			body->m_transformIsDirty = true;
		}
	});
	ParallelExecute(SetTransform);
	
	ndScene::UpdateTransform();
}

void ndWorldSceneCuda::UpdateBodyList()
{
	bool bodyListChanged = m_bodyListChanged;
	ndWorldScene::UpdateBodyList();
	if (bodyListChanged)
	{
		LoadBodyData();
	}
}

void ndWorldSceneCuda::InitBodyArray()
{
	//ndWorldScene::InitBodyArray();

	//const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
	// hack to update transform for now.
	//for (ndInt32 i = 0; i < bodyArray.GetCount()-1; ++i)
	//{
	//	ndBodyKinematic* const body = bodyArray[i];
	//	body->m_transformIsDirty = true;
	//}
}
