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
	const ndArray<ndBodyKinematic*>& bodyArray = GetActiveBodyArray();
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


void ndWorldSceneCuda::InitBodyArray()
{
	bool bodyListChanged = m_bodyListChanged;
	ndWorldScene::InitBodyArray();

	if (bodyListChanged)
	{
		//ndWorldScene::InitBodyArray();
		LoadBodyData();
	}
}
