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

//#include "ndCoreStdafx.h"
//#include "ndNewtonStdafx.h"
//#include "ndWorldSceneCuda.h"

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
#include "cuVector3.h"
#include "cuVector4.h"
#include "cuMatrix3x3.h"

#include "ndCudaContext.h"
#include "ndCudaKernels.h"
#include "ndWorldSceneCuda.h"

ndWorldSceneCuda::ndWorldSceneCuda(ndWorld* const world)
	:ndWorldScene(world)
{
}

ndWorldSceneCuda::~ndWorldSceneCuda()
{
}

void ndWorldSceneCuda::CalculateContacts()
{
	//ndWorldScene::CalculateContacts();
}

void ndWorldSceneCuda::FindCollidingPairs()
{
	//ndWorldScene::FindCollidingPairs();
}

void ndWorldSceneCuda::InitBodyArray()
{
	static int xxx;
	//if (xxx < 10)
	ndWorldScene::InitBodyArray();
	xxx++;
}
