/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgNewtonCpuStdafx.h"
#include "dgNewtonCpu.h"


// This is an example of an exported function.
dgWorldPlugin* GetPlugin()
{
	static dgNewtonCpu module;
	return &module;
}


dgNewtonCpu::dgNewtonCpu()
	:dgWorldPlugin()
	,m_cluster(NULL)
	,m_bodyArray(NULL)
	,m_jointArray(NULL)
	,m_timestep(dgFloat32 (0.0f))
{
}

const char* dgNewtonCpu::GetId() const
{
#ifdef _DEBUG
	return "newtonCpu_d";
#else
	return "newtonCpu";
#endif
}

void dgNewtonCpu::CalculateJointForces(const dgBodyCluster& cluster, const dgBodyInfo* const bodyArray, const dgJointInfo* const jointArray, dgFloat32 timestep)
{
	m_timestep = timestep;
	m_cluster= &cluster;
	m_bodyArray = bodyArray;
	m_jointArray = jointArray;


/*
	dgWorld* const world = (dgWorld*) this;
//	dgWorldPlugin* const plugin = world->m_currentPlugin->GetInfo().m_plugin;
//	plugin->CalculateJointForces(&m_clusterMemory[index], count, timestep);

	dgBodyInfo* const bodyPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgJointInfo* const constraintPtr = (dgJointInfo*)&world->m_jointsMemory[0];
	
	dgBodyInfo* const bodyArray = &bodyPtr[m_bodies];
	dgJointInfo* const jointArray = &constraintPtr[m_joints];

	bodyArray[0].m_body = world->m_sentinelBody;
	dgAssert(world->m_sentinelBody->m_index == 0);
*/

/*
	syncData.m_bodyLocks = dgAlloca(dgInt32, bodyIndex);
	memset(syncData.m_bodyLocks, 0, bodyIndex * sizeof(dgInt32));

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	internalForces[0].m_linear = dgVector::m_zero;
	internalForces[0].m_angular = dgVector::m_zero;

	const dgInt32 rkSubSteps = 4;
	syncData.m_timestep = timestep;
	syncData.m_invTimestep = (timestep > dgFloat32(0.0f)) ? dgFloat32(1.0f) / timestep : dgFloat32(0.0f);
	syncData.m_invStepRK = (dgFloat32(1.0f) / dgFloat32(rkSubSteps));
	syncData.m_timestepRK = syncData.m_timestep * syncData.m_invStepRK;
	syncData.m_invTimestepRK = syncData.m_invTimestep * dgFloat32(rkSubSteps);
	syncData.m_rkSubSteps = rkSubSteps;
	syncData.m_passes = world->m_solverMode;
	syncData.m_passes = 16;

	syncData.m_bodyCount = bodyIndex;
	syncData.m_jointCount = jointsCount;
	syncData.m_bodyArray = bodyArray;
	syncData.m_jointsArray = jointArray;
	syncData.m_atomicIndex = 0;

	syncData.m_clusterCount = clustersCount;
	syncData.m_clusterArray = clusterArray;

	syncData.m_cluster = &cluster;
	syncData.m_weight = dgAlloca(dgFloat32, cluster.m_bodyCount * 2);
	memset(syncData.m_weight, 0, sizeof(dgFloat32) * cluster.m_bodyCount);

	for (dgInt32 i = 0; i < jointsCount; i++) {
		dgJointInfo* const jointInfo = &jointArray[i];
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		syncData.m_weight[m0] += dgFloat32(1.0f);
		syncData.m_weight[m1] += dgFloat32(1.0f);
	}

	for (dgInt32 i = 0; i < cluster.m_bodyCount; i++) {
		const dgFloat32 weight = syncData.m_weight[i] ? syncData.m_weight[i] : dgFloat32(1.0f);
		syncData.m_weight[i + cluster.m_bodyCount] = weight;
		syncData.m_weight[i] = dgFloat32(1.0f) / weight;
	}
*/

}