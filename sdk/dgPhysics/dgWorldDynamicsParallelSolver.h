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

#ifndef _DG_PARALLEL_SOLVER_H_
#define _DG_PARALLEL_SOLVER_H_


#include "dgPhysicsStdafx.h"

class dgBodyInfo;
class dgJointInfo;
class dgBodyCluster;

#define DG_WORK_GROUP_BITS	3
#define DG_WORK_GROUP_SIZE	(1<<DG_WORK_GROUP_BITS) 


DG_MSC_VECTOR_ALIGMENT
class dgWorkGroupFloat
{
	public:
	DG_INLINE dgWorkGroupFloat()
	{
	}

	dgVector m_value[2];
} DG_GCC_VECTOR_ALIGMENT;



template<class T>
class dgParallelVector: public dgArray<T>
{
	public:
	dgParallelVector(dgMemoryAllocator* const allocator, dgInt32 aligmentInBytes = DG_MEMORY_GRANULARITY)
		:dgArray<T>(allocator, aligmentInBytes)
		, m_ptr(NULL)
	{
	}

	void Reserve(dgInt32 count)
	{
		ResizeIfNecessary(count);
		dgArray<T>& me = *this;
		m_ptr = &me[0];
	}

	DG_INLINE T& operator[] (dgInt32 i)
	{
		return m_ptr[i];
	}

	DG_INLINE const T& operator[] (dgInt32 i) const
	{
		return m_ptr[i];
	}

	T* m_ptr;
};

class dgParallelSolverSyncData
{
	public:
	dgParallelSolverSyncData()
	{
		memset (this, 0, sizeof (dgParallelSolverSyncData));
	}

	dgFloat32 m_accelNorm[DG_MAX_THREADS_HIVE_COUNT];

	dgFloat32 m_timestep;
	dgFloat32 m_invTimestep;
	dgFloat32 m_invStepRK;
	dgFloat32 m_timestepRK;
	dgFloat32 m_invTimestepRK;
	dgFloat32 m_firstPassCoef;

	dgInt32 m_passes;
	dgInt32 m_rkSubSteps;
	dgInt32 m_bodyCount;
	dgInt32 m_jointCount;
	dgInt32 m_rowCount;
	dgInt32 m_atomicIndex;
	dgInt32 m_clusterCount;
	dgInt32 m_jacobianMatrixRowAtomicIndex;

	dgInt32* m_bodyLocks;  
	dgBodyInfo* m_bodyArray;
	dgJointInfo* m_jointsArray;
	dgBodyCluster* m_cluster;
	dgFloat32* m_weight;
	const dgBodyCluster* m_clusterArray;

	dgInt32 m_hasJointFeeback[DG_MAX_THREADS_HIVE_COUNT];
};

class dgParallelBodySolver
{
	public:

	dgParallelBodySolver(dgMemoryAllocator* const allocator);
	~dgParallelBodySolver();
	void CalculateJointForces(dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep);

	private:
	void Reserve (dgInt32 count);
	void InitWeights();
	void InitInvWeights();
	void InityBodyArray();

	static void InitWeightKernel (void* const context, void* const, dgInt32 threadID);
	static void InitInvWeightKernel (void* const context, void* const, dgInt32 threadID);


	dgParallelVector<dgWorkGroupFloat> m_weigh;
	dgParallelVector<dgWorkGroupFloat> m_invWeigh;

	dgWorld* m_world;
	dgBodyCluster* m_cluster;
	dgJointInfo* m_jointArray;
	dgInt32 m_count;
	dgInt32 m_atomicIndex;

	friend class dgWorldDynamicUpdate;
};


#endif

