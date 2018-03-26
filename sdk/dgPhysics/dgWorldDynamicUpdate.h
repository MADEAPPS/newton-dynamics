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

#ifndef _DG_WORLD_DYNAMICS_H_
#define _DG_WORLD_DYNAMICS_H_


#include "dgPhysicsStdafx.h"


//#define DG_PSD_DAMP_TOL				dgFloat32 (1.0e-2f)
#define DG_PSD_DAMP_TOL					dgFloat32 (1.0e-3f)
//#define DG_PSD_DAMP_TOL				dgFloat32 (1.0e-4f)


#define	DG_BODY_LRU_STEP				2	
#define	DG_MAX_SKELETON_JOINT_COUNT		256
#define DG_MAX_CONTINUE_COLLISON_STEPS	8
#define	DG_SMALL_ISLAND_COUNT			2

#define	DG_FREEZZING_VELOCITY_DRAG		dgFloat32 (0.9f)
#define	DG_SOLVER_MAX_ERROR				(DG_FREEZE_ACCEL * dgFloat32 (0.5f))

#define DG_HEAVY_MASS_SCALE_FACTOR		dgFloat32 (25.0f)
#define DG_HEAVY_MASS_INV_SCALE_FACTOR	(dgFloat32 (1.0f) / DG_HEAVY_MASS_SCALE_FACTOR)



// the solver is a RK order 4, but instead of weighting the intermediate derivative by the usual 1/6, 1/3, 1/3, 1/6 coefficients
// I am using 1/4, 1/4, 1/4, 1/4.
// This is correct.The weighting coefficients of any RK method comes from an arbitrary criteria
// solving for a set of linear equation on the coefficients. 
// The standard coefficients just happen to lead to an accurate result because they are optimal 
// with respect to a second order solution.For differential equations of higher order, then is not 
// clear if the traditional weigh factors are any better than any other set of weighting factors.
// A different set of coefficient generates results that are not much different than the optimal set, 
// but it allows for simpler calculation of the intermediate derivatives and also for less intermediate memory.
// For more detail on the derivation of the Runge Kutta coefficients you can go to:  
// http://pathfinder.scar.utoronto.ca/~dyer/csca57/book_P/node51.html


class dgBody;
class dgDynamicBody;
class dgParallelSolverSyncData;
class dgWorldDynamicUpdateSyncDescriptor;


class dgClusterCallbackStruct
{
	public:
	dgWorld* m_world;
	dgInt32 m_count;
	dgInt32 m_strideInByte;
	void* m_bodyArray;
};

class dgBodyInfo
{
	public:
	dgBody* m_body;
};

class dgBodyCluster
{
	public:
	dgInt32 m_bodyStart;
	dgInt32 m_bodyCount;
	dgInt32 m_jointStart;	
	dgInt32 m_jointCount;
	dgInt32 m_rowsStart;
	dgInt32 m_rowsCount;
	dgInt32 m_clusterLRU;
	dgInt16 m_isContinueCollision;
	dgInt16 m_hasSoftBodies;
};


class dgJointInfo
{
	public:
	dgConstraint* m_joint;
	dgFloat32 m_scale0;
	dgFloat32 m_scale1;
	dgInt32 m_m0;
	dgInt32 m_m1;
	dgInt32 m_pairStart;
	dgInt32 m_pairCount;
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
	dgInt32 m_bachIndex;
	dgInt32 m_bachCount;
	dgInt32 m_maxPasses;
	dgInt32 m_bodyCount;
	dgInt32 m_jointCount;
	dgInt32 m_rowCount;
	dgInt32 m_atomicIndex;
//	dgInt32 m_lock0;
//	dgInt32 m_lock1;
	dgInt32 m_clusterCount;
	dgInt32 m_jacobianMatrixRowAtomicIndex;

	dgInt32* m_bodyLocks;  
	dgBodyInfo* m_bodyArray;
	dgJointInfo* m_jointsArray;
	dgBodyCluster* m_cluster;
	const dgBodyCluster* m_clusterArray;

	dgInt32 m_hasJointFeeback[DG_MAX_THREADS_HIVE_COUNT];
};


template<class T>
class dgQueue
{
	public:
	dgQueue (T* const pool, dgInt32 size)
		:m_pool (pool)
	{
		m_mod = size;
		m_lastIndex = 0;
		m_firstIndex = 0;
	}

	void Insert (T info) 
	{
		m_pool[m_firstIndex] = info;
		m_firstIndex ++;
		if (m_firstIndex >= m_mod) {
			m_firstIndex = 0;
		}
		dgAssert (m_firstIndex != m_lastIndex);
	}

	T Remove () 
	{
		dgAssert (m_firstIndex != m_lastIndex);

		T element = m_pool[m_lastIndex];
		m_lastIndex ++;
		if (m_lastIndex >= m_mod) {
			m_lastIndex = 0;
		}
		
		return element;
	}

	void Reset ()
	{
		m_lastIndex = m_firstIndex;
	}

	bool IsEmpty () const 
	{
		return (m_firstIndex == m_lastIndex);
	}

	dgInt32 m_mod;
	dgInt32 m_firstIndex;
	dgInt32 m_lastIndex;
	T* m_pool;
};

DG_MSC_VECTOR_ALIGMENT
class dgJacobianMatrixElement
{
	public:
	dgJacobianPair m_Jt;
	dgJacobianPair m_JMinv;

	dgFloat32 m_force;
	dgFloat32 m_jinvMJt;
	dgFloat32 m_diagDamp;
	dgFloat32 m_invJinvMJt;

	dgFloat32 m_deltaAccel;
	dgFloat32 m_coordenateAccel;
	dgFloat32 m_lowerBoundFrictionCoefficent;
	dgFloat32 m_upperBoundFrictionCoefficent;

	dgFloat32 m_maxImpact;
	dgFloat32 m_restitution;
	dgFloat32 m_penetration;
	dgFloat32 m_stiffness;

	dgForceImpactPair* m_jointFeebackForce;
	dgFloat32 m_penetrationStiffness;
	dgInt32 m_normalForceIndex;
} DG_GCC_VECTOR_ALIGMENT;

class dgJacobianMemory
{
	public:
	void Init (dgWorld* const world, dgInt32 rowsCount, dgInt32 bodyCount, dgInt32 blockMatrixSizeInBytes);

	dgJacobian* m_internalForcesBuffer;
	dgJacobianMatrixElement* m_jacobianBuffer;
};

class dgWorldDynamicUpdate
{
	public:
	dgWorldDynamicUpdate();
	void UpdateDynamics (dgFloat32 timestep);
	dgBody* GetClusterBody (const void* const cluster, dgInt32 index) const;

	private:
	void BuildClusters(dgFloat32 timestep);
	dgInt32 SortClusters(const dgBodyCluster* const cluster, dgFloat32 timestep, dgInt32 threadID) const;
	void SpanningTree (dgDynamicBody* const body, dgDynamicBody** const queueBuffer, dgFloat32 timestep);
	
	static dgInt32 CompareClusters (const dgBodyCluster* const clusterA, const dgBodyCluster* const clusterB, void* notUsed);

	static void CalculateClusterReactionForcesKernel (void* const context, void* const worldContext, dgInt32 threadID);

	static void IntegrateInslandParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void InitializeBodyArrayParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void BuildJacobianMatrixParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void SolverInitInternalForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void CalculateJointsForceParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void CalculateJointsVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void KinematicCallbackUpdateParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void UpdateFeedbackForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void UpdateBodyVelocityParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void CalculateBodiesForceParallelKernel(void* const context, void* const worldContext, dgInt32 threadID);
	static void IntegrateClusterParallelParallelKernel(void* const context, void* const worldContext, dgInt32 threadID);

	void IntegrateClusterParallel(dgParallelSolverSyncData* const syncData) const; 
	void InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData) const; 
	void BuildJacobianMatrixParallel (dgParallelSolverSyncData* const syncData) const; 
	void SolverInitInternalForcesParallel (dgParallelSolverSyncData* const syncData) const; 
	void CalculateForcesParallel (dgParallelSolverSyncData* const syncData) const; 

	void CalculateReactionForcesParallel (const dgBodyCluster* const clusters, dgInt32 clustersCount, dgFloat32 timestep) const;
	void CalculateNetAcceleration (dgBody* const body, const dgVector& invTimeStep, const dgVector& accNorm) const;
	void BuildJacobianMatrix (dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const;
	void ResolveClusterForces (dgBodyCluster* const cluste, dgInt32 threadID, dgFloat32 timestep) const;
	void IntegrateReactionsForces(const dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const;
	void CalculateClusterReactionForces (const dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const;
	void CalculateSingleContactReactionForces (const dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const;
	void BuildJacobianMatrix (const dgBodyInfo* const bodyInfo, dgJointInfo* const jointInfo, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgFloat32 forceImpulseScale) const;
		
	dgFloat32 CalculateJointForce(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const;
	dgFloat32 CalculateJointForce_1_50(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const;
	dgFloat32 CalculateJointForce_3_13(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const;
	dgFloat32 CalculateJointForceParallel(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const;

	void SortClustersByCount ();
	void IntegrateExternalForce(const dgBodyCluster* const cluster, dgFloat32 timestep, dgInt32 threadID) const;
	void IntegrateVelocity (const dgBodyCluster* const cluster, dgFloat32 accelTolerance, dgFloat32 timestep, dgInt32 threadID) const;

	void CalculateClusterContacts (dgBodyCluster* const cluster, dgFloat32 timestep, dgInt32 currLru, dgInt32 threadID) const;
	dgInt32 GetJacobianDerivatives (dgContraintDescritor& constraintParamOut, dgJointInfo* const jointInfo, dgConstraint* const constraint, dgJacobianMatrixElement* const matrixRow, dgInt32 rowCount) const;
	
	dgInt32 m_bodies;
	dgInt32 m_joints;
	dgInt32 m_clusters;
	dgInt32 m_markLru;
	dgJacobianMemory m_solverMemory;
	dgThread::dgCriticalSection m_softBodyCriticalSectionLock;
	dgBodyCluster* m_clusterMemory;
	
	static dgVector m_velocTol;
	

	friend class dgWorld;
	friend class dgJacobianMemory;
	friend class dgSkeletonContainer;
	friend class dgSolverWorlkerThreads;
};

#endif

