/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_WORLD_DYNAMICS_UPDATE_H__
#define __D_WORLD_DYNAMICS_UPDATE_H__

#include "ndNewtonStdafx.h"
//#include "dgWorldDynamicsParallelSolver.h"

#if 0
#define	DG_BODY_LRU_STEP					2	
#define	DG_MAX_SKELETON_JOINT_COUNT			256
#define DG_MAX_CONTINUE_COLLISON_STEPS		8
#define	DG_SMALL_ISLAND_COUNT				32

#define	DG_FREEZZING_VELOCITY_DRAG			dFloat32 (0.9f)
#define	DG_SOLVER_MAX_ERROR					(DG_FREEZE_MAG * dFloat32 (0.5f))

#define DG_CCD_EXTRA_CONTACT_COUNT			(8 * 3)
#define DG_PARALLEL_JOINT_COUNT_CUT_OFF		(64)
//#define DG_PARALLEL_JOINT_COUNT_CUT_OFF	(2)

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
class dgWorldDynamicUpdateSyncDescriptor;

class dgClusterCallbackStruct
{
	public:
	dgWorld* m_world;
	dInt32 m_count;
	dInt32 m_strideInByte;
	void* m_bodyArray;
};

class dgBodyInfo
{
	public:
	dgBody* m_body;
};

class dgJointInfo
{
	public:
	union 
	{
		struct
		{
			dgBody* m_body;
			dInt32 m_bodyCount;
			dInt32 m_jointCount;
			dInt32 m_setId;
			dInt32 m_unUsed;
		};
		struct 
		{
			dgConstraint* m_joint;
			dInt32 m_m0;
			dInt32 m_m1;
			dInt32 m_pairStart;
			dInt32 m_pairCount;
		};
	};
	dFloat32 m_preconditioner0;
	dFloat32 m_preconditioner1;
};

class dgBodyJacobianPair
{
	public:
	dInt32 m_bodyIndex;
	dInt32 m_JointIndex;
};

class dgBodyCluster
{
	public:
	dInt32 m_bodyCount;
	dInt32 m_jointCount;
	dInt32 m_rowCount;
	dInt32 m_bodyStart;
	dInt32 m_jointStart;	
	dInt32 m_rowStart;
	dgInt16 m_hasSoftBodies;
	dgInt16 m_isContinueCollision;
};

class dgJointImpulseInfo
{
	public:
	dgContact* m_joint;
	dInt32 m_m0;
	dInt32 m_m1;
	dInt32 m_pairStart;
	dInt32 m_pairCount;
	dInt32 m_rhsStart;
};

template<class T>
class dgQueue
{
	public:
	dgQueue (T* const pool, dInt32 size)
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

	dInt32 m_mod;
	dInt32 m_firstIndex;
	dInt32 m_lastIndex;
	T* m_pool;
};

DG_MSC_VECTOR_ALIGNMENT
class dgLeftHandSide
{
	public:
	dgJacobianPair m_Jt;
	dgJacobianPair m_JMinv;
} DG_GCC_VECTOR_ALIGNMENT;


DG_MSC_VECTOR_ALIGNMENT
class dgRightHandSide
{
	public:
	dFloat32 m_force;
	dFloat32 m_diagDamp;
	dFloat32 m_invJinvMJt;
	dFloat32 m_coordenateAccel;

	dFloat32 m_lowerBoundFrictionCoefficent;
	dFloat32 m_upperBoundFrictionCoefficent;
	dFloat32 m_deltaAccel;
	dFloat32 m_restitution;

	dFloat32 m_maxImpact;
	dFloat32 m_penetration;
	dFloat32 m_diagonalRegularizer;
	dFloat32 m_penetrationStiffness;

	dgForceImpactPair* m_jointFeebackForce;
	dInt32 m_normalForceIndex;
} DG_GCC_VECTOR_ALIGNMENT;

class dgJacobianMemory
{
	public:
	dgJacobianMemory() {}
	void Init (dgWorld* const world, dInt32 rowsCount, dInt32 bodyCount);

	dgJacobian* m_internalForcesBuffer;
	dgLeftHandSide* m_leftHandSizeBuffer;
	dgRightHandSide* m_righHandSizeBuffer;
};
#endif

D_MSV_NEWTON_ALIGN_32
class ndDynamicsUpdate
{
	public:
	//class dgParallelClusterArray;

	//void DynamicsUpdate (dFloat32 timestep);
	//dgBody* GetClusterBody (const void* const cluster, dInt32 index) const;
	//
	//dgJacobianMemory& GetSolverMemory() { return m_solverMemory; }
	//virtual dInt32 GetJacobianDerivatives (dgContraintDescritor& constraintParamOut, dgJointInfo* const jointInfo, dgConstraint* const constraint, dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide, dInt32 rowCount) const;
	//virtual void CalculateNetAcceleration (dgBody* const body, const dgVector& invTimeStep, const dgVector& accNorm) const;
	//
	//private:
	//static DG_INLINE dInt32 CompareKey(dInt32 highA, dInt32 lowA, dInt32 highB, dInt32 lowB);
	//static dInt32 CompareJointInfos(const dgJointInfo* const infoA, const dgJointInfo* const infoB, void* notUsed);
	//static dInt32 CompareClusterInfos (const dgBodyCluster* const clusterA, const dgBodyCluster* const clusterB, void* notUsed);
	//
	//void BuildClusters(dFloat32 timestep);
	//
	//dgBodyCluster MergeClusters(const dgBodyCluster* const clusterArray, dInt32 clustersCount) const;
	//dInt32 SortClusters(const dgBodyCluster* const cluster, dFloat32 timestep, dInt32 threadID) const;
	//
	//static dInt32 CompareBodyJacobianPair(const dgBodyJacobianPair* const infoA, const dgBodyJacobianPair* const infoB, void* notUsed);
	//static void IntegrateClustersParallelKernel (void* const context, void* const worldContext, dInt32 threadID);
	//static void CalculateClusterReactionForcesKernel (void* const context, void* const worldContext, dInt32 threadID);
	//
	//void BuildJacobianMatrix (dgBodyCluster* const cluster, dInt32 threadID, dFloat32 timestep) const;
	//void ResolveClusterForces (dgBodyCluster* const cluster, dInt32 threadID, dFloat32 timestep) const;
	//void IntegrateReactionsForces(const dgBodyCluster* const cluster, dInt32 threadID, dFloat32 timestep) const;
	//void BuildJacobianMatrix (const dgBodyInfo* const bodyInfo, dgJointInfo* const jointInfo, dgJacobian* const internalForces, dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide, dFloat32 forceImpulseScale) const;
	//void CalculateClusterReactionForces(const dgBodyCluster* const cluster, dInt32 threadID, dFloat32 timestep) const;
	//
	//void IntegrateInslandParallel(dgParallelClusterArray* const clusters, dInt32 threadID);
	//void CalculateReactionForcesParallel(const dgBodyCluster* const clusters, dInt32 clustersCount, dFloat32 timestep);
	//
	//dFloat32 CalculateJointForce(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, const dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide) const;
	//dFloat32 CalculateJointForce_3_13(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, const dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide) const;
	//dgJacobian IntegrateForceAndToque(dgDynamicBody* const body, const dgVector& force, const dgVector& torque, const dgVector& timestep) const ;
	//
	//void IntegrateExternalForce(const dgBodyCluster* const cluster, dFloat32 timestep, dInt32 threadID) const;
	//void IntegrateVelocity (const dgBodyCluster* const cluster, dFloat32 accelTolerance, dFloat32 timestep, dInt32 threadID) const;
	//void CalculateClusterContacts (dgBodyCluster* const cluster, dFloat32 timestep, dInt32 currLru, dInt32 threadID) const;
	//
	//void CalculateImpulseVeloc(dgJointImpulseInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dFloat32* const contactVeloc) const;
	//void ResolveImpulse(const dgJointInfo* const constraintArray, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, dgDownHeap<dgContact*, dFloat32>& impactJoints) const;
	//dFloat32 CalculateJointImpulse(const dgJointImpulseInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, const dgLeftHandSide* const matrixRow, const dgRightHandSide* const rightHandSide, dFloat32* const relVel, dFloat32* const outImpulse) const;
	//
	//dgJacobianMemory m_solverMemory;
	//dgParallelBodySolver m_parallelSolver;
	//dgBodyCluster* m_clusterData;
	//
	//dInt32 m_bodies;
	//dInt32 m_joints;
	//dInt32 m_clusters;
	//dInt32 m_markLru;
	//dInt32 m_softBodiesCount;
	//mutable dInt32 m_impulseLru;
	//mutable dInt32 m_softBodyCriticalSectionLock;
	//
	//static dgVector m_velocTol;
	//
	//friend class dgWorld;
	//friend class dgJacobianMemory;
	//friend class dgSkeletonContainer;
	//friend class dgParallelBodySolver;
	//friend class dgSolverWorlkerThreads;

	class ndBodyProxy
	{
		public:
		dFloat32 m_weight;
		dFloat32 m_invWeight;
		dSpinLock m_lock;
	};

	public:
	ndDynamicsUpdate();
	~ndDynamicsUpdate();
	void DynamicsUpdate();

	private:
	void DefaultUpdate();
	void InitWeights();
	void InitBodyArray();

	dArray<ndBodyProxy> m_bodyProxyArray;
	dInt32 m_solverPasses;

} D_GCC_NEWTON_ALIGN_32;

#endif

