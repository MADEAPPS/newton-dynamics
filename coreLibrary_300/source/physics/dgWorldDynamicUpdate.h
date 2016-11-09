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
#define	DG_SMALL_ISLAND_COUNT			2
#define	DG_MAX_SKELETON_JOINT_COUNT		256

#define	DG_FREEZZING_VELOCITY_DRAG		dgFloat32 (0.9f)
#define	DG_SOLVER_MAX_ERROR				(DG_FREEZE_MAG * dgFloat32 (0.5f))


// the solver is a RK order, but instead of weighting the intermediate derivative by the usual 1/6, 1/3, 1/3, 1/6 coefficients
// I am using 1/4, 1/4, 1/4, 1/4.
// This is absolutely correct. the weighting coefficients of any RK method comes for fixing arbitrary criteria
// solving a set of linear equation on the coefficients. 
// the standard coefficients just happen to lead to a accurate result because they are optimal with reset to a second order solution. 
// However a different set of coefficient generates results that are not much different than the optimal set, but it allows 
// for simpler calculation of the intermediate derivatives and also for less intermediate memory.
// for more detail on the derivation of the Runge Kutta coefficients you can go to:  
// http://pathfinder.scar.utoronto.ca/~dyer/csca57/book_P/node51.html


#define DG_MAX_CONTINUE_COLLISON_STEPS	8

class dgBody;
class dgDynamicBody;
class dgParallelSolverSyncData;
class dgWorldDynamicUpdateSyncDescriptor;


class dgIslandCallbackStruct
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

class dgIsland
{
	public:
	dgInt32 m_bodyStart;
	dgInt32 m_bodyCount;
	dgInt32 m_jointStart;	
	dgInt32 m_jointCount;
	dgInt32 m_rowsStart;
	dgInt32 m_rowsCount;
	dgInt32 m_blockMatrixStart;
	dgInt32 m_blockMatrixSize;
	dgInt32 m_islandLRU;
	dgInt32 m_isContinueCollision;
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
	dgInt32 m_blockMatrixSize;
	dgInt32 m_blockMatrixStart;
};


class dgParallelSolverSyncData
{
	public:
	class dgParallelJointMap
	{
		public:
		dgInt32 m_color;
		dgInt32 m_bashCount;
		dgInt32 m_jointIndex;
	};

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
	dgInt32 m_lock0;
	dgInt32 m_lock1;
	dgInt32 m_jacobianMatrixRowAtomicIndex;

	dgInt32* m_bodyLocks;  
	const dgIsland* m_island;
	dgParallelJointMap* m_jointConflicts;
	dgInt32 m_jointBatches[32];
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
	dgJacobianPair m_JMinv____;

	dgFloat32 m_force;
	dgFloat32 m_deltaAccel;
	dgFloat32 m_diagDamp;
	dgFloat32 m_invJMinvJt;

	dgFloat32 m_restitution;
	dgFloat32 m_penetration;
	dgFloat32 m_coordenateAccel;
	dgFloat32 m_penetrationStiffness;

	dgFloat32 m_lowerBoundFrictionCoefficent;
	dgFloat32 m_upperBoundFrictionCoefficent;
	dgFloat32 m_stiffness;
	dgFloat32 m_maxImpact;

	dgForceImpactPair* m_jointFeebackForce;
	dgInt32 m_normalForceIndex;
} DG_GCC_VECTOR_ALIGMENT;

class dgJacobianMemory
{
	public:
	void Init (dgWorld* const world, dgInt32 rowsCount, dgInt32 bodyCount, dgInt32 blockMatrixSizeInBytes);

	dgJacobian* m_internalForcesBuffer;
	dgFloat32* m_solverBlockJacobianMemory;
	dgJacobianMatrixElement* m_jacobianBuffer;
};

class dgWorldDynamicUpdate
{
	class dgJointMapTable
	{
		public:
		const dgJointInfo* m_info;
		dgInt32 index;
	};

	public:
	dgWorldDynamicUpdate();
	void UpdateDynamics (dgFloat32 timestep);
	dgBody* GetIslandBody (const void* const island, dgInt32 index) const;

	private:
	void BuildIslands(dgFloat32 timestep);
	//void BuildIsland (dgQueue<dgDynamicBody*>& queue, dgFloat32 timestep, dgInt32 jountCount);
	void SortIsland(dgIsland* const island, dgFloat32 timestep, dgInt32 threadID) const;
	void SpanningTree (dgDynamicBody* const body, dgDynamicBody** const queueBuffer, dgFloat32 timestep);
	
	static dgInt32 CompareIslands (const dgIsland* const islandA, const dgIsland* const islandB, void* notUsed);
	static void CalculateIslandReactionForcesKernel (void* const context, void* const worldContext, dgInt32 threadID);

	static void IntegrateInslandParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void InitializeBodyArrayParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void BuildJacobianMatrixParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void SolverInitInternalForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void CalculateJointsForceParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void CalculateJointsVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void KinematicCallbackUpdateParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	//static void CalculateJointsImpulseVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void UpdateFeedbackForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void UpdateBodyVelocityParallelKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static void FindActiveJointAndBodiesKernel (void* const context, void* const worldContext, dgInt32 threadID); 
	static dgInt32 SortJointInfoByColor(const dgParallelSolverSyncData::dgParallelJointMap* const indirectIndexA, const dgParallelSolverSyncData::dgParallelJointMap* const indirectIndexB, void* const context);
	//void CreateParallelArrayBatchArrays (dgParallelSolverSyncData* const solverSyncData, dgJointInfo* const constraintArray, const dgIsland* const island) const;

	void FindActiveJointAndBodies (dgIsland* const island); 
	void IntegrateIslandParallel(dgParallelSolverSyncData* const syncData) const; 
	void InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData) const; 
	void BuildJacobianMatrixParallel (dgParallelSolverSyncData* const syncData) const; 
	void SolverInitInternalForcesParallel (dgParallelSolverSyncData* const syncData) const; 
	void CalculateForcesGameModeParallel (dgParallelSolverSyncData* const syncData) const; 

	void CalculateReactionForcesParallel (const dgIsland* const island, dgFloat32 timestep) const;
	void LinearizeJointParallelArray(dgParallelSolverSyncData* const solverSyncData, dgJointInfo* const constraintArray, const dgIsland* const island) const;
	void ApplyNetTorqueAndForce (dgDynamicBody* const body, const dgVector& invTimeStep, const dgVector& accNorm, const dgVector& mask) const;
	void ApplyNetVelcAndOmega (dgDynamicBody* const body, const dgJacobian& forceAndTorque, const dgVector& timestep4, const dgVector& speedFreeze2, const dgVector& mask) const;
	
	void CalculateIslandReactionForces (dgIsland* const island, dgFloat32 timestep, dgInt32 threadID) const;
	void BuildJacobianMatrix (dgIsland* const island, dgInt32 threadID, dgFloat32 timestep) const;
	void BuildJacobianMatrix (const dgBodyInfo* const bodyInfo, const dgJointInfo* const jointInfo, dgJacobianMatrixElement* const matrixRow, dgFloat32 forceImpulseScale) const;
	void InitJointForce (dgJointInfo* const jointInfo, dgJacobianMatrixElement* const matrixRow, dgJacobian& force0, dgJacobian& force1) const;
	void CalculateForcesGameMode (const dgIsland* const island, dgInt32 threadID, dgFloat32 timestep, dgFloat32 maxAccNorm) const;
	void CalculateReactionsForces(const dgIsland* const island, dgInt32 threadID, dgFloat32 timestep, dgFloat32 maxAccNorm) const;
	void ApplyExternalForcesAndAcceleration(const dgIsland* const island, dgInt32 threadID, dgFloat32 timestep, dgFloat32 maxAccNorm) const;

	dgFloat32 CalculateJointForce(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const;
	void IntegrateArray (const dgIsland* const island, dgFloat32 accelTolerance, dgFloat32 timestep, dgInt32 threadID) const;

	void CalculateIslandContacts (dgIsland* const island, dgFloat32 timestep, dgInt32 currLru, dgInt32 threadID) const;
	void GetJacobianDerivatives (const dgIsland* const island, dgInt32 threadID, dgInt32 rowCount, dgFloat32 timestep) const;	
	dgInt32 GetJacobianDerivatives (dgContraintDescritor& constraintParamOut, dgJointInfo* const jointInfo, dgConstraint* const constraint, dgJacobianMatrixElement* const matrixRow, dgInt32 rowCount) const;

	
	dgInt32 m_bodies;
	dgInt32 m_joints;
	dgInt32 m_islands;
	dgUnsigned32 m_markLru;
	dgJacobianMemory m_solverMemory;
	dgThread::dgCriticalSection m_softBodyCriticalSectionLock;
	dgIsland* m_islandMemory;
	dgFloat32 m_solverConvergeQuality;
	static dgVector m_velocTol;
	static dgVector m_eulerTaylorCorrection;
	static dgMatrix m_vectorMasks;

	friend class dgWorld;
	friend class dgAmpInstance;
	friend class dgJacobianMemory;
	friend class dgSkeletonContainer;
	friend class dgSolverWorlkerThreads;
};

#endif

