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

//#define	D_BODY_LRU_STEP					2	
//#define	D_MAX_SKELETON_JOINT_COUNT		256
//#define D_MAX_CONTINUE_COLLISON_STEPS	8
//#define	D_SMALL_ISLAND_COUNT			32
//#define	D_FREEZZING_VELOCITY_DRAG		dFloat32 (0.9f)
#define	D_SOLVER_MAX_ERROR					(D_FREEZE_MAG * dFloat32 (0.5f))

#if 0
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
	class ndBodyProxy
	{
		public:
		dFloat32 m_weight;
		dFloat32 m_invWeight;
	};

	class ndBodyIndexPair
	{
		public:
		ndBodyKinematic* m_body;
		ndBodyKinematic* m_root;
	};

	class ndIsland
	{
		public:
		ndIsland(ndBodyKinematic* const root)
			:m_start(0)
			,m_count(0)
			,m_root(root)
		{
		}

		dInt32 m_start;
		dInt32 m_count;
		ndBodyKinematic* m_root;
	};

	public:
	ndDynamicsUpdate();
	~ndDynamicsUpdate();
	void DynamicsUpdate();

	void Clear();

	private:
	void DefaultUpdate();
	
	void BuildIsland();
	void InitWeights();
	void InitBodyArray();
	void CalculateForces();
	void IntegrateBodies();
	void InitJacobianMatrix();
	void UpdateForceFeedback();
	void CalculateJointsForce();
	void IntegrateBodiesVelocity();
	void CalculateJointsAcceleration();

	void DetermineSleepStates();
	void UpdateIslandState(const ndIsland& island);
	void BuildJacobianMatrix(ndConstraint* const joint);
	dFloat32 CalculateJointsForce(ndConstraint* const joint);
	dInt32 GetJacobianDerivatives(dInt32 baseIndex, ndConstraint* const joint);

	static dInt32 CompareIslands(const ndIsland* const  A, const ndIsland* const B, void* const context);
	static dInt32 CompareIslandBodies(const ndBodyIndexPair* const  A, const ndBodyIndexPair* const B, void* const context);
	ndBodyKinematic* FindRootAndSplit(ndBodyKinematic* const body);

	dArray<ndIsland> m_islands;
	dArray<ndBodyKinematic*> m_bodyIslands;
	dArray<ndJacobian> m_internalForces;
	dArray<ndJacobian> m_internalForcesBack;
	dArray<ndConstraint*> m_jointArray;
	dArray<ndBodyProxy> m_bodyProxyArray;
	dArray<ndLeftHandSide> m_leftHandSide;
	dArray<ndRightHandSide> m_rightHandSide;
	dFloat32 m_accelNorm[D_MAX_THREADS_COUNT];
	dInt32 m_hasJointFeeback[D_MAX_THREADS_COUNT];

	ndWorld* m_world;
	dFloat32 m_timestep;
	dFloat32 m_invTimestep;
	dFloat32 m_firstPassCoef;
	dFloat32 m_invStepRK;
	dFloat32 m_timestepRK;
	dFloat32 m_invTimestepRK;
	dUnsigned32 m_solverPasses;
	dUnsigned32 m_maxRowsCount;
	dAtomic<dUnsigned32> m_rowsCount;

} D_GCC_NEWTON_ALIGN_32;


#endif

