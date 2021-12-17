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

#ifndef __ND_WORLD_DYNAMICS_UPDATE_H__
#define __ND_WORLD_DYNAMICS_UPDATE_H__

#include "ndNewtonStdafx.h"

#define D_SMALL_ISLAND_COUNT		32
#define	D_FREEZZING_VELOCITY_DRAG	ndFloat32 (0.9f)
#define	D_SOLVER_MAX_ERROR			(D_FREEZE_MAG * ndFloat32 (0.5f))

#define D_MAX_BODY_RADIX_BIT		9

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

class ndWorld;

D_MSV_NEWTON_ALIGN_32
class ndDynamicsUpdate : public ndClassAlloc
{
	public:
	class ndJointBodyPairIndex
	{
		public:
		ndInt32 m_body;
		ndInt32 m_joint;
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
			, m_count(0)
			, m_root(root)
		{
		}

		ndInt32 m_start;
		ndInt32 m_count;
		ndBodyKinematic* m_root;
	};

	public:
	ndDynamicsUpdate(ndWorld* const world);
	virtual ~ndDynamicsUpdate();

	void* GetTempBuffer() const;
	virtual const char* GetStringId() const;
	ndInt32 GetUnconstrainedBodyCount() const;
	void ClearBuffer(void* const buffer, ndInt32 sizeInByte) const;
	void ClearJacobianBuffer(ndInt32 count, ndJacobian* const dst) const;

	ndVector GetVelocTol() const;
	ndFloat32 GetTimestepRK() const;
	ndArray<ndIsland>& GetIslands();
	ndArray<ndJacobian>& GetInternalForces();
	ndArray<ndLeftHandSide>& GetLeftHandSide();
	ndArray<ndInt32>& GetJointForceIndexBuffer();
	ndArray<ndRightHandSide>& GetRightHandSide();
	ndArray<ndJacobian>& GetTempInternalForces();
	ndArray<ndBodyKinematic*>& GetBodyIslandOrder();
	ndArray<ndJointBodyPairIndex>& GetJointBodyPairIndexBuffer();

	private:
	void SortJoints();
	void SortIslands();
	void BuildIsland();
	void InitWeights();
	void InitBodyArray();
	void InitSkeletons();
	void CalculateForces();
	void IntegrateBodies();
	void UpdateSkeletons();
	void InitJacobianMatrix();
	void UpdateForceFeedback();
	void CalculateJointsForce();
	void IntegrateBodiesVelocity();
	void CalculateJointsAcceleration();
	void IntegrateUnconstrainedBodies();

	void DetermineSleepStates();
	void GetJacobianDerivatives(ndConstraint* const joint);

	protected:
	void Clear();
	virtual void Update();
	void SortJointsScan();
	void SortBodyJointScan();
	ndBodyKinematic* FindRootAndSplit(ndBodyKinematic* const body);

	ndVector m_velocTol;
	ndArray<ndIsland> m_islands;
	ndArray<ndInt32> m_jointForcesIndex;
	ndArray<ndJacobian> m_internalForces;
	ndArray<ndLeftHandSide> m_leftHandSide;
	ndArray<ndRightHandSide> m_rightHandSide;
	ndArray<ndJacobian> m_tempInternalForces;
	ndArray<ndBodyKinematic*> m_bodyIslandOrder;
	ndArray<ndJointBodyPairIndex> m_jointBodyPairIndexBuffer;

	ndWorld* m_world;
	ndFloat32 m_timestep;
	ndFloat32 m_invTimestep;
	ndFloat32 m_firstPassCoef;
	ndFloat32 m_invStepRK;
	ndFloat32 m_timestepRK;
	ndFloat32 m_invTimestepRK;
	ndUnsigned32 m_solverPasses;
	ndInt32 m_activeJointCount;
	ndInt32 m_unConstrainedBodyCount;

	friend class ndWorld;
} D_GCC_NEWTON_ALIGN_32;

inline ndVector ndDynamicsUpdate::GetVelocTol() const
{
	return m_velocTol;
}

inline ndFloat32 ndDynamicsUpdate::GetTimestepRK() const
{
	return m_timestepRK;
}

inline ndArray<ndDynamicsUpdate::ndIsland>& ndDynamicsUpdate::GetIslands()
{
	return m_islands;
}

inline void* ndDynamicsUpdate::GetTempBuffer() const
{
	return (void*)&m_leftHandSide[0];
}

inline ndArray<ndJacobian>& ndDynamicsUpdate::GetInternalForces()
{
	return m_internalForces;
}

inline ndArray<ndJacobian>& ndDynamicsUpdate::GetTempInternalForces()
{
	return m_tempInternalForces;
}

inline  ndArray<ndLeftHandSide>& ndDynamicsUpdate::GetLeftHandSide()
{
	return m_leftHandSide;
}

inline  ndArray<ndRightHandSide>& ndDynamicsUpdate::GetRightHandSide()
{
	return m_rightHandSide;
}

inline  ndArray<ndBodyKinematic*>& ndDynamicsUpdate::GetBodyIslandOrder()
{
	return m_bodyIslandOrder;
}

inline ndInt32 ndDynamicsUpdate::GetUnconstrainedBodyCount() const
{
	return m_unConstrainedBodyCount;
}

inline ndArray<ndDynamicsUpdate::ndJointBodyPairIndex>& ndDynamicsUpdate::GetJointBodyPairIndexBuffer()
{
	return m_jointBodyPairIndexBuffer;
}

inline ndArray<ndInt32>& ndDynamicsUpdate::GetJointForceIndexBuffer()
{
	return m_jointForcesIndex;
}

inline void ndDynamicsUpdate::ClearJacobianBuffer(ndInt32 count, ndJacobian* const buffer) const
{
	const ndVector zero(ndVector::m_zero);
	ndVector* const dst = &buffer[0].m_linear;
	for (ndInt32 i = 0; i < count; i++)
	{
		dst[i * 2 + 0] = zero;
		dst[i * 2 + 1] = zero;
	}
}

inline void ndDynamicsUpdate::ClearBuffer(void* const buffer, ndInt32 sizeInByte) const
{
	ndInt32 sizeInJacobian = sizeInByte / sizeof(ndJacobian);
	ClearJacobianBuffer(sizeInJacobian, (ndJacobian*)buffer);
	char* const ptr = (char*)buffer;
	for (ndInt32 i = sizeInJacobian * sizeof(ndJacobian); i < sizeInByte; i++)
	{
		ptr[i] = 0;
	}
}

inline ndBodyKinematic* ndDynamicsUpdate::FindRootAndSplit(ndBodyKinematic* const body)
{
	ndBodyKinematic* node = body;
	while (node->m_islandParent != node)
	{
		ndBodyKinematic* const prev = node;
		node = node->m_islandParent;
		prev->m_islandParent = node->m_islandParent;
	}
	return node;
}

#endif

