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

#ifndef __ND_SKELETON_IMMEDIATE_SOLVER_H__
#define __ND_SKELETON_IMMEDIATE_SOLVER_H__

#include "ndNewtonStdafx.h"

class ndWorld;
class ndConstraint;
class ndSkeletonContainer;

class ndSkelIkSolver: public ndClassAlloc
{
	public:	
	ndSkelIkSolver()
		:ndClassAlloc()
		,m_bodies(32)
		,m_internalForces(32)
		,m_leftHandSide(64)
		,m_rightHandSide(64)
		,m_timestep(ndFloat32 (0.0f))
		,m_invTimestep(ndFloat32(0.0f))
		,m_world(nullptr)
		,m_skeleton(nullptr)
	{
	}

	D_NEWTON_API bool IsSleeping(ndSkeletonContainer* const skeleton) const;
	D_NEWTON_API void AddCloseLoopJoint(ndSkeletonContainer* const skeleton, ndConstraint* const joint);

	D_NEWTON_API void BeginSolve(ndSkeletonContainer* const skeleton, ndWorld* const world, ndFloat32 timestep);
	D_NEWTON_API void UpdateAccel();
	D_NEWTON_API void Solve();
	D_NEWTON_API void EndSolve();

	D_NEWTON_API ndVector GetBodyForce(const ndBodyKinematic* const body) const;
	D_NEWTON_API ndVector GetBodyTorque(const ndBodyKinematic* const body) const;

	private:
	void GetJacobianDerivatives(ndConstraint* const joint);
	void BuildJacobianMatrix(ndConstraint* const joint);

	ndArray<ndBodyKinematic*> m_bodies;
	ndArray<ndJacobian> m_internalForces;
	ndArray<ndLeftHandSide> m_leftHandSide;
	ndArray<ndRightHandSide> m_rightHandSide;
	ndFloat32 m_timestep;
	ndFloat32 m_invTimestep;

	private:
	ndWorld* m_world;
	ndSkeletonContainer* m_skeleton;
	friend class ndSkeletonContainer;
};


#endif


