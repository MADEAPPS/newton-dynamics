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

#ifndef __D_WORLD_DYNAMICS_UPDATE_SOA_H__
#define __D_WORLD_DYNAMICS_UPDATE_SOA_H__

#include "ndNewtonStdafx.h"
#include "ndDynamicsUpdate.h"

namespace ndSoa
{
	class ndSoaVector3
	{
		public:
		dVector m_x;
		dVector m_y;
		dVector m_z;
	};

	class ndSoaVector6
	{
		public:
		ndSoaVector3 m_linear;
		ndSoaVector3 m_angular;
	};

	class ndSoaJacobianPair
	{
		public:
		ndSoaVector6 m_jacobianM0;
		ndSoaVector6 m_jacobianM1;
	};

	class ndSoaMatrixElement
	{
		public:
		ndSoaJacobianPair m_Jt;
		ndSoaJacobianPair m_JMinv;

		dVector m_force;
		dVector m_diagDamp;
		dVector m_invJinvMJt;
		dVector m_coordenateAccel;
		dVector m_normalForceIndex;
		dVector m_lowerBoundFrictionCoefficent;
		dVector m_upperBoundFrictionCoefficent;
	};
};

D_MSV_NEWTON_ALIGN_32
class ndDynamicsUpdateSoa: public ndDynamicsUpdate
{
	public:
	ndDynamicsUpdateSoa(ndWorld* const world);
	virtual ~ndDynamicsUpdateSoa();

	virtual const char* GetStringId() const;

	protected:
	virtual void Update();

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

	dVector m_ordinals;
	dArray<dInt32> m_soaJointRows;
	dArray<ndSoa::ndSoaMatrixElement> m_soaMassMatrix;

} D_GCC_NEWTON_ALIGN_32;

#endif

