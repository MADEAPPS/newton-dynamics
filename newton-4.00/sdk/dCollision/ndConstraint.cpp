/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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


#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndConstraint.h"
#include "ndBodyKinematic.h"

void ndForceImpactPair::Clear()
{
	m_force = ndFloat32(ndFloat32(0.0f));
	m_impact = ndFloat32(ndFloat32(0.0f));
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); ++i)
	{
		m_initialGuess[i] = ndFloat32(ndFloat32(0.0f));
	}
}

void ndForceImpactPair::Push(ndFloat32 val)
{
	for (ndInt32 i = 1; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); ++i)
	{
		m_initialGuess[i - 1] = m_initialGuess[i];
	}
	m_initialGuess[sizeof(m_initialGuess) / sizeof(m_initialGuess[0]) - 1] = val;
}

ndFloat32 ndForceImpactPair::GetInitialGuess() const
{
	//return 100.0f;
	ndFloat32 smallest = ndFloat32(1.0e15f);
	ndFloat32 value = ndFloat32(ndFloat32(0.0f));
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_initialGuess) / sizeof(m_initialGuess[0])); ++i)
	{
		ndFloat32 mag = ndAbs(m_initialGuess[i]);
		if (mag < smallest)
		{
			smallest = mag;
			value = m_initialGuess[i];
		}
	}
	return value;
}

ndConstraint::ndConstraint()
	:ndContainersFreeListAlloc<ndConstraint>()
	,m_forceBody0(ndVector::m_zero)
	,m_torqueBody0(ndVector::m_zero)
	,m_forceBody1(ndVector::m_zero)
	,m_torqueBody1(ndVector::m_zero)
	,m_body0(nullptr)
	,m_body1(nullptr)
	,m_rowCount(0)
	,m_rowStart(0)
	,m_maxDof(0)
	,m_active(1)
	,m_fence0(0)
	,m_fence1(0)
	,m_resting(0)
	,m_isInSkeletonLoop(0)
{
}

void ndConstraint::InitPointParam(ndPointParam& param, const ndVector& p0Global, const ndVector& p1Global) const
{
	ndBodyKinematic* const body0 = GetBody0();
	ndBodyKinematic* const body1 = GetBody1();
	ndAssert(body0);
	ndAssert(body1);

	param.m_posit0 = p0Global;
	param.m_posit1 = p1Global;

	param.m_r0 = (p0Global - body0->m_globalCentreOfMass) & ndVector::m_triplexMask;
	param.m_r1 = (p1Global - body1->m_globalCentreOfMass) & ndVector::m_triplexMask;
}

#ifdef _DEBUG
void ndRightHandSide::SetSanityCheck(const ndConstraint* const joint)
{
	m_debugCheck = m_bilateral;
	if (!joint->IsBilateral())
	{
		if (m_normalForceIndex == D_INDEPENDENT_ROW)
		{
			m_debugCheck = m_contact;
		}
		else if (m_normalForceIndex != D_OVERRIDE_FRICTION_ROW)
		{
			m_debugCheck = m_friction;
		}
	}
}
#else
void ndRightHandSide::SetSanityCheck(const ndConstraint* const)
{
}
#endif

ndConstraint::~ndConstraint()
{
}

ndContact* ndConstraint::GetAsContact()
{
	return nullptr;
}

ndJointBilateralConstraint* ndConstraint::GetAsBilateral()
{
	return nullptr;
}

bool ndConstraint::IsActive() const
{
	return m_active ? true : false;
}

void ndConstraint::SetActive(bool state)
{
	m_active = ndUnsigned8(state ? 1 : 0);
}

bool ndConstraint::IsBilateral() const
{
	return false;
}

ndUnsigned32 ndConstraint::GetRowsCount() const
{
	return m_maxDof;
}

ndBodyKinematic* ndConstraint::GetBody0() const
{
	return m_body0;
}

ndBodyKinematic* ndConstraint::GetBody1() const
{
	return m_body1;
}

void ndConstraint::DebugJoint(ndConstraintDebugCallback&) const
{
}

ndVector ndConstraint::GetForceBody0() const
{
	return m_forceBody0;
}

ndVector ndConstraint::GetTorqueBody0() const
{
	return m_torqueBody0;
}

ndVector ndConstraint::GetForceBody1() const
{
	return m_forceBody1;
}

ndVector ndConstraint::GetTorqueBody1() const
{
	return m_torqueBody1;
}

void ndConstraint::UpdateParameters()
{
}

bool ndConstraint::CheckBlockMatrixPSD(const ndLeftHandSide* const bigMatrix, const ndRightHandSide* const bigRhs) const
{
	ndAssert(m_rowCount <= D_CONSTRAINT_MAX_ROWS);
	ndFloat32 matrix[D_CONSTRAINT_MAX_ROWS][D_CONSTRAINT_MAX_ROWS];

	const ndInt32 index = m_rowStart;
	const ndInt32 count = m_rowCount;
	const ndVector zero(ndVector::m_zero);

	for (ndInt32 i = 0; i < count; ++i)
	{
		const ndLeftHandSide* const row_i = &bigMatrix[index + i];
		const ndRightHandSide* const rhs = &bigRhs[index + i];

		const ndJacobian& JMinvM0 = row_i->m_JMinv.m_jacobianM0;
		const ndJacobian& JMinvM1 = row_i->m_JMinv.m_jacobianM1;
		const ndJacobian& JtM0 = row_i->m_Jt.m_jacobianM0;
		const ndJacobian& JtM1 = row_i->m_Jt.m_jacobianM1;
		const ndVector tmpDiag(
			JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular +
			JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular);
		
		ndFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
		diag *= (ndFloat32(1.0f) + rhs->m_diagonalRegularizer);
		matrix[i][i] = diag;

		for (ndInt32 j = i + 1; j < m_rowCount; ++j)
		{
			const ndLeftHandSide* const row_j = &bigMatrix[index + j];
			const ndJacobian& JtM0_j = row_j->m_Jt.m_jacobianM0;
			const ndJacobian& JtM1_j = row_j->m_Jt.m_jacobianM1;

			const ndVector offDiag(
				JMinvM0.m_linear * JtM0_j.m_linear + JMinvM0.m_angular * JtM0_j.m_angular +
				JMinvM1.m_linear * JtM1_j.m_linear + JMinvM1.m_angular * JtM1_j.m_angular);

			ndFloat32 off = offDiag.AddHorizontal().GetScalar();
			matrix[i][j] = off;
			matrix[j][i] = off;
		}
	}

	bool test = ndTestPSDmatrix(count, D_CONSTRAINT_MAX_ROWS, &matrix[0][0]);
	return test;
}