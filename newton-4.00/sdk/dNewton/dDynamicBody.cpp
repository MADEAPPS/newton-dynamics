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

#include "dNewtonStdafx.h"
#include "dNewton.h"
#include "dDynamicBody.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//dgVector dDynamicBody::m_equilibriumError2 (DG_ERR_TOLERANCE2);

/*
dDynamicBodyAsymetric::dDynamicBodyAsymetric()
	:dDynamicBody()
	, m_principalAxis(dgGetIdentityMatrix())
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyAsymentricRTTI;
	dgAssert(dgInt32(sizeof(dDynamicBody) & 0x0f) == 0);
}

dDynamicBodyAsymetric::dDynamicBodyAsymetric(dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionNode, dgDeserialize serializeCallback, void* const userData, dgInt32 revisionNumber)
	:dDynamicBody(world, collisionNode, serializeCallback, userData, revisionNumber)
	, m_principalAxis(dgGetIdentityMatrix())
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;
	serializeCallback(userData, &m_principalAxis, sizeof(m_principalAxis));
}

void dDynamicBodyAsymetric::Serialize(const dgTree<dgInt32, const dgCollision*>& collisionRemapId, dgSerialize serializeCallback, void* const userData)
{
	dDynamicBody::Serialize(collisionRemapId, serializeCallback, userData);
	serializeCallback(userData, &m_principalAxis, sizeof(m_principalAxis));
}


void dDynamicBodyAsymetric::SetMassMatrix(dgFloat32 mass, const dgMatrix& inertia)
{
	//dgVector II;
	m_principalAxis = inertia;
	dgVector II (m_principalAxis.EigenVectors());
	dgMatrix massMatrix(dgGetIdentityMatrix());
	massMatrix[0][0] = II[0];
	massMatrix[1][1] = II[1];
	massMatrix[2][2] = II[2];
	dBody::SetMassMatrix(mass, massMatrix);
}

dgMatrix dDynamicBodyAsymetric::CalculateLocalInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_mass[0];
	diagonal[1][1] = m_mass[1];
	diagonal[2][2] = m_mass[2];
	return matrix * diagonal * matrix.Inverse();
}

dgMatrix dDynamicBodyAsymetric::CalculateInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis * m_matrix);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_mass[0];
	diagonal[1][1] = m_mass[1];
	diagonal[2][2] = m_mass[2];
	return matrix * diagonal * matrix.Inverse();
}

dgMatrix dDynamicBodyAsymetric::CalculateInvInertiaMatrix() const
{
	dgMatrix matrix(m_principalAxis * m_matrix);
	matrix.m_posit = dgVector::m_wOne;
	dgMatrix diagonal(dgGetIdentityMatrix());
	diagonal[0][0] = m_invMass[0];
	diagonal[1][1] = m_invMass[1];
	diagonal[2][2] = m_invMass[2];
	return matrix * diagonal * matrix.Inverse();
}

void dDynamicBodyAsymetric::IntegrateOpenLoopExternalForce(dgFloat32 timestep)
{
	dDynamicBody::IntegrateOpenLoopExternalForce(timestep);
}
*/

dDynamicBody::dDynamicBody()
	:dBody()
{
}

dDynamicBody::~dDynamicBody()
{
}

void dDynamicBody::ApplyExternalForces(dInt32 threadID, dFloat32 tiemstep)
{
	dTriplex xxx(GetNewton()->GetGravity());
}
