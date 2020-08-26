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

#define D_MINIMUM_MASS	dFloat32(1.0e-5f)
#define D_INFINITE_MASS	dFloat32(1.0e15f)

//dgVector dDynamicBody::m_equilibriumError2 (DG_ERR_TOLERANCE2);

/*
dDynamicBodyAsymetric::dDynamicBodyAsymetric()
	:dDynamicBody()
	, m_principalAxis(dgGetIdentityMatrix())
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyAsymentricRTTI;
	dAssert(dgInt32(sizeof(dDynamicBody) & 0x0f) == 0);
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


void dDynamicBodyAsymetric::SetMassMatrix(dFloat32 mass, const dgMatrix& inertia)
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

void dDynamicBodyAsymetric::IntegrateOpenLoopExternalForce(dFloat32 timestep)
{
	dDynamicBody::IntegrateOpenLoopExternalForce(timestep);
}
*/

dDynamicBody::dDynamicBody()
	:dBody()
	,m_mass(dVector::m_zero)
	,m_invMass(dVector::m_zero)
	,m_externalForce(dVector::m_zero)
	,m_externalTorque(dVector::m_zero)
{
	SetMassMatrix(dVector::m_zero);
}

dDynamicBody::~dDynamicBody()
{
}

void dDynamicBody::SetMassMatrix(dFloat32 mass, const dMatrix& inertia)
{
	mass = dAbs(mass);

	//if (m_collision->IsType(dgCollision::dgCollisionMesh_RTTI) || m_collision->IsType(dgCollision::dgCollisionScene_RTTI)) {
	//	mass = DG_INFINITE_MASS * 2.0f;
	//}

	dShape* const shape = (dShape*)m_shapeInstance.GetShape();
	if ((mass < D_MINIMUM_MASS) || shape->GetAsShapeNull() || !shape->GetAsShapeConvex())
	{
		mass = D_INFINITE_MASS * 2.0f;
	}
	
	//if (m_collision->IsType(dgCollision::dgCollisionCompound_RTTI)) 
	//{
	//	const dgCollision* const childShape = m_collision->GetChildShape();
	//	if ((childShape->m_inertia.m_x < dFloat32(1.0e-5f)) || (childShape->m_inertia.m_y < dFloat32(1.0e-5f)) || (childShape->m_inertia.m_z < dFloat32(1.0e-5f))) 
	//	{
	//		mass = DG_INFINITE_MASS * 2.0f;
	//	}
	//}
	
	//dAssert (m_masterNode);
	//m_world->GetBroadPhase()->CheckStaticDynamic(this, mass);

	if (mass >= D_INFINITE_MASS) 
	{
		//if (m_masterNode) {
		//	if (m_invMass.m_w != dFloat32(0.0f)) {
		//		dgBodyMasterList& masterList(*m_world);
		//		if (masterList.GetFirst() != m_masterNode) {
		//			masterList.InsertAfter(masterList.GetFirst(), m_masterNode);
		//		}
		//	}
		//}
		
		m_mass.m_x = D_INFINITE_MASS;
		m_mass.m_y = D_INFINITE_MASS;
		m_mass.m_z = D_INFINITE_MASS;
		m_mass.m_w = D_INFINITE_MASS;
		m_invMass = dVector::m_zero;
	}
	else 
	{
		dFloat32 Ixx = dAbs(inertia[0][0]);
		dFloat32 Iyy = dAbs(inertia[1][1]);
		dFloat32 Izz = dAbs(inertia[2][2]);
		
		dFloat32 Ixx1 = dClamp(Ixx, dFloat32(0.001f) * mass, dFloat32(1000.0f) * mass);
		dFloat32 Iyy1 = dClamp(Iyy, dFloat32(0.001f) * mass, dFloat32(1000.0f) * mass);
		dFloat32 Izz1 = dClamp(Izz, dFloat32(0.001f) * mass, dFloat32(1000.0f) * mass);
		
		dAssert(Ixx > dFloat32(0.0f));
		dAssert(Iyy > dFloat32(0.0f));
		dAssert(Izz > dFloat32(0.0f));
		
		//if (m_masterNode) {
		//	if (m_invMass.m_w == dFloat32(0.0f)) {
		//		dgBodyMasterList& masterList(*m_world);
		//		masterList.RotateToEnd(m_masterNode);
		//	}
		//}
		
		m_mass.m_x = Ixx1;
		m_mass.m_y = Iyy1;
		m_mass.m_z = Izz1;
		m_mass.m_w = mass;
		
		m_invMass.m_x = dFloat32(1.0f) / Ixx1;
		m_invMass.m_y = dFloat32(1.0f) / Iyy1;
		m_invMass.m_z = dFloat32(1.0f) / Izz1;
		m_invMass.m_w = dFloat32(1.0f) / mass;
	}

	//#ifdef _DEBUG
#if 0
	dgBodyMasterList& me = *m_world;
	for (dgBodyMasterList::dgListNode* refNode = me.GetFirst(); refNode; refNode = refNode->GetNext()) {
		dgBody* const body0 = refNode->GetInfo().GetBody();
		dgVector invMass(body0->GetInvMass());
		if (invMass.m_w != 0.0f) {
			for (; refNode; refNode = refNode->GetNext()) {
				dgBody* const body1 = refNode->GetInfo().GetBody();
				dgVector invMass1(body1->GetInvMass());
				dAssert(invMass1.m_w != 0.0f);
			}
			break;
		}
	}
#endif
}

void dDynamicBody::ApplyExternalForces(dInt32 threadID, dFloat32 tiemstep)
{
	dVector gravity(GetNewton()->GetGravity());

	dFloat32 mass = (m_invMass.m_w > dFloat32(0.0f)) ? m_mass.m_w : dFloat32(0.0f);
	m_externalForce = gravity.Scale(mass);
	m_externalTorque = dVector::m_zero;
}
