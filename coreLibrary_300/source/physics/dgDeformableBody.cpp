/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"


#include "dgWorld.h"
#include "dgDeformableBody.h"
#include "dgCollisionInstance.h"
#include "dgCollisionDeformableMesh.h"

dgVector dgDeformableBody::m_dummy (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

dgDeformableBody::dgDeformableBody()
	:dgBody()
	,m_force(dgFloat32 (0.0f))
	,m_torque(dgFloat32 (0.0f))
	,m_applyExtForces(NULL)
{
	m_type = m_deformableBody;
	m_rtti |= m_deformableBodyRTTI;
//	m_isDeformable = true;
}

//dgDeformableBody::dgDeformableBody(dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionCashe, OnBodyDeserialize bodyCallback, dgDeserialize serializeCallback, void* const userData)
//	:dgBody(world, collisionCashe, bodyCallback, serializeCallback, userData)
dgDeformableBody::dgDeformableBody(dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionNode, dgDeserialize serializeCallback, void* const userData)
	:dgBody (world, collisionNode, serializeCallback, userData)
{
	m_type = m_deformableBody;
	m_rtti |= m_deformableBodyRTTI;
	dgAssert (0);
}

dgDeformableBody::~dgDeformableBody()
{
}


//void dgDeformableBody::Serialize (const dgTree<dgInt32, const dgCollision*>* const collisionCashe, OnBodySerialize bodyCallback, dgSerialize serializeCallback, void* const userData)
void dgDeformableBody::Serialize (const dgTree<dgInt32, const dgCollision*>* const collisionCashe, dgSerialize serializeCallback, void* const userData)
{
//	dgBody::Serialize (collisionCashe, bodyCallback, serializeCallback, userData);
	dgBody::Serialize (collisionCashe, serializeCallback, userData);
	dgAssert (0);
}


void dgDeformableBody::SetMassProperties (dgFloat32 mass, const dgCollisionInstance* const collision)
{
	SetMassMatrix (mass, dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f));
}

void dgDeformableBody::SetMassMatrix (dgFloat32 mass, dgFloat32 Ix, dgFloat32 Iy, dgFloat32 Iz)
{
	dgBody::SetMassMatrix (mass, dgMax(Ix, dgFloat32 (0.1f)), dgMax(Iy, dgFloat32 (0.1f)), dgMax(Iz, dgFloat32 (0.1f)));

	dgAssert (mass > dgFloat32 (0.0f));
	if (m_collision && m_collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const defomableMesh = (dgCollisionDeformableMesh*) m_collision->GetChildShape();
		defomableMesh->SetMass(mass);
	}
}

dgBody::OnApplyExtForceAndTorque dgDeformableBody::GetExtForceAndTorqueCallback () const
{
	return m_applyExtForces;
}

const dgVector& dgDeformableBody::GetForce() const 
{
	return m_force;
}

void dgDeformableBody::AddForce (const dgVector& force) 
{
	m_force += force;
}

void dgDeformableBody::SetForce (const dgVector& force)
{
	m_force = force;
}



void dgDeformableBody::SetExtForceAndTorqueCallback (OnApplyExtForceAndTorque callback)
{
	m_applyExtForces = callback;
}


void dgDeformableBody::ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex)
{
	m_force = dgVector (dgFloat32 (0.0f));
	m_torque = dgVector (dgFloat32 (0.0f));
	if (m_applyExtForces) {
		m_applyExtForces(*this, timestep, threadIndex);
	}
}


void dgDeformableBody::SetVelocity (const dgVector& velocity)
{
	dgAssert(0);
/*
	dgBody::SetVelocity(velocity);
	dgAssert (m_collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI));

	dgCollisionDeformableMesh* const deformableCollision = (dgCollisionDeformableMesh*) m_collision;
	deformableCollision->SetParticlesVelocities (velocity);
*/
}


bool dgDeformableBody::IsInEquilibrium  () const
{
	// for now soft bodies do not rest
	return false;
}


void dgDeformableBody::AttachCollision (dgCollisionInstance* const collision)
{
	if (m_collision && m_collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const defomableMesh = (dgCollisionDeformableMesh*) m_collision->GetChildShape();
		defomableMesh->m_myBody = NULL;
	}

	dgBody::AttachCollision(collision);

	if (m_collision && m_collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const defomableMesh = (dgCollisionDeformableMesh*) m_collision->GetChildShape();
		defomableMesh->m_myBody = this;
	}
}


void dgDeformableBody::SetMatrix(const dgMatrix& matrix)
{
    if (m_collision && m_collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
        dgCollisionDeformableMesh* const defomableMesh = (dgCollisionDeformableMesh*) m_collision->GetChildShape();
        defomableMesh->SetMatrix (matrix);
    }
   
    dgBody::SetMatrix(matrix);
}