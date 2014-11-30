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
#include "dgBody.h"
#include "dgWorld.h"
#include "dgContact.h"
#include "dgCollisionInstance.h"
#include "dgWorldDynamicUpdate.h"

#define REST_RELATIVE_VELOCITY			dgFloat32 (1.0e-3f)
#define MAX_DYNAMIC_FRICTION_SPEED		dgFloat32 (0.3f)
#define MAX_PENETRATION_STIFFNESS		dgFloat32 (100.0f)



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dgContactMaterial::dgContactMaterial()
	:m_dir0 (dgFloat32 (0.0f))
	,m_dir1 (dgFloat32 (0.0f))
	,m_userData(NULL)
	,m_aabbOverlap(NULL)
	,m_contactPoint(NULL)
	,m_compoundAABBOverlap(NULL)
{
	//	dgAssert ( dgInt32 (sizeof (dgContactMaterial) & 15) == 0);
	dgAssert ((((dgUnsigned64) this) & 15) == 0);
	m_point = dgVector (dgFloat32 (0.0f));
	m_softness = dgFloat32 (0.1f);
	m_restitution = dgFloat32 (0.4f);

	m_staticFriction0 = dgFloat32 (0.9f);
	m_staticFriction1 = dgFloat32 (0.9f);
	m_dynamicFriction0 = dgFloat32 (0.5f);
	m_dynamicFriction1 = dgFloat32 (0.5f);
	m_dir0_Force.m_force = dgFloat32 (0.0f);
	m_dir0_Force.m_impact = dgFloat32 (0.0f);
	m_dir1_Force.m_force = dgFloat32 (0.0f);
	m_dir1_Force.m_impact = dgFloat32 (0.0f);
	m_normal_Force.m_force = dgFloat32 (0.0f);
	m_normal_Force.m_impact = dgFloat32 (0.0f);
	m_skinThickness = dgFloat32 (0.0f);
	//m_skinThickness = dgFloat32 (0.1f);
	//m_skinThickness = DG_MAX_COLLISION_AABB_PADDING * dgFloat32 (0.125f);
	m_flags = m_collisionEnable | m_friction0Enable | m_friction1Enable;
}



dgContact::dgContact(dgWorld* const world, const dgContactMaterial* const material)
	:dgConstraint(), dgList<dgContactMaterial>(world->GetAllocator())
	,m_closestDistance (dgFloat32 (0.0f))
	,m_timeOfImpact(dgFloat32 (0.0f))
	,m_world(world)
	,m_material(material)
	,m_contactNode(NULL)
	,m_broadphaseLru(0)
	,m_isNewContact(true)
{
	dgAssert ((((dgUnsigned64) this) & 15) == 0);
	m_maxDOF = 0;
	m_enableCollision = true;
	m_constId = m_contactConstraint;
}

dgContact::~dgContact()
{
	dgList<dgContactMaterial>::RemoveAll();

	if (m_contactNode) {
		dgActiveContacts* const activeContacts = m_world;
		activeContacts->Remove (m_contactNode);
	}
}

void dgContact::AppendToActiveList()
{
	dgAssert (!m_contactNode);
	dgActiveContacts* const activeContacts = m_world;
	m_contactNode = activeContacts->Append (this);
}

void dgContact::SwapBodies()
{
	dgSwap (m_body0, m_body1);
	dgSwap (m_link0, m_link1);
}


void dgContact::GetInfo (dgConstraintInfo* const info) const
{
	memset (info, 0, sizeof (dgConstraintInfo));
	InitInfo (info);
	info->m_collideCollisionOn = GetCount();
	strcpy (info->m_discriptionType, "contact");
}

void dgContact::CalculatePointDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& dir, const dgPointParam& param) const
{
	dgAssert (m_body0);
	dgAssert (m_body1);

	dgVector r0CrossDir (param.m_r0 * dir);
	dgJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
	jacobian0.m_linear[0] = dir.m_x;
	jacobian0.m_linear[1] = dir.m_y;
	jacobian0.m_linear[2] = dir.m_z;
	jacobian0.m_linear[3] = dgFloat32 (0.0f);
	jacobian0.m_angular[0] = r0CrossDir.m_x;
	jacobian0.m_angular[1] = r0CrossDir.m_y;
	jacobian0.m_angular[2] = r0CrossDir.m_z;
	jacobian0.m_angular[3] = dgFloat32 (0.0f);


	dgVector r1CrossDir (dir * param.m_r1);
	dgJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 
	jacobian1.m_linear[0] = -dir.m_x;
	jacobian1.m_linear[1] = -dir.m_y;
	jacobian1.m_linear[2] = -dir.m_z;
	jacobian1.m_linear[3] =  dgFloat32 (0.0f);
	jacobian1.m_angular[0] = r1CrossDir.m_x;
	jacobian1.m_angular[1] = r1CrossDir.m_y;
	jacobian1.m_angular[2] = r1CrossDir.m_z;
	jacobian1.m_angular[3] = dgFloat32 (0.0f);
}

dgUnsigned32 dgContact::JacobianDerivative (dgContraintDescritor& params)
{
	dgInt32 frictionIndex = 0;
	if (m_maxDOF) {
		dgInt32 i = 0;
		frictionIndex = GetCount();
		for (dgList<dgContactMaterial>::dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			const dgContactMaterial& contact = node->GetInfo(); 
			JacobianContactDerivative (params, contact, i, frictionIndex);
			i ++;
		}
	}

	return dgUnsigned32 (frictionIndex);
}



void dgContact::JacobianContactDerivative (dgContraintDescritor& params, const dgContactMaterial& contact, dgInt32 normalIndex, dgInt32& frictionIndex) 
{
	dgPointParam pointData;

	dgFloat32 impulseOrForceScale = (params.m_timestep > dgFloat32 (0.0f)) ? params.m_invTimestep : dgFloat32 (1.0f);

	InitPointParam (pointData, dgFloat32 (1.0f), contact.m_point, contact.m_point);
	CalculatePointDerivative (normalIndex, params, contact.m_normal, pointData); 

	dgVector velocError (pointData.m_veloc1 - pointData.m_veloc0);
	dgFloat32 restitution = contact.m_restitution;

	dgFloat32 relVelocErr = velocError % contact.m_normal;
	dgFloat32 penetration = dgClamp (contact.m_penetration - DG_RESTING_CONTACT_PENETRATION, dgFloat32(0.0f), dgFloat32(0.5f));

//static int xxx;
//xxx++;
//dgTrace (("%d %f\n", xxx, penetration));
//if (xxx > 685)
//xxx *= 1;

	dgFloat32 penetrationStiffness = MAX_PENETRATION_STIFFNESS * contact.m_softness;
	dgFloat32 penetrationVeloc = penetration * penetrationStiffness;
	dgAssert (dgAbsf (penetrationVeloc - MAX_PENETRATION_STIFFNESS * contact.m_softness * penetration) < dgFloat32 (1.0e-6f));
	if (relVelocErr > REST_RELATIVE_VELOCITY) {
		relVelocErr *= (restitution + dgFloat32 (1.0f));
	}

	params.m_restitution[normalIndex] = restitution;
	params.m_penetration[normalIndex] = penetration;
	params.m_penetrationStiffness[normalIndex] = penetrationStiffness;
	params.m_forceBounds[normalIndex].m_low = dgFloat32 (0.0f);
	params.m_forceBounds[normalIndex].m_normalIndex = DG_NORMAL_CONSTRAINT;
	params.m_forceBounds[normalIndex].m_jointForce = (dgForceImpactPair*) &contact.m_normal_Force;
	params.m_jointStiffness[normalIndex] = dgFloat32 (1.0f);
	params.m_isMotor[normalIndex] = 0;

//	params.m_jointAccel[normalIndex] = GetMax (dgFloat32 (-4.0f), relVelocErr + penetrationVeloc) * params.m_invTimestep;
	params.m_jointAccel[normalIndex] = dgMax (dgFloat32 (-4.0f), relVelocErr + penetrationVeloc) * impulseOrForceScale;
	if (contact.m_flags & dgContactMaterial::m_overrideNormalAccel) {
		params.m_jointAccel[normalIndex] += contact.m_normal_Force.m_force;
	}

	// first dir friction force
	if (contact.m_flags & dgContactMaterial::m_friction0Enable) {
		dgInt32 jacobIndex = frictionIndex;
		frictionIndex += 1;
		CalculatePointDerivative (jacobIndex, params, contact.m_dir0, pointData); 
		relVelocErr = velocError % contact.m_dir0;
		params.m_forceBounds[jacobIndex].m_normalIndex = normalIndex;
		params.m_jointStiffness[jacobIndex] = dgFloat32 (1.0f);

		params.m_restitution[jacobIndex] = dgFloat32 (0.0f);
		params.m_penetration[jacobIndex] = dgFloat32 (0.0f);
		params.m_penetrationStiffness[jacobIndex] = dgFloat32 (0.0f);
//		if (contact.m_override0Accel) {
		if (contact.m_flags & dgContactMaterial::m_override0Accel) {
			params.m_jointAccel[jacobIndex] = contact.m_dir0_Force.m_force;
			params.m_isMotor[jacobIndex] = 1;
		} else {
			//params.m_jointAccel[jacobIndex] = relVelocErr * params.m_invTimestep;
			params.m_jointAccel[jacobIndex] = relVelocErr * impulseOrForceScale;
			params.m_isMotor[jacobIndex] = 0;
		}
		if (dgAbsf (relVelocErr) > MAX_DYNAMIC_FRICTION_SPEED) {
			params.m_forceBounds[jacobIndex].m_low = -contact.m_dynamicFriction0;
			params.m_forceBounds[jacobIndex].m_upper = contact.m_dynamicFriction0;
		} else {
			params.m_forceBounds[jacobIndex].m_low = -contact.m_staticFriction0;
			params.m_forceBounds[jacobIndex].m_upper = contact.m_staticFriction0;
		}
		params.m_forceBounds[jacobIndex].m_jointForce = (dgForceImpactPair*)&contact.m_dir0_Force;
	}

//	if (contact.m_friction1Enable) {
	if (contact.m_flags & dgContactMaterial::m_friction1Enable) {
		dgInt32 jacobIndex = frictionIndex;
		frictionIndex += 1;
		CalculatePointDerivative (jacobIndex, params, contact.m_dir1, pointData); 
		relVelocErr = velocError % contact.m_dir1;
		params.m_forceBounds[jacobIndex].m_normalIndex = normalIndex;
		params.m_jointStiffness[jacobIndex] = dgFloat32 (1.0f);

		params.m_restitution[jacobIndex] = dgFloat32 (0.0f);
		params.m_penetration[jacobIndex] = dgFloat32 (0.0f);
		params.m_penetrationStiffness[jacobIndex] = dgFloat32 (0.0f);
//		if (contact.m_override1Accel) {
		if (contact.m_flags & dgContactMaterial::m_override1Accel) {
			dgAssert (0);
			params.m_jointAccel[jacobIndex] = contact.m_dir1_Force.m_force;
			params.m_isMotor[jacobIndex] = 1;
		} else {
			//params.m_jointAccel[jacobIndex] = relVelocErr * params.m_invTimestep;
			params.m_jointAccel[jacobIndex] = relVelocErr * impulseOrForceScale;
			params.m_isMotor[jacobIndex] = 0;
		}
		if (dgAbsf (relVelocErr) > MAX_DYNAMIC_FRICTION_SPEED) {
			params.m_forceBounds[jacobIndex].m_low = - contact.m_dynamicFriction1;
			params.m_forceBounds[jacobIndex].m_upper = contact.m_dynamicFriction1;
		} else {
			params.m_forceBounds[jacobIndex].m_low = - contact.m_staticFriction1;
			params.m_forceBounds[jacobIndex].m_upper = contact.m_staticFriction1;
		}
		params.m_forceBounds[jacobIndex].m_jointForce = (dgForceImpactPair*)&contact.m_dir1_Force;
	}
//dgTrace (("p(%f %f %f)\n", params.m_jointAccel[normalIndex], params.m_jointAccel[normalIndex + 1], params.m_jointAccel[normalIndex + 2]));
}


void dgContact::JointAccelerations(dgJointAccelerationDecriptor* const params)
{
	dgJacobianMatrixElement* const rowMatrix = params->m_rowMatrix;
	const dgVector& bodyVeloc0 = m_body0->m_veloc;
	const dgVector& bodyOmega0 = m_body0->m_omega;
	const dgVector& bodyVeloc1 = m_body1->m_veloc;
	const dgVector& bodyOmega1 = m_body1->m_omega;

	dgInt32 count = params->m_rowsCount;

	dgFloat32 timestep = dgFloat32 (1.0f);
	dgFloat32 invTimestep = dgFloat32 (1.0f);
	if (params->m_timeStep > dgFloat32 (0.0f)) {
		timestep = params->m_timeStep;
		invTimestep = params->m_invTimeStep;
	}

	for (dgInt32 k = 0; k < count; k ++) {
		if (!rowMatrix[k].m_accelIsMotor) {
			dgJacobianMatrixElement* const row = &rowMatrix[k];

			dgVector relVeloc (row->m_Jt.m_jacobianM0.m_linear.CompProduct4(bodyVeloc0) + row->m_Jt.m_jacobianM0.m_angular.CompProduct4(bodyOmega0) + row->m_Jt.m_jacobianM1.m_linear.CompProduct4(bodyVeloc1) + row->m_Jt.m_jacobianM1.m_angular.CompProduct4(bodyOmega1));
			dgFloat32 vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
			dgFloat32 aRel = row->m_deltaAccel;

			if (row->m_normalForceIndex < 0) {
				//dgFloat32 restitution = dgFloat32 (1.0f);
				//if (vRel <= dgFloat32 (0.0f)) {
					//restitution += row->m_restitution;
				//}
				dgFloat32 restitution = (vRel <= dgFloat32 (0.0f)) ? (dgFloat32 (1.0f) + row->m_restitution) : dgFloat32 (1.0f);

				dgFloat32 penetrationVeloc = dgFloat32 (0.0f);
				if (row->m_penetration > DG_RESTING_CONTACT_PENETRATION * dgFloat32 (0.125f)) {
					if (vRel > dgFloat32 (0.0f)) {
						dgFloat32 penetrationCorrection = vRel * timestep;
						dgAssert (penetrationCorrection >= dgFloat32 (0.0f));
						row->m_penetration = dgMax (dgFloat32 (0.0f), row->m_penetration - penetrationCorrection);
					}
					penetrationVeloc = -(row->m_penetration * row->m_penetrationStiffness);
				}

				vRel *= restitution;
				vRel = dgMin (dgFloat32 (4.0f), vRel + penetrationVeloc);
			}
			row->m_coordenateAccel = (aRel - vRel * invTimestep);
		}
	}
}

void dgContact::JointVelocityCorrection(dgJointAccelerationDecriptor* const params)
{
	dgAssert (0);
}


dgCollidingPairCollector::dgCollidingPairCollector ()
	:m_count(0)
	,m_maxSize(0)
//	,m_pairs(NULL)
	,m_sentinel(NULL)
	,m_lock()
{
}

dgCollidingPairCollector::~dgCollidingPairCollector ()
{
}

void dgCollidingPairCollector::Init ()
{
	dgWorld* const world = (dgWorld*) this;
	// need to expand the buffer, clang compile does no like this is actually a function call to expand a buffer 
	//world->m_pairMemoryBuffer[0];
	//I am writing 0 to resolve the issue
	world->m_pairMemoryBuffer[0] = 0;
	m_count = 0;
}


void dgCollidingPairCollector::AddPair (dgContact* const contact, dgInt32 threadIndex)
{
	dgWorld* const world = (dgWorld*) this;
	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;
	dgAssert (body0 != m_sentinel);
	dgAssert (body1 != m_sentinel);
	dgAssert (contact->GetId() == dgConstraint::m_contactConstraint);
	dgAssert (body0->GetWorld());
	dgAssert (body1->GetWorld());
	dgAssert (body0->GetWorld() == world);
	dgAssert (body1->GetWorld() == world);
	if (!(body0->m_collideWithLinkedBodies & body1->m_collideWithLinkedBodies)) {
		if (world->AreBodyConnectedByJoints (body0, body1)) {
			return;
		}
	}

	const dgContactMaterial* const material = contact->m_material;
	if (material->m_flags & dgContactMaterial::m_collisionEnable) {
		dgInt32 processContacts = 1;
		if (material->m_aabbOverlap) {
			processContacts = material->m_aabbOverlap (*material, *body0, *body1, threadIndex);
		}
		if (processContacts) {

			dgAssert (!body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI));
			dgAssert (!body1->m_collision->IsType (dgCollision::dgCollisionNull_RTTI));

			dgThreadHiveScopeLock lock (world, &m_lock);
			if (world->m_pairMemoryBuffer.ExpandCapacityIfNeessesary (m_count, sizeof (dgPair))) {
				m_maxSize = dgInt32 (world->m_pairMemoryBuffer.GetBytesCapacity() / sizeof (dgPair));
			}
			dgPair* const pairs = (dgPair*) &world->m_pairMemoryBuffer[0];
			dgPair* const pair = &pairs[m_count];
			m_count ++;
			pair->m_contact = contact;
			pair->m_isDeformable = 0;
		}
	}
}






