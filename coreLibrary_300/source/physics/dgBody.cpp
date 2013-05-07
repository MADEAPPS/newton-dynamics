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
#include "dgCollision.h"
#include "dgCollisionInstance.h"
#include "dgCollisionCompound.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionDeformableMesh.h"
#include "dgCollisionCompoundBreakable.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DG_AABB_ERROR		dgFloat32 (1.0e-4f)
#define DG_MAX_ANGLE_STEP   dgFloat32 (45.0f * dgDEG2RAD)




dgBody::dgBody()
	:m_matrix (dgGetIdentityMatrix())
	,m_veloc(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_omega(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_minAABB(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_maxAABB(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_rotation(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f))
	,m_localCentreOfMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))	
	,m_globalCentreOfMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))	
	,m_index(0)
	,m_uniqueID(0)
	,m_bodyGroupId(0)
	,m_rtti(m_baseBodyRTTI)
	,m_genericLRUMark(0)
	,m_criticalSectionLock()
	,m_flags(0)
//	,m_freeze(0)
//	,m_sleeping(0)
//	,m_autoSleep(true)
//	,m_equilibrium(0)
//	,m_continueCollisionMode(0)
//	,m_spawnnedFromCallback(0)
//	,m_collideWithLinkedBodies(true)
//	,m_inCallback(0)
	,m_userData(NULL)
	,m_world(NULL)
	,m_collision(NULL)
	,m_collisionCell(NULL)
	,m_masterNode(NULL)
	,m_destructor(NULL)
	,m_matrixUpdate(NULL)
{
	m_autoSleep = true;
	m_collidable = true;
	m_collideWithLinkedBodies = true;
}

dgBody::dgBody (dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionCashe, dgDeserialize serializeCallback, void* const userData)
	:m_matrix (dgGetIdentityMatrix())
	,m_veloc(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_omega(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_minAABB(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_maxAABB(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_rotation(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f))
	,m_localCentreOfMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))	
	,m_globalCentreOfMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))	
	,m_index(0)
	,m_uniqueID(0)
	,m_bodyGroupId(0)
	,m_rtti(m_baseBodyRTTI)
	,m_genericLRUMark(0)
	,m_criticalSectionLock()
	,m_flags(0)
//	,m_freeze(0)
//	,m_sleeping(0)
//	,m_autoSleep(true)
//	,m_equilibrium(0)
//	,m_continueCollisionMode(0)
//	,m_spawnnedFromCallback(0)
//	,m_collideWithLinkedBodies(true)
//	,m_inCallback(0)
	,m_userData(NULL)
	,m_world(world)
	,m_collision(NULL)
	,m_collisionCell(NULL)
	,m_masterNode(NULL)
	,m_destructor(NULL)
	,m_matrixUpdate(NULL)
{
	m_autoSleep = true;
	m_collidable = true;
	m_collideWithLinkedBodies = true;

	serializeCallback (userData, &m_rotation, sizeof (m_rotation));
	serializeCallback (userData, &m_matrix.m_posit, sizeof (m_matrix.m_posit));
	serializeCallback (userData, &m_veloc, sizeof (m_veloc));
	serializeCallback (userData, &m_omega, sizeof (m_omega));
	serializeCallback (userData, &m_localCentreOfMass, sizeof (m_localCentreOfMass));
	serializeCallback (userData, &m_flags, sizeof (m_flags));

	m_matrix = dgMatrix (m_rotation, m_matrix.m_posit);

	dgInt32 id;
	serializeCallback (userData, &id, sizeof (id));

	dgTree<const dgCollision*, dgInt32>::dgTreeNode* const node = collisionCashe->Find(id);
	dgAssert (node);

	const dgCollision* const collision = node->GetInfo();
	collision->AddRef();

	dgCollisionInstance* const instance = new (world->GetAllocator()) dgCollisionInstance (world, serializeCallback, userData);
	instance->m_childShape = collision;
	m_collision = instance;
}

dgBody::~dgBody()
{
}

void dgBody::AttachCollision (dgCollisionInstance* const collisionSrc)
{
	dgCollisionInstance* const instance = new (m_world->GetAllocator()) dgCollisionInstance (*collisionSrc);
	if (m_collision) {
		m_collision->Release();
	}
	m_collision = instance;
}



void dgBody::Serialize (const dgTree<dgInt32, const dgCollision*>* const collisionCashe, dgSerialize serializeCallback, void* const userData)
{
	serializeCallback (userData, &m_rotation, sizeof (m_rotation));
	serializeCallback (userData, &m_matrix.m_posit, sizeof (m_matrix.m_posit));
	serializeCallback (userData, &m_veloc, sizeof (m_veloc));
	serializeCallback (userData, &m_omega, sizeof (m_omega));
	serializeCallback (userData, &m_localCentreOfMass, sizeof (m_localCentreOfMass));
	serializeCallback (userData, &m_flags, sizeof (m_flags));

	dgTree<dgInt32, const dgCollision*>::dgTreeNode* const node = collisionCashe->Find(m_collision->GetChildShape());
	dgAssert (node);

	dgInt32 id = node->GetInfo();
	serializeCallback (userData, &id, sizeof (id));
	m_collision->Serialize(serializeCallback, userData, false);
}


void dgBody::SetMatrix(const dgMatrix& matrix)
{
	SetMatrixOriginAndRotation(matrix);

	if (!m_inCallback) {
		if (m_world->m_cpu == dgSimdPresent) {
			UpdateCollisionMatrixSimd (dgFloat32 (0.0f), 0);
		} else {
			UpdateCollisionMatrix (dgFloat32 (0.0f), 0);
		}
	}
}

void dgBody::SetMatrixIgnoreSleep(const dgMatrix& matrix)
{
	m_sleeping = false;
	SetMatrix(matrix);
}


void dgBody::UpdateCollisionMatrixSimd (dgFloat32 timestep, dgInt32 threadIndex)
{
	m_collision->SetGlobalMatrix (m_collision->GetLocalMatrix().MultiplySimd(m_matrix));

	dgVector oldP0 (m_minAABB);
	dgVector oldP1 (m_maxAABB);
	m_collision->CalcAABBSimd(m_collision->GetGlobalMatrix(), m_minAABB, m_maxAABB);

	dgAssert (0);
	//UpdatePredictinVelocity(timestep);

	if (m_continueCollisionMode) {
		dgAssert (0);
	}

	if (m_collisionCell) {
		dgAssert (m_world);

		if (!m_sleeping) {
			dgSimd tol (DG_AABB_ERROR, DG_AABB_ERROR, DG_AABB_ERROR, DG_AABB_ERROR);
			dgSimd diff0 (((dgSimd&)oldP0) - ((dgSimd&)m_minAABB));
			dgSimd diff1 (((dgSimd&)oldP1) - ((dgSimd&)m_maxAABB));
			dgSimd test ((diff0.Abs() > tol) | (diff1.Abs() > tol)); 
			dgInt32 signMask = test.GetSignMask();
			if (signMask & 0x07) {
				m_world->GetBroadPhase()->UpdateBodyBroadphaseSimd (this, threadIndex);
			}
		}
	}
}


void dgBody::UpdateWorlCollsionMatrix() const
{
	m_collision->SetGlobalMatrix (m_collision->GetLocalMatrix() * m_matrix);
}



void dgBody::UpdateCollisionMatrix (dgFloat32 timestep, dgInt32 threadIndex)
{
	//m_collision->SetGlobalMatrix (m_collision->GetLocalMatrix() * m_matrix);
	UpdateWorlCollsionMatrix();

	dgVector oldP0 (m_minAABB);
	dgVector oldP1 (m_maxAABB);
	m_collision->CalcAABB (m_collision->GetGlobalMatrix(), m_minAABB, m_maxAABB);

	if (m_continueCollisionMode) {
		dgVector predictiveVeloc (PredictLinearVelocity(timestep));
		dgVector predictiveOmega (PredictAngularVelocity(timestep));
		dgMovingAABB (m_minAABB, m_maxAABB, predictiveVeloc, predictiveOmega, timestep, m_collision->GetBoxMaxRadius(), m_collision->GetBoxMinRadius());

//      this is no longer necessary since function dgMovingAABB does the job
//		if (m_collision->IsType (dgCollision::dgCollisionCompound_RTTI)) {
//			dgCollisionCompound *const compoundCollision = (dgCollisionCompound *)m_collision;
//			dgVector box1Min;
//			dgVector box1Max;
//			compoundCollision-> GetAABB (box1Min, box1Max);

//			dgVector boxSize ((box1Max - box1Min).Scale (dgFloat32 (0.25f)));
//			for (dgInt32 j = 0; j < 3; j ++) {
//				if (dgAbsf (step[j]) > boxSize[j]) {
//					if (step[j] > dgFloat32 (0.0f)) {
//						box1Max[j] += step[j]; 
//					} else {
//						box1Min[j] += step[j]; 
//					}
//				}
//			}
//		}
	}

	if (m_collisionCell) {
		dgAssert (m_world);
		
		if (!m_sleeping) {
			if ((dgAbsf (oldP0.m_x - m_minAABB.m_x) > DG_AABB_ERROR) || (dgAbsf (oldP0.m_y - m_minAABB.m_y) > DG_AABB_ERROR) ||
				(dgAbsf (oldP0.m_z - m_minAABB.m_z) > DG_AABB_ERROR) || (dgAbsf (oldP1.m_x - m_maxAABB.m_x) > DG_AABB_ERROR) || 
				(dgAbsf (oldP1.m_y - m_maxAABB.m_y) > DG_AABB_ERROR) || (dgAbsf (oldP1.m_z - m_maxAABB.m_z) > DG_AABB_ERROR)) {
				m_world->GetBroadPhase()->UpdateBodyBroadphase (this, threadIndex);
			}
		}
	}
}


dgFloat32 dgBody::RayCastSimd (const dgLineBox& line, OnRayCastAction filter, OnRayPrecastAction preFilter, void* const userData, dgFloat32 minT) const
{	
/*
	dgAssert (filter);
	if (dgOverlapTestSimd (line.m_boxL0, line.m_boxL1, m_minAABB, m_maxAABB)) {
		dgContactPoint contactOut;
		const dgMatrix& globalMatrix = m_collision->GetGlobalMatrix();
		dgVector localP0 (globalMatrix.UntransformVectorSimd(line.m_l0));
		dgVector localP1 (globalMatrix.UntransformVectorSimd(line.m_l1));
		dgFloat32 t = m_collision->RayCastSimd(localP0, localP1, contactOut, preFilter, this, userData);
		if (t < minT) {
			dgAssert (t >= 0.0f);
			dgAssert (t <= 1.0f);
			contactOut.m_normal = globalMatrix.RotateVectorSimd(contactOut.m_normal);
			minT = filter (this, contactOut.m_normal, dgInt32 (contactOut.m_userId), userData, t);
		}
	}
	return minT;
*/
	dgAssert (0);
	return 0;
}

dgFloat32 dgBody::RayCast (const dgLineBox& line, OnRayCastAction filter, OnRayPrecastAction preFilter, void* const userData, dgFloat32 minT) const
{
	dgAssert (filter);

	dgVector l0 (line.m_l0);
	dgVector l1 (line.m_l1);
	if (dgRayBoxClip (l0, l1, m_minAABB, m_maxAABB)) {
		dgContactPoint contactOut;
		const dgMatrix& globalMatrix = m_collision->GetGlobalMatrix();
		dgVector localP0 (globalMatrix.UntransformVector (l0));
		dgVector localP1 (globalMatrix.UntransformVector (l1));
		dgFloat32 t = m_collision->RayCast (localP0, localP1, contactOut, preFilter, this, userData);
		if (t < dgFloat32 (1.0f)) {
			dgVector p (globalMatrix.TransformVector(localP0 + (localP1 - localP0).Scale(t)));
			dgVector l1l0 (line.m_l1 - line.m_l0);
			t = ((p - line.m_l0) % l1l0) / (l1l0 % l1l0);
			if (t < minT) {
				dgAssert (t >= 0.0f);
				dgAssert (t <= 1.0f);
				contactOut.m_normal = globalMatrix.RotateVector (contactOut.m_normal);
				minT = filter (this, contactOut.m_collision0, contactOut.m_normal, (void*)contactOut.m_shapeId0, userData, t);
			}
		}
	} 
	return minT;
}


void dgBody::AddBuoyancyForce (dgFloat32 fluidDensity, dgFloat32 fluidLinearViscousity, dgFloat32 fluidAngularViscousity, const dgVector& gravityVector, GetBuoyancyPlane buoyancuPlane, void* const context)
{
	dgAssert (0);
/*
	if (m_mass.m_w > dgFloat32 (1.0e-2f)) {
		dgVector volumeIntegral (m_collision->CalculateVolumeIntegral (m_collision->GetGlobalMatrix(), buoyancuPlane, context));
		if (volumeIntegral.m_w > dgFloat32 (1.0e-4f)) {

//			dgVector buoyanceCenter (volumeIntegral - m_matrix.m_posit);
			dgVector buoyanceCenter (volumeIntegral - m_globalCentreOfMass);

			dgVector force (gravityVector.Scale (-fluidDensity * volumeIntegral.m_w));
			dgVector torque (buoyanceCenter * force);

			dgFloat32 damp = GetMax (GetMin ((m_veloc % m_veloc) * dgFloat32 (100.0f) * fluidLinearViscousity, dgFloat32 (1.0f)), dgFloat32(dgFloat32 (10.0f)));
			force -= m_veloc.Scale (damp);

			//damp = (m_omega % m_omega) * dgFloat32 (10.0f) * fluidAngularViscousity;
			damp = GetMax (GetMin ((m_omega % m_omega) * dgFloat32 (1000.0f) * fluidAngularViscousity, dgFloat32(0.25f)), dgFloat32(2.0f));
			torque -= m_omega.Scale (damp);

//			dgAssert (dgSqrt (force % force) < (dgSqrt (gravityVector % gravityVector) * m_mass.m_w * dgFloat32 (100.0f)));
//			dgAssert (dgSqrt (torque % torque) < (dgSqrt (gravityVector % gravityVector) * m_mass.m_w * dgFloat32 (100.0f) * dgFloat32 (10.0f)));
			
			dgThreadHiveScopeLock lock (m_world, &m_criticalSectionLock);
			m_accel += force;
			m_alpha += torque;
 		}
	}
*/
}



void dgBody::UpdateMatrix (dgFloat32 timestep, dgInt32 threadIndex)
{
	if (m_matrixUpdate) {
		m_matrixUpdate (*this, m_matrix, threadIndex);
	}
	if (m_world->m_cpu == dgSimdPresent) {
		UpdateCollisionMatrixSimd (timestep, threadIndex);
	} else {
		UpdateCollisionMatrix (timestep, threadIndex);
	}
}


void dgBody::IntegrateVelocity (dgFloat32 timestep)
{
	m_globalCentreOfMass += m_veloc.Scale (timestep); 
	while (((m_omega % m_omega) * timestep * timestep) > (DG_MAX_ANGLE_STEP * DG_MAX_ANGLE_STEP)) {
		m_omega = m_omega.Scale (dgFloat32 (0.8f));
	}

	// this is correct
	dgFloat32 omegaMag2 = m_omega % m_omega;
	if (omegaMag2 > ((dgFloat32 (0.0125f) * dgDEG2RAD) * (dgFloat32 (0.0125f) * dgDEG2RAD))) {
		dgFloat32 invOmegaMag = dgRsqrt (omegaMag2);
		dgVector omegaAxis (m_omega.Scale (invOmegaMag));
		dgFloat32 omegaAngle = invOmegaMag * omegaMag2 * timestep;
		dgQuaternion rotation (omegaAxis, omegaAngle);
		m_rotation = m_rotation * rotation;
		m_rotation.Scale(dgRsqrt (m_rotation.DotProduct (m_rotation)));
		m_matrix = dgMatrix (m_rotation, m_matrix.m_posit);
	}

	m_matrix.m_posit = m_globalCentreOfMass - m_matrix.RotateVector(m_localCentreOfMass);

#ifdef _DEBUG
	for (dgInt32 i = 0; i < 4; i ++) {
		for (dgInt32 j = 0; j < 4; j ++) {
			dgAssert (dgCheckFloat(m_matrix[i][j]));
		}
	}

	dgInt32 j0 = 1;
	dgInt32 j1 = 2;
	for (dgInt32 i = 0; i < 3; i ++) {
		dgAssert (m_matrix[i][3] == 0.0f);
		dgFloat32 val = m_matrix[i] % m_matrix[i];
		dgAssert (dgAbsf (val - 1.0f) < 1.0e-5f);
		dgVector tmp (m_matrix[j0] * m_matrix[j1]);
		val = tmp % m_matrix[i];
		dgAssert (dgAbsf (val - 1.0f) < 1.0e-5f);
		j0 = j1;
		j1 = i;
	}
#endif
}



dgVector dgBody::CalculateInverseDynamicForce (const dgVector& desiredVeloc, dgFloat32 timestep) const
{
	dgAssert (0);
	return dgVector();
/*
	dgFloat32 massAccel = m_mass.m_w / timestep;
	if (m_world->m_solverMode){
		if (m_masterNode->GetInfo().GetCount() >= 2){
			massAccel *= (dgFloat32 (2.0f) * dgFloat32 (LINEAR_SOLVER_SUB_STEPS) / dgFloat32 (LINEAR_SOLVER_SUB_STEPS + 1));
		} 
	}
	return (desiredVeloc - m_veloc).Scale (massAccel);
*/
}


dgConstraint* dgBody::GetFirstJoint() const
{
	for (dgBodyMasterListRow::dgListNode* node = m_masterNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
		dgConstraint* joint;
		joint = node->GetInfo().m_joint;
		if (joint->GetId() >= dgConstraint::m_unknownConstraint) {
			return joint;
		}
	}
	return NULL;
}

dgConstraint* dgBody::GetNextJoint(dgConstraint* const joint) const
{
	dgBodyMasterListRow::dgListNode* node = joint->GetLink0();
	if (joint->GetBody0() != this) {
		node = joint->GetLink1();
	}

	if (node->GetInfo().m_joint == joint) {
		for (node = node->GetNext(); node; node = node->GetNext()) {
			dgConstraint* joint;
			joint = node->GetInfo().m_joint;
			if (joint->GetId() >= dgConstraint::m_unknownConstraint) {
				return joint;
			}
		}
	}

	return NULL;
}


dgConstraint* dgBody::GetFirstContact() const
{
	for (dgBodyMasterListRow::dgListNode* node = m_masterNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
		dgConstraint* const joint = node->GetInfo().m_joint;
		if ((joint->GetId() == dgConstraint::m_contactConstraint) && (joint->GetMaxDOF() != 0)) {
			return joint;
		}
	}
	return NULL;
}

dgConstraint* dgBody::GetNextContact(dgConstraint* const joint) const
{
	dgBodyMasterListRow::dgListNode* node = joint->GetLink0();
	if (joint->GetBody0() != this) {
		node = joint->GetLink1();
	}

	if (node->GetInfo().m_joint == joint) {
		for (node = node->GetNext(); node; node = node->GetNext()) {
			dgConstraint* const joint = node->GetInfo().m_joint;
			if ((joint->GetId() == dgConstraint::m_contactConstraint) && (joint->GetMaxDOF() != 0)) {
				return joint;
			}
		}
	}

	return NULL;
}




void dgBody::SetFreeze (bool state)
{
	if (state){
		Freeze();
	} else {
		Unfreeze();	
	}
}


void dgBody::Freeze ()
{
	if (GetInvMass().m_w > dgFloat32 (0.0f)) {
		if (!m_freeze) {
			m_freeze = true;
			for (dgBodyMasterListRow::dgListNode* node = m_masterNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
				dgBody* const body = node->GetInfo().m_bodyNode;
				body->Freeze ();
			}
		}
	}
}

void dgBody::Unfreeze ()
{
	if (GetInvMass().m_w > dgFloat32 (0.0f)) {
// note this is in observation (to prevent bodies from not going to sleep  inside triggers	               
//		m_equilibrium = false;			
		if (m_freeze) {
			m_freeze = false;
			for (dgBodyMasterListRow::dgListNode* node = m_masterNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
				dgBody* const body = node->GetInfo().m_bodyNode;
				body->Unfreeze ();
			}
		}
	}
}


void dgBody::InvalidateCache ()
{
	m_sleeping = false;
	m_equilibrium = false;
	m_genericLRUMark = 0;
	dgMatrix matrix (m_matrix);
	SetMatrixOriginAndRotation(matrix);
}


