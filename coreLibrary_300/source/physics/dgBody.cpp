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
#include "dgCollisionCompoundFractured.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define DG_AABB_ERROR		dgFloat32 (1.0e-4f)
#define DG_MAX_ANGLE_STEP   dgFloat32 (45.0f * dgDEG2RAD)

//dgVector dgBody::m_aabbTol (DG_AABB_ERROR, DG_AABB_ERROR, DG_AABB_ERROR, dgFloat32 (0.0));

dgBody::dgBody()
	:m_invWorldInertiaMatrix(dgGetZeroMatrix())
	,m_matrix (dgGetIdentityMatrix())
	,m_rotation(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f))
	,m_mass(dgFloat32 (DG_INFINITE_MASS * 2.0f), dgFloat32 (DG_INFINITE_MASS * 2.0f), dgFloat32 (DG_INFINITE_MASS * 2.0f), dgFloat32 (DG_INFINITE_MASS * 2.0f))
	,m_invMass(dgFloat32 (0.0))
	,m_veloc(dgFloat32 (0.0))
	,m_omega(dgFloat32 (0.0))
	,m_minAABB(dgFloat32 (0.0))
	,m_maxAABB(dgFloat32 (0.0))
	,m_netForce(dgFloat32 (0.0))
	,m_netTorque(dgFloat32 (0.0))
	,m_localCentreOfMass(dgFloat32 (0.0))	
	,m_globalCentreOfMass(dgFloat32 (0.0))	
	,m_aparentMass(dgFloat32 (DG_INFINITE_MASS), dgFloat32 (DG_INFINITE_MASS), dgFloat32 (DG_INFINITE_MASS), dgFloat32 (DG_INFINITE_MASS))
	,m_index(0)
	,m_uniqueID(0)
	,m_bodyGroupId(0)
	,m_rtti(m_baseBodyRTTI)
	,m_type(0)
	,m_dynamicsLru(0)
	,m_genericLRUMark(0)
	,m_criticalSectionLock()
	,m_flags(0)
	,m_userData(NULL)
	,m_world(NULL)
	,m_collision(NULL)
	,m_collisionCell(NULL)
	,m_masterNode(NULL)
	,m_destructor(NULL)
	,m_matrixUpdate(NULL)
{
	m_resting = true;
	m_autoSleep = true;
	m_collidable = true;
	m_collideWithLinkedBodies = true;
	m_invWorldInertiaMatrix[3][3] = dgFloat32 (1.0f);
}

dgBody::dgBody (dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionCashe, dgDeserialize serializeCallback, void* const userData)
	:m_invWorldInertiaMatrix(dgGetZeroMatrix())
	,m_matrix (dgGetIdentityMatrix())
	,m_rotation(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f))
	,m_mass(dgFloat32 (DG_INFINITE_MASS * 2.0f), dgFloat32 (DG_INFINITE_MASS * 2.0f), dgFloat32 (DG_INFINITE_MASS * 2.0f), dgFloat32 (DG_INFINITE_MASS * 2.0f))
	,m_invMass(dgFloat32 (0.0))
	,m_veloc(dgFloat32 (0.0))
	,m_omega(dgFloat32 (0.0))
	,m_minAABB(dgFloat32 (0.0))
	,m_maxAABB(dgFloat32 (0.0))
	,m_netForce(dgFloat32 (0.0))
	,m_netTorque(dgFloat32 (0.0))
	,m_localCentreOfMass(dgFloat32 (0.0))	
	,m_globalCentreOfMass(dgFloat32 (0.0))	
	,m_aparentMass(dgFloat32 (DG_INFINITE_MASS), dgFloat32 (DG_INFINITE_MASS), dgFloat32 (DG_INFINITE_MASS), dgFloat32 (DG_INFINITE_MASS))
	,m_index(0)
	,m_uniqueID(0)
	,m_bodyGroupId(0)
	,m_rtti(m_baseBodyRTTI)
	,m_type(0)
	,m_dynamicsLru(0)
	,m_genericLRUMark(0)
	,m_criticalSectionLock()
	,m_flags(0)
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
	m_invWorldInertiaMatrix[3][3] = dgFloat32 (1.0f);

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
	m_equilibrium = 0;
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
		UpdateCollisionMatrix (dgFloat32 (0.0f), 0);
	}
}

void dgBody::SetMatrixIgnoreSleep(const dgMatrix& matrix)
{
	m_sleeping = false;
	SetMatrix(matrix);
}

void dgBody::UpdateWorlCollisionMatrix() const
{
	m_collision->SetGlobalMatrix (m_collision->GetLocalMatrix() * m_matrix);
}



void dgBody::UpdateCollisionMatrix (dgFloat32 timestep, dgInt32 threadIndex)
{
	m_collision->SetGlobalMatrix (m_collision->GetLocalMatrix() * m_matrix);
	m_collision->CalcAABB (m_collision->GetGlobalMatrix(), m_minAABB, m_maxAABB);

	if (m_continueCollisionMode) {
		dgVector predictiveVeloc (PredictLinearVelocity(timestep));
		dgVector predictiveOmega (PredictAngularVelocity(timestep));
		dgMovingAABB (m_minAABB, m_maxAABB, predictiveVeloc, predictiveOmega, timestep, m_collision->GetBoxMaxRadius(), m_collision->GetBoxMinRadius());
	}

	if (m_collisionCell) {
		dgAssert (m_world);
		
		if (!m_sleeping) {
			m_world->GetBroadPhase()->UpdateBodyBroadphase (this, threadIndex);
		}
	}
}


dgFloat32 dgBody::RayCast (const dgLineBox& line, OnRayCastAction filter, OnRayPrecastAction preFilter, void* const userData, dgFloat32 maxT) const
{
	dgAssert (filter);
	dgVector l0 (line.m_l0);
//	dgVector l1 (line.m_l1);
	dgVector l1 (line.m_l0 + (line.m_l1 - line.m_l0).Scale4 (dgMin(maxT, dgFloat32 (1.0f))));
	if (dgRayBoxClip (l0, l1, m_minAABB, m_maxAABB)) {
		dgContactPoint contactOut;
		const dgMatrix& globalMatrix = m_collision->GetGlobalMatrix();
		dgVector localP0 (globalMatrix.UntransformVector (l0));
		dgVector localP1 (globalMatrix.UntransformVector (l1));
		dgVector p1p0 (localP1 - localP0);
		if ((p1p0 % p1p0) > dgFloat32 (1.0e-12f)) {
			dgFloat32 t = m_collision->RayCast (localP0, localP1, dgFloat32 (1.0f), contactOut, preFilter, this, userData);
			if (t < dgFloat32 (1.0f)) {
				dgVector p (globalMatrix.TransformVector(localP0 + (localP1 - localP0).Scale3(t)));
				dgVector l1l0 (line.m_l1 - line.m_l0);
				t = ((p - line.m_l0) % l1l0) / (l1l0 % l1l0);
				if (t < maxT) {
					dgAssert (t >= dgFloat32 (0.0f));
					dgAssert (t <= dgFloat32 (1.0f));
					contactOut.m_normal = globalMatrix.RotateVector (contactOut.m_normal);
					maxT = filter (this, contactOut.m_collision0, p, contactOut.m_normal, contactOut.m_shapeId0, userData, t);
				}
			}
		}
	} 
	return maxT;
}



dgFloat32 dgBody::ConvexRayCast (const dgFastRayTest& ray, const dgCollisionInstance* const convexShape, const dgVector& shapeMinBox, const dgVector& shapeMaxBox, const dgMatrix& origin, const dgVector& shapeVeloc, OnRayCastAction filter, OnRayPrecastAction preFilter, void* const userData, dgFloat32 minT, dgInt32 threadId) const
{
	dgVector minBox (m_minAABB - shapeMaxBox);
	dgVector maxBox (m_maxAABB - shapeMinBox);
	if (ray.BoxTest (minBox, maxBox)) {
		dgContactPoint contactOut;
		dgFloat32 t = m_collision->ConvexRayCast (convexShape, origin, shapeVeloc, minT, contactOut, preFilter, this, userData, threadId);
		if (t < minT) {
			dgAssert (t >= dgFloat32 (0.0f));
			dgAssert (t <= dgFloat32 (1.1f));
			minT = filter (this, contactOut.m_collision0, contactOut.m_point, contactOut.m_normal, contactOut.m_shapeId0, userData, t);
		}
	} 
	return minT;
}


void dgBody::UpdateMatrix (dgFloat32 timestep, dgInt32 threadIndex)
{
	if (m_matrixUpdate) {
		m_matrixUpdate (*this, m_matrix, threadIndex);
	}
	UpdateCollisionMatrix (timestep, threadIndex);
}


void dgBody::IntegrateVelocity (dgFloat32 timestep)
{
	m_globalCentreOfMass += m_veloc.Scale3 (timestep); 
	while (((m_omega % m_omega) * timestep * timestep) > (DG_MAX_ANGLE_STEP * DG_MAX_ANGLE_STEP)) {
		m_omega = m_omega.Scale3 (dgFloat32 (0.8f));
	}

	// this is correct
	dgFloat32 omegaMag2 = m_omega % m_omega;
	if (omegaMag2 > ((dgFloat32 (0.0125f) * dgDEG2RAD) * (dgFloat32 (0.0125f) * dgDEG2RAD))) {
		dgFloat32 invOmegaMag = dgRsqrt (omegaMag2);
		dgVector omegaAxis (m_omega.Scale3 (invOmegaMag));
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
	return (desiredVeloc - m_veloc).Scale3 (massAccel);
*/
}


dgConstraint* dgBody::GetFirstJoint() const
{
	if (m_masterNode) {
		for (dgBodyMasterListRow::dgListNode* node = m_masterNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
			dgConstraint* const joint = node->GetInfo().m_joint;
			if (joint->GetId() >= dgConstraint::m_unknownConstraint) {
				return joint;
			}
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
			dgConstraint* const joint = node->GetInfo().m_joint;
			if (joint->GetId() >= dgConstraint::m_unknownConstraint) {
				return joint;
			}
		}
	}

	return NULL;
}


dgConstraint* dgBody::GetFirstContact() const
{
	if (m_masterNode) {
		for (dgBodyMasterListRow::dgListNode* node = m_masterNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
			dgConstraint* const joint = node->GetInfo().m_joint;
			//if ((joint->GetId() == dgConstraint::m_contactConstraint) && (joint->GetMaxDOF() != 0)) {
			if (joint->GetId() == dgConstraint::m_contactConstraint) {
				return joint;
			}
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
			//if ((joint->GetId() == dgConstraint::m_contactConstraint) && (joint->GetMaxDOF() != 0)) {
			if (joint->GetId() == dgConstraint::m_contactConstraint) {
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



void dgBody::SetAparentMassMatrix (const dgVector& massMatrix)
{
	m_aparentMass = massMatrix;
	if (m_collision->IsType(dgCollision::dgCollisionMesh_RTTI)) {
		m_aparentMass.m_w = DG_INFINITE_MASS * 2.0f;
	}

	if (m_aparentMass.m_w >= DG_INFINITE_MASS) {
		m_aparentMass.m_x = DG_INFINITE_MASS;
		m_aparentMass.m_y = DG_INFINITE_MASS;
		m_aparentMass.m_z = DG_INFINITE_MASS;
		m_aparentMass.m_w = DG_INFINITE_MASS;
	}
}


void dgBody::SetMassMatrix(dgFloat32 mass, dgFloat32 Ixx, dgFloat32 Iyy, dgFloat32 Izz)
{
	mass = dgAbsf (mass);
	if (m_collision->IsType(dgCollision::dgCollisionMesh_RTTI) || m_collision->IsType(dgCollision::dgCollisionScene_RTTI)) {
		mass = DG_INFINITE_MASS * 2.0f;
	}

	if (mass < DG_MINIMUM_MASS) {
		mass = DG_INFINITE_MASS * 2.0f;
	}

	//dgAssert (m_masterNode);
	if (mass >= DG_INFINITE_MASS) {
		m_mass.m_x = DG_INFINITE_MASS;
		m_mass.m_y = DG_INFINITE_MASS;
		m_mass.m_z = DG_INFINITE_MASS;
		m_mass.m_w = DG_INFINITE_MASS;
		m_invMass.m_x = dgFloat32 (0.0f);
		m_invMass.m_y = dgFloat32 (0.0f);
		m_invMass.m_z = dgFloat32 (0.0f);
		m_invMass.m_w = dgFloat32 (0.0f);

		if (m_masterNode) {
			dgBodyMasterList& masterList (*m_world);
			if (masterList.GetFirst() != m_masterNode) {
				masterList.InsertAfter (masterList.GetFirst(), m_masterNode);
			}
		}
		SetAparentMassMatrix (m_mass);

	} else {
		Ixx = dgAbsf (Ixx);
		Iyy = dgAbsf (Iyy);
		Izz = dgAbsf (Izz);

		dgFloat32 Ixx1 = dgClamp (Ixx, dgFloat32 (0.001f) * mass, dgFloat32 (1000.0f) * mass);
		dgFloat32 Iyy1 = dgClamp (Iyy, dgFloat32 (0.001f) * mass, dgFloat32 (1000.0f) * mass);
		dgFloat32 Izz1 = dgClamp (Izz, dgFloat32 (0.001f) * mass, dgFloat32 (1000.0f) * mass);

		dgAssert (Ixx > dgFloat32 (0.0f));
		dgAssert (Iyy > dgFloat32 (0.0f));
		dgAssert (Izz > dgFloat32 (0.0f));

		m_mass.m_x = Ixx1;
		m_mass.m_y = Iyy1;
		m_mass.m_z = Izz1;
		m_mass.m_w = mass;

		m_invMass.m_x = dgFloat32 (1.0f) / Ixx1;
		m_invMass.m_y = dgFloat32 (1.0f) / Iyy1;
		m_invMass.m_z = dgFloat32 (1.0f) / Izz1;
		m_invMass.m_w = dgFloat32 (1.0f) / mass;

		if (m_masterNode) {
			dgBodyMasterList& masterList (*m_world);
			masterList.RotateToEnd (m_masterNode);
		}
		SetAparentMassMatrix (dgVector (Ixx, Iyy, Izz, mass));
	}

#ifdef _DEBUG
	dgBodyMasterList& me = *m_world;
	for (dgBodyMasterList::dgListNode* refNode = me.GetFirst(); refNode; refNode = refNode->GetNext()) {
		dgBody* const body0 = refNode->GetInfo().GetBody();
		dgVector invMass (body0->GetInvMass());
		if (invMass.m_w != 0.0f) {
			for (; refNode; refNode = refNode->GetNext()) {
				dgBody* const body1 = refNode->GetInfo().GetBody();
				dgVector invMass (body1->GetInvMass());
				dgAssert (invMass.m_w != 0.0f);
			}
			break;
		}
	}
#endif
}


void dgBody::SetMassProperties (dgFloat32 mass, const dgCollisionInstance* const collision)
{
	// using general central theorem, to extract the Inertia relative to the center of mass 
	//IIorigin = IImatrix - unitmass * [(displacemnet % displacemnet) * identityMatrix - transpose(displacement) * displacement)];
	dgMatrix inertia (collision->CalculateInertia());

	dgVector origin (inertia.m_posit);
	dgFloat32 mag = origin % origin;
	for (dgInt32 i = 0; i < 3; i ++) {
		inertia[i][i] = (inertia[i][i] - (mag - origin[i] * origin[i])) * mass;
		for (dgInt32 j = i + 1; j < 3; j ++) {
			dgFloat32 crossIJ = origin[i] * origin[j];
			inertia[i][j] = (inertia[i][j] + crossIJ) * mass;
			inertia[j][i] = (inertia[j][i] + crossIJ) * mass;
		}
	}

	// although the engine fully supports asymmetric inertia, I will ignore cross inertia for now
	SetMassMatrix(mass, inertia[0][0], inertia[1][1], inertia[2][2]);
	SetCentreOfMass(origin);
}

dgMatrix dgBody::CalculateInertiaMatrix () const
{
	dgMatrix tmp (m_matrix.Transpose4X4());
	tmp[0] = tmp[0].CompProduct4 (m_mass);
	tmp[1] = tmp[1].CompProduct4 (m_mass);
	tmp[2] = tmp[2].CompProduct4 (m_mass);
#if 0
	return tmp * m_matrix;
#else
	return dgMatrix (m_matrix.RotateVector(tmp[0]), m_matrix.RotateVector(tmp[1]), m_matrix.RotateVector(tmp[2]), dgVector::m_wOne);
#endif
}

dgMatrix dgBody::CalculateInvInertiaMatrix () const
{
	dgMatrix tmp (m_matrix.Transpose4X4());
	tmp[0] = tmp[0].CompProduct4(m_invMass);
	tmp[1] = tmp[1].CompProduct4(m_invMass);
	tmp[2] = tmp[2].CompProduct4(m_invMass);
#if 0
	return tmp * m_matrix;
#else
	return dgMatrix (m_matrix.RotateVector(tmp[0]), m_matrix.RotateVector(tmp[1]), m_matrix.RotateVector(tmp[2]), dgVector::m_wOne);
#endif
}

void dgBody::InvalidateCache ()
{
	m_sleeping = false;
	m_equilibrium = false;
	m_genericLRUMark = 0;
	dgMatrix matrix (m_matrix);
	SetMatrixOriginAndRotation(matrix);
}


void dgBody::AddImpulse (const dgVector& pointDeltaVeloc, const dgVector& pointPosit)
{
	dgMatrix invInertia (CalculateInvInertiaMatrix());

	// get contact matrix
	dgMatrix tmp;
	dgVector globalContact (pointPosit - m_globalCentreOfMass);

	tmp[0][0] = dgFloat32 (0.0f);
	tmp[0][1] = + globalContact[2];
	tmp[0][2] = - globalContact[1];
	tmp[0][3] = dgFloat32 (0.0f);

	tmp[1][0] = -globalContact[2];
	tmp[1][1] = dgFloat32 (0.0f);
	tmp[1][2] = +globalContact[0];
	tmp[1][3] = dgFloat32 (0.0f);

	tmp[2][0] = +globalContact[1];
	tmp[2][1] = -globalContact[0];
	tmp[2][2] = dgFloat32 (0.0f);
	tmp[2][3] = dgFloat32 (0.0f);

	tmp[3][0] = dgFloat32 (0.0f);
	tmp[3][1] = dgFloat32 (0.0f);
	tmp[3][2] = dgFloat32 (0.0f);
	tmp[3][3] = dgFloat32 (1.0f);

	dgMatrix contactMatrix (tmp * invInertia * tmp);
	for (dgInt32 i = 0; i < 3; i ++) {
		for (dgInt32 j = 0; j < 3; j ++) {
			contactMatrix[i][j] *= -dgFloat32 (1.0f);
		}
	}
	contactMatrix[0][0] += m_invMass.m_w;	
	contactMatrix[1][1] += m_invMass.m_w;	
	contactMatrix[2][2] += m_invMass.m_w;	

	contactMatrix = contactMatrix.Symetric3by3Inverse ();

	// change of momentum
	dgVector changeOfMomentum (contactMatrix.RotateVector (pointDeltaVeloc));


	dgVector dv (changeOfMomentum.Scale3 (m_invMass.m_w));
	dgVector dw (invInertia.RotateVector (globalContact * changeOfMomentum));

	m_veloc += dv;
	m_omega += dw;

	m_sleeping	= false;
	m_equilibrium = false;
	Unfreeze ();
}

void dgBody::ApplyImpulsePair (const dgVector& linearImpulseIn, const dgVector& angularImpulseIn)
{
	dgMatrix inertia (CalculateInertiaMatrix());
	dgMatrix invInertia (CalculateInvInertiaMatrix());

	dgVector linearImpulse (m_veloc.Scale3 (m_mass.m_w) + linearImpulseIn);
	dgVector angularImpulse (inertia.RotateVector (m_omega) + angularImpulseIn);

	m_veloc = linearImpulse.Scale3(m_invMass.m_w);
	m_omega = invInertia.RotateVector(angularImpulse);

	m_sleeping	= false;
	m_equilibrium = false;
	Unfreeze ();
}

void dgBody::ApplyImpulsesAtPoint (dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const impulseArray, const dgFloat32* const pointArray)
{
	dgInt32 stride = strideInBytes / sizeof (dgFloat32);

	dgMatrix inertia (CalculateInertiaMatrix());

	dgVector impulse (m_veloc.Scale3 (m_mass.m_w));
	dgVector angularImpulse (inertia.RotateVector (m_omega));

	dgVector com (m_globalCentreOfMass);
	for (dgInt32 i = 0; i < count; i ++) {
		dgInt32 index = i * stride;
		dgVector r (pointArray[index], pointArray[index + 1], pointArray[index + 2], dgFloat32 (0.0f));
		dgVector L (impulseArray[index], impulseArray[index + 1], impulseArray[index + 2], dgFloat32 (0.0f));
		dgVector Q ((r - com) * L);

		impulse += L;
		angularImpulse += Q;
	}

	dgMatrix invInertia (CalculateInvInertiaMatrix());
	m_veloc = impulse.Scale3(m_invMass.m_w);
	m_omega = invInertia.RotateVector(angularImpulse);

	m_sleeping	= false;
	m_equilibrium = false;
	Unfreeze ();
}