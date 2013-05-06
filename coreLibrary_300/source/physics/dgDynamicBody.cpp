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
#include "dgDynamicBody.h"
#include "dgCollisionInstance.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


dgDynamicBody::dgDynamicBody()
	:dgBody()
	,m_invWorldInertiaMatrix(dgGetZeroMatrix())
	,m_mass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_invMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_accel(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_alpha(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_netForce(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_netTorque(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalForce(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalTorque(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_dampCoef(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_aparentMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_sleepingCounter(0)
	,m_dynamicsLru(0)	
	,m_isInDestructionArrayLRU(0)
	,m_applyExtForces(NULL)
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;
	m_invWorldInertiaMatrix[3][3] = dgFloat32 (1.0f);
	dgAssert ( dgInt32 (sizeof (dgDynamicBody) & 0x0f) == 0);
}

dgDynamicBody::dgDynamicBody (dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionCashe, dgDeserialize serializeCallback, void* const userData)
	:dgBody(world, collisionCashe, serializeCallback, userData)
	,m_invWorldInertiaMatrix(dgGetZeroMatrix())
	,m_mass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_invMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_accel(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_alpha(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_netForce(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_netTorque(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalForce(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalTorque(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_dampCoef(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_aparentMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_sleepingCounter(0)
	,m_dynamicsLru(0)	
	,m_isInDestructionArrayLRU(0)
	,m_applyExtForces(NULL)
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;

	m_invWorldInertiaMatrix[3][3] = dgFloat32 (1.0f);

	serializeCallback (userData, &m_mass, sizeof (m_mass));
	serializeCallback (userData, &m_invMass, sizeof (m_invMass));
	serializeCallback (userData, &m_dampCoef, sizeof (m_dampCoef));
	serializeCallback (userData, &m_aparentMass, sizeof (m_aparentMass));
}

dgDynamicBody::~dgDynamicBody()
{
}





void dgDynamicBody::Serialize (const dgTree<dgInt32, const dgCollision*>* const collisionCashe, dgSerialize serializeCallback, void* const userData)
{
	dgBody::Serialize (collisionCashe, serializeCallback, userData);
	serializeCallback (userData, &m_mass, sizeof (m_mass));
	serializeCallback (userData, &m_invMass, sizeof (m_invMass));
	serializeCallback (userData, &m_dampCoef, sizeof (m_dampCoef));
	serializeCallback (userData, &m_aparentMass, sizeof (m_aparentMass));
}

/*
void dgDynamicBody::AddBuoyancyForce (dgFloat32 fluidDensity, dgFloat32 fluidLinearViscousity, dgFloat32 fluidAngularViscousity, const dgVector& gravityVector, GetBuoyancyPlane buoyancuPlane, void* const context)
{
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
}
*/

void dgDynamicBody::SetMatrixIgnoreSleep(const dgMatrix& matrix)
{
	dgBody::SetMatrixIgnoreSleep(matrix);
	m_prevExternalForce = dgVector (0.0f, 0.0f, 0.0f, 0.0f);
	m_prevExternalTorque = dgVector (0.0f, 0.0f, 0.0f, 0.0f);
	CalcInvInertiaMatrix();
}


void dgDynamicBody::AttachCollision (dgCollisionInstance* const collision)
{
	dgBody::AttachCollision(collision);
	if (m_collision->IsType(dgCollision::dgCollisionMesh_RTTI) || m_collision->IsType(dgCollision::dgCollisionScene_RTTI)) {
		SetMassMatrix (m_mass.m_w, m_mass.m_x, m_mass.m_y, m_mass.m_z);
	}
}


void dgDynamicBody::SetAparentMassMatrix (const dgVector& massMatrix)
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


void dgDynamicBody::SetMassMatrix(dgFloat32 mass, dgFloat32 Ixx, dgFloat32 Iyy, dgFloat32 Izz)
{
	mass = dgAbsf (mass);
	if (m_collision->IsType(dgCollision::dgCollisionMesh_RTTI) || m_collision->IsType(dgCollision::dgCollisionScene_RTTI)) {
		mass = DG_INFINITE_MASS * 2.0f;
	}

	if (mass < dgFloat32 (1.0e-3f)) {
		mass = DG_INFINITE_MASS * 2.0f;
	}

	dgAssert (m_masterNode);
	if (mass >= DG_INFINITE_MASS) {
		m_mass.m_x = DG_INFINITE_MASS;
		m_mass.m_y = DG_INFINITE_MASS;
		m_mass.m_z = DG_INFINITE_MASS;
		m_mass.m_w = DG_INFINITE_MASS;
		m_invMass.m_x = dgFloat32 (0.0f);
		m_invMass.m_y = dgFloat32 (0.0f);
		m_invMass.m_z = dgFloat32 (0.0f);
		m_invMass.m_w = dgFloat32 (0.0f);

		dgBodyMasterList& masterList (*m_world);
		if (masterList.GetFirst() != m_masterNode) {
			masterList.InsertAfter (masterList.GetFirst(), m_masterNode);
		}
		SetAparentMassMatrix (m_mass);

	} else {
		Ixx = dgAbsf (Ixx);
		Iyy = dgAbsf (Iyy);
		Izz = dgAbsf (Izz);

		dgFloat32 Ixx1 = dgClamp (Ixx, dgFloat32 (0.001f) * mass, dgFloat32 (100.0f) * mass);
		dgFloat32 Iyy1 = dgClamp (Iyy, dgFloat32 (0.001f) * mass, dgFloat32 (100.0f) * mass);
		dgFloat32 Izz1 = dgClamp (Izz, dgFloat32 (0.001f) * mass, dgFloat32 (100.0f) * mass);

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

		dgBodyMasterList& masterList (*m_world);
		masterList.RotateToEnd (m_masterNode);

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



bool dgDynamicBody::IsInEquilibrium () const
{
	dgFloat32 invMassMag2 = m_invMass[3] * m_invMass[3];
	if (m_equilibrium) {
		dgVector error (m_accel - m_prevExternalForce);
		dgFloat32 errMag2 = (error % error) * invMassMag2;
		if (errMag2 < DG_ErrTolerance2) {
			error = m_alpha - m_prevExternalTorque;
			errMag2 = (error % error) * invMassMag2;
			if (errMag2 < DG_ErrTolerance2) {
				errMag2 = (m_netForce % m_netForce) * invMassMag2;
				if (errMag2 < DG_ErrTolerance2) {
					errMag2 = (m_netTorque % m_netTorque) * invMassMag2;
					if (errMag2 < DG_ErrTolerance2) {
						errMag2 = m_veloc % m_veloc;
						if (errMag2 < DG_ErrTolerance2) {
							errMag2 = m_omega % m_omega;
							if (errMag2 < DG_ErrTolerance2) {
								return true;
							}
						}
					}
				}
			}
		}
	}

	return false;
}


void dgDynamicBody::CalcInvInertiaMatrix ()
{
	dgAssert (m_invWorldInertiaMatrix[0][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[1][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[2][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[3][3] == dgFloat32 (1.0f));

	m_invWorldInertiaMatrix[0][0] = m_invMass[0] * m_matrix[0][0];
	m_invWorldInertiaMatrix[0][1] = m_invMass[1] * m_matrix[1][0];
	m_invWorldInertiaMatrix[0][2] = m_invMass[2] * m_matrix[2][0];

	m_invWorldInertiaMatrix[1][0] = m_invMass[0] * m_matrix[0][1];
	m_invWorldInertiaMatrix[1][1] = m_invMass[1] * m_matrix[1][1];
	m_invWorldInertiaMatrix[1][2] = m_invMass[2] * m_matrix[2][1];

	m_invWorldInertiaMatrix[2][0] = m_invMass[0] * m_matrix[0][2];
	m_invWorldInertiaMatrix[2][1] = m_invMass[1] * m_matrix[1][2];
	m_invWorldInertiaMatrix[2][2] = m_invMass[2] * m_matrix[2][2];
	m_invWorldInertiaMatrix = m_invWorldInertiaMatrix * m_matrix;

	m_invWorldInertiaMatrix[3][0] = dgFloat32 (0.0f);
	m_invWorldInertiaMatrix[3][1] = dgFloat32 (0.0f);
	m_invWorldInertiaMatrix[3][2] = dgFloat32 (0.0f);

	dgAssert (m_invWorldInertiaMatrix[0][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[1][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[2][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[3][3] == dgFloat32 (1.0f));
}

void dgDynamicBody::CalcInvInertiaMatrixSimd ()
{
	//	dgMatrix tmp;
	dgSimd tmp[4];
	dgSimd::Transpose4x4 (tmp[0], tmp[1], tmp[2], tmp[3], (dgSimd&)m_matrix[0], (dgSimd&)m_matrix[1], (dgSimd&)m_matrix[2], dgSimd(0.0f));

	tmp[0] = tmp[0] * dgSimd (m_invMass[0]);
	tmp[1] = tmp[1] * dgSimd (m_invMass[1]);
	tmp[2] = tmp[2] * dgSimd (m_invMass[2]);
	m_invWorldInertiaMatrix = ((dgMatrix&)tmp[0]).MultiplySimd(m_matrix);
	m_invWorldInertiaMatrix[3] = dgSimd(dgFloat32 (1.0f)).AndNot(dgSimd::m_triplexMask);

	dgAssert (m_invWorldInertiaMatrix[0][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[1][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[2][3] == dgFloat32 (0.0f));
	dgAssert (m_invWorldInertiaMatrix[3][3] == dgFloat32 (1.0f));
}

void dgDynamicBody::SetMassProperties (dgFloat32 mass, const dgCollisionInstance* const collision)
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

dgMatrix dgDynamicBody::CalculateInertiaMatrix () const
{
	dgMatrix tmpMatrix;

	tmpMatrix[0][0] = m_mass[0] * m_matrix[0][0];
	tmpMatrix[0][1] = m_mass[1] * m_matrix[1][0];
	tmpMatrix[0][2] = m_mass[2] * m_matrix[2][0];
	tmpMatrix[0][3] = dgFloat32 (0.0f);

	tmpMatrix[1][0] = m_mass[0] * m_matrix[0][1];
	tmpMatrix[1][1] = m_mass[1] * m_matrix[1][1];
	tmpMatrix[1][2] = m_mass[2] * m_matrix[2][1];
	tmpMatrix[1][3] = dgFloat32 (0.0f);

	tmpMatrix[2][0] = m_mass[0] * m_matrix[0][2];
	tmpMatrix[2][1] = m_mass[1] * m_matrix[1][2];
	tmpMatrix[2][2] = m_mass[2] * m_matrix[2][2];
	tmpMatrix[2][3] = dgFloat32 (0.0f);

	tmpMatrix[3][0] = dgFloat32 (0.0f);
	tmpMatrix[3][1] = dgFloat32 (0.0f);
	tmpMatrix[3][2] = dgFloat32 (0.0f);
	tmpMatrix[3][3] = dgFloat32 (1.0f);
	return tmpMatrix * m_matrix;
}

dgMatrix dgDynamicBody::CalculateInvInertiaMatrix () const
{
	dgMatrix tmpMatrix;

	tmpMatrix[0][0] = m_invMass[0] * m_matrix[0][0];
	tmpMatrix[0][1] = m_invMass[1] * m_matrix[1][0];
	tmpMatrix[0][2] = m_invMass[2] * m_matrix[2][0];
	tmpMatrix[0][3] = dgFloat32 (0.0f);

	tmpMatrix[1][0] = m_invMass[0] * m_matrix[0][1];
	tmpMatrix[1][1] = m_invMass[1] * m_matrix[1][1];
	tmpMatrix[1][2] = m_invMass[2] * m_matrix[2][1];
	tmpMatrix[1][3] = dgFloat32 (0.0f);

	tmpMatrix[2][0] = m_invMass[0] * m_matrix[0][2];
	tmpMatrix[2][1] = m_invMass[1] * m_matrix[1][2];
	tmpMatrix[2][2] = m_invMass[2] * m_matrix[2][2];
	tmpMatrix[2][3] = dgFloat32 (0.0f);

	tmpMatrix[3][0] = dgFloat32 (0.0f);
	tmpMatrix[3][1] = dgFloat32 (0.0f);
	tmpMatrix[3][2] = dgFloat32 (0.0f);
	tmpMatrix[3][3] = dgFloat32 (1.0f);
	return tmpMatrix * m_matrix;
}

void dgDynamicBody::AddImpulse (const dgVector& pointDeltaVeloc, const dgVector& pointPosit)
{
	dgMatrix invInertia (CalculateInvInertiaMatrix());

	// get contact matrix
	dgMatrix tmp;
	//	dgVector globalContact (pointPosit - m_matrix.m_posit);
	dgVector globalContact (pointPosit - m_globalCentreOfMass);

	//globalContact[0] = dgFloat32 (0.0f);
	//globalContact[1] = dgFloat32 (0.0f);
	//globalContact[2] = dgFloat32 (0.0f);

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


	dgVector dv (changeOfMomentum.Scale (m_invMass.m_w));
	dgVector dw (invInertia.RotateVector (globalContact * changeOfMomentum));

	m_veloc += dv;
	m_omega += dw;

	m_sleeping	= false;
	m_equilibrium = false;
	Unfreeze ();
}

void dgDynamicBody::ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex)
{
	m_accel = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	m_alpha = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	if (m_applyExtForces) {
		m_applyExtForces(*this, timestep, threadIndex);
	}
}



void dgDynamicBody::ApplyImpulsePair (const dgVector& linearImpulseIn, const dgVector& angularImpulseIn)
{
	dgMatrix inertia (CalculateInertiaMatrix());
	dgMatrix invInertia (CalculateInvInertiaMatrix());

	dgVector linearImpulse (m_veloc.Scale (m_mass.m_w) + linearImpulseIn);
	dgVector angularImpulse (inertia.RotateVector (m_omega) + angularImpulseIn);

	m_veloc = linearImpulse.Scale(m_invMass.m_w);
	m_omega = invInertia.RotateVector(angularImpulse);

	m_sleeping	= false;
	m_equilibrium = false;
	Unfreeze ();
}

void dgDynamicBody::ApplyImpulsesAtPoint (dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const impulseArray, const dgFloat32* const pointArray)
{
	dgInt32 stride = strideInBytes / sizeof (dgFloat32);

	dgMatrix inertia (CalculateInertiaMatrix());

	dgVector impulse (m_veloc.Scale (m_mass.m_w));
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
	m_veloc = impulse.Scale(m_invMass.m_w);
	m_omega = invInertia.RotateVector(angularImpulse);

	m_sleeping	= false;
	m_equilibrium = false;
	Unfreeze ();
}

void dgDynamicBody::InvalidateCache ()
{
	m_sleepingCounter = 0;
	m_prevExternalForce = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	m_prevExternalTorque = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgBody::InvalidateCache ();
}
