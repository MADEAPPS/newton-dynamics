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

dgVector dgDynamicBody::m_equilibriumError2 (DG_ErrTolerance2);


dgDynamicBody::dgDynamicBody()
	:dgBody()
	,m_accel(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_alpha(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalForce(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalTorque(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_dampCoef(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_aparentMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_sleepingCounter(0)
	,m_isInDestructionArrayLRU(0)
	,m_applyExtForces(NULL)
{
	m_type = m_dynamicBody;
	m_rtti |= m_dynamicBodyRTTI;
	dgAssert ( dgInt32 (sizeof (dgDynamicBody) & 0x0f) == 0);
}

dgDynamicBody::dgDynamicBody (dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionCashe, dgDeserialize serializeCallback, void* const userData)
	:dgBody(world, collisionCashe, serializeCallback, userData)
	,m_accel(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_alpha(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalForce(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_prevExternalTorque(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_dampCoef(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_aparentMass(dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0), dgFloat32 (0.0))
	,m_sleepingCounter(0)
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

			dgVector force (gravityVector.Scale3 (-fluidDensity * volumeIntegral.m_w));
			dgVector torque (buoyanceCenter * force);

			dgFloat32 damp = GetMax (GetMin ((m_veloc % m_veloc) * dgFloat32 (100.0f) * fluidLinearViscousity, dgFloat32 (1.0f)), dgFloat32(dgFloat32 (10.0f)));
			force -= m_veloc.Scale3 (damp);

			//damp = (m_omega % m_omega) * dgFloat32 (10.0f) * fluidAngularViscousity;
			damp = GetMax (GetMin ((m_omega % m_omega) * dgFloat32 (1000.0f) * fluidAngularViscousity, dgFloat32(0.25f)), dgFloat32(2.0f));
			torque -= m_omega.Scale3 (damp);

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


bool dgDynamicBody::IsInEquilibrium () const
{
	if (m_equilibrium) {
		dgVector forceError (m_accel - m_prevExternalForce);
		dgVector torqueError (m_alpha - m_prevExternalTorque);
		dgVector mask0 ((forceError.DotProduct4(forceError) < m_equilibriumError2) & (torqueError.DotProduct4(torqueError) < m_equilibriumError2));
		if (mask0.GetSignMask()) {
			dgVector invMassMag2 (m_invMass[3] * m_invMass[3]);
			dgVector mask1 ((invMassMag2.CompProduct4 (m_netForce.DotProduct4(m_netForce)) < m_equilibriumError2) & (invMassMag2.CompProduct4 (m_netTorque.DotProduct4(m_netTorque)) < m_equilibriumError2));
			if (mask1.GetSignMask()) {
				dgVector mask2 ((m_veloc.DotProduct4(m_veloc) < m_equilibriumError2) & (m_omega.DotProduct4(m_omega) < m_equilibriumError2));
				return mask2.GetSignMask() ? true : false;
			}
		}
	}

	return false;
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


	dgVector dv (changeOfMomentum.Scale3 (m_invMass.m_w));
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

#if 0
	#if 0
		static FILE* file = fopen ("replay.bin", "wb");
		if (file) {
			fwrite (&m_accel, sizeof (dgVector), 1, file);
			fwrite (&m_alpha, sizeof (dgVector), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("replay.bin", "rb");
		if (file) {
			fread (&m_accel, sizeof (dgVector), 1, file);
			fread (&m_alpha, sizeof (dgVector), 1, file);
		}
	#endif
#endif

}



void dgDynamicBody::ApplyImpulsePair (const dgVector& linearImpulseIn, const dgVector& angularImpulseIn)
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

void dgDynamicBody::ApplyImpulsesAtPoint (dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const impulseArray, const dgFloat32* const pointArray)
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

void dgDynamicBody::InvalidateCache ()
{
	m_sleepingCounter = 0;
	m_prevExternalForce = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	m_prevExternalTorque = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgBody::InvalidateCache ();
}
