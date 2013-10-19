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
	,m_accel(dgFloat32 (0.0))
	,m_alpha(dgFloat32 (0.0))
	,m_prevExternalForce(dgFloat32 (0.0))
	,m_prevExternalTorque(dgFloat32 (0.0))
	,m_dampCoef(dgFloat32 (0.0))
	,m_aparentMass(dgFloat32 (0.0))
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
	,m_accel(dgFloat32 (0.0))
	,m_alpha(dgFloat32 (0.0))
	,m_prevExternalForce(dgFloat32 (0.0))
	,m_prevExternalTorque(dgFloat32 (0.0))
	,m_dampCoef(dgFloat32 (0.0))
	,m_aparentMass(dgFloat32 (0.0))
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


void dgDynamicBody::SetMatrixIgnoreSleep(const dgMatrix& matrix)
{
	dgBody::SetMatrixIgnoreSleep(matrix);
	m_prevExternalForce = dgVector (dgFloat32 (0.0f));
	m_prevExternalTorque = dgVector (dgFloat32 (0.0f));
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




void dgDynamicBody::ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex)
{
	m_accel = dgVector (dgFloat32 (0.0f));
	m_alpha = dgVector (dgFloat32 (0.0f));
	if (m_applyExtForces) {
		m_applyExtForces(*this, timestep, threadIndex);
	}


#if 0
	#if 1
		static FILE* file = fopen ("replay.bin", "wb");
		if (file) {
			fwrite (&m_accel, sizeof (dgVector), 1, file);
			fwrite (&m_alpha, sizeof (dgVector), 1, file);
			fwrite (&m_veloc, sizeof (dgVector), 1, file);
			fwrite (&m_omega, sizeof (dgVector), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("replay.bin", "rb");
		if (file) {
			fread (&m_accel, sizeof (dgVector), 1, file);
			fread (&m_alpha, sizeof (dgVector), 1, file);
			fread (&m_veloc, sizeof (dgVector), 1, file);
			fread (&m_omega, sizeof (dgVector), 1, file);
		}
	#endif
#endif

//_ASSERTE ((m_accel.m_y == -100.0f) || (m_accel.m_y == 0.0f)); 
}



void dgDynamicBody::InvalidateCache ()
{
	m_sleepingCounter = 0;
	m_prevExternalForce = dgVector (dgFloat32 (0.0f));
	m_prevExternalTorque = dgVector (dgFloat32 (0.0f));
	dgBody::InvalidateCache ();
}
