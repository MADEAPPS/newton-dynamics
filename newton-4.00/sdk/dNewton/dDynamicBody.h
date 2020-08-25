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

#ifndef _D_DYNAMIC_BODY_H_
#define _D_DYNAMIC_BODY_H_

#include "dNewtonStdafx.h"
#include "dBody.h"

//#define DG_MAX_SPEED_ATT	dgFloat32(0.02f)
////#define DG_FREEZE_ACCEL	dgFloat32(0.1f)
//#define DG_FREEZE_ACCEL		dgFloat32(1.0f)
//#define DG_FREEZE_SPEED		dgFloat32(0.032f)
//
//#define DG_FREEZE_ACCEL2	(DG_FREEZE_ACCEL * DG_FREEZE_ACCEL)
//#define DG_FREEZE_SPEED2	(DG_FREEZE_SPEED * DG_FREEZE_SPEED)
//
//#define DG_FREEZE_MAG		DG_FREEZE_ACCEL
//#define DG_FREEZE_MAG2		(DG_FREEZE_MAG * DG_FREEZE_MAG)
//
//#define DG_ERR_TOLERANCE	dgFloat32(1.0e-2f)
//#define DG_ERR_TOLERANCE2	(DG_ERR_TOLERANCE * DG_ERR_TOLERANCE)
//
//class dgSkeletonContainer;

class dDynamicBody: public dBody 
{
	public:
	D_NEWTON_API dDynamicBody();
	D_NEWTON_API virtual ~dDynamicBody ();

	D_NEWTON_API virtual void ApplyExternalForces(dInt32 threadID, dFloat32 tiemstep);
};

/*
DG_MSC_VECTOR_ALIGNMENT
class dDynamicBodyAsymetric: public dDynamicBody
{
	public:
	dDynamicBodyAsymetric();
	dDynamicBodyAsymetric(dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionNode, dgDeserialize serializeCallback, void* const userData, dgInt32 revisionNumber);

	virtual void Serialize(const dgTree<dgInt32, const dgCollision*>& collisionRemapId, dgSerialize serializeCallback, void* const userData);

	virtual dgMatrix CalculateInertiaMatrix() const;
	virtual dgMatrix CalculateLocalInertiaMatrix() const;
	virtual dgMatrix CalculateInvInertiaMatrix() const;
	virtual void SetMassMatrix(dgFloat32 mass, const dgMatrix& inertia);
	virtual void IntegrateOpenLoopExternalForce(dgFloat32 timestep);

	dgMatrix m_principalAxis;
} DG_GCC_VECTOR_ALIGNMENT;



DG_INLINE const dgVector& dDynamicBody::GetForce() const
{
	return m_externalForce; 
}

DG_INLINE const dgVector& dDynamicBody::GetTorque() const
{
	return m_externalTorque;
}


DG_INLINE dgFloat32 dDynamicBody::GetLinearDamping () const
{
	return m_dampCoef.m_w / DG_MAX_SPEED_ATT;
}

DG_INLINE dgVector dDynamicBody::GetAngularDamping () const
{
	return dgVector (m_dampCoef.m_x / DG_MAX_SPEED_ATT,
					 m_dampCoef.m_y / DG_MAX_SPEED_ATT,
					 m_dampCoef.m_z / DG_MAX_SPEED_ATT, dgFloat32 (0.0f));
}

DG_INLINE void dDynamicBody::SetLinearDamping (dgFloat32 linearDamp)
{
	linearDamp = dgClamp (linearDamp, dgFloat32(0.0f), dgFloat32(1.0f));
	m_dampCoef.m_w = DG_MAX_SPEED_ATT * linearDamp;
	m_cachedTimeStep = dgFloat32(0.0f);
}

DG_INLINE void dDynamicBody::SetAngularDamping (const dgVector& angularDamp)
{
	dgFloat32 tmp = dgClamp (angularDamp.m_x, dgFloat32(0.0f), dgFloat32(1.0f));
	m_dampCoef.m_x = DG_MAX_SPEED_ATT * tmp;

	tmp = dgClamp (angularDamp.m_y, dgFloat32(0.0f), dgFloat32(1.0f));
	m_dampCoef.m_y = DG_MAX_SPEED_ATT * tmp;

	tmp = dgClamp (angularDamp.m_z, dgFloat32(0.0f), dgFloat32(1.0f));
	m_dampCoef.m_z = DG_MAX_SPEED_ATT * tmp;

	m_cachedTimeStep = dgFloat32(0.0f);
}

DG_INLINE void dDynamicBody::AddForce (const dgVector& force)
{
	SetForce (m_externalForce + force);
}

DG_INLINE void dDynamicBody::AddTorque (const dgVector& torque)
{
	SetTorque (torque + m_externalTorque);
}


DG_INLINE void dDynamicBody::SetForce (const dgVector& force)
{
	m_externalForce = force;
}

DG_INLINE void dDynamicBody::SetTorque (const dgVector& torque)
{
	m_externalTorque = torque;
}


DG_INLINE dgVector dDynamicBody::PredictLinearVelocity(dgFloat32 timestep) const
{
	return 	m_veloc + m_externalForce.Scale (timestep * m_invMass.m_w);
}

DG_INLINE dgVector dDynamicBody::PredictAngularVelocity(dgFloat32 timestep) const
{
	return m_omega + m_invWorldInertiaMatrix.RotateVector(m_externalTorque).Scale (timestep);
}


DG_INLINE dBody::OnApplyExtForceAndTorque dDynamicBody::GetExtForceAndTorqueCallback () const
{
	return m_applyExtForces;
}

DG_INLINE void dDynamicBody::SetExtForceAndTorqueCallback (OnApplyExtForceAndTorque callback)
{
	m_applyExtForces = callback;
}

DG_INLINE dgSkeletonContainer* dDynamicBody::GetSkeleton() const
{
	return m_skeleton;
}

DG_INLINE void dDynamicBody::SetSkeleton(dgSkeletonContainer* const skeleton)
{
	dgAssert (!(m_skeleton && skeleton));
	m_skeleton = skeleton;
}

DG_INLINE const dgVector& dDynamicBody::GetDampCoeffcient (dgFloat32 timestep)
{
	if (dgAbs(m_cachedTimeStep - timestep) > dgFloat32(1.0e-6f)) {
		m_cachedTimeStep = timestep;
		const dgFloat32 tau = dgFloat32(60.0f) * timestep;
		m_cachedDampCoef.m_x = dgPow(dgFloat32(1.0f) - m_dampCoef.m_x, tau);
		m_cachedDampCoef.m_y = dgPow(dgFloat32(1.0f) - m_dampCoef.m_y, tau);
		m_cachedDampCoef.m_z = dgPow(dgFloat32(1.0f) - m_dampCoef.m_z, tau);
		m_cachedDampCoef.m_w = dgPow(dgFloat32(1.0f) - m_dampCoef.m_w, tau);
	}
	return m_cachedDampCoef;
}
*/

#endif 


