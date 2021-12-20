/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_BODY_PLAYER_CAPSULE_H__
#define __ND_BODY_PLAYER_CAPSULE_H__

#include "ndCollisionStdafx.h"
#include "ndBodyKinematic.h"

class ndBodyPlayerCapsuleContactSolver;
class ndBodyPlayerCapsuleImpulseSolver;

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsule : public ndBodyKinematic
{
	public:
	D_CLASS_REFLECTION(ndBodyPlayerCapsule);
	D_COLLISION_API ndBodyPlayerCapsule(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_COLLISION_API ndBodyPlayerCapsule(const ndMatrix& localAxis, ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight);
	D_COLLISION_API virtual ~ndBodyPlayerCapsule();

	ndBodyPlayerCapsule* GetAsBodyPlayerCapsule();

	ndFloat32 GetForwardSpeed() const;
	void SetForwardSpeed(ndFloat32 speed);

	ndFloat32 GetLateralSpeed() const;
	void SetLateralSpeed(ndFloat32 speed);

	ndFloat32 GetHeadingAngle() const;
	void SetHeadingAngle(ndFloat32 angle);

	bool IsOnFloor() const;

	virtual void ApplyInputs(ndFloat32 timestep);
	virtual ndFloat32 ContactFrictionCallback(const ndVector& position, const ndVector& normal, ndInt32 contactId, const ndBodyKinematic* const otherbody) const;

	private:
	enum dCollisionState
	{
		m_colliding,
		m_freeMovement,
		m_deepPenetration,
	};

	virtual void IntegrateExternalForce(ndFloat32 timestep);
	D_COLLISION_API virtual void SpecialUpdate(ndFloat32 timestep);
	virtual void SetCollisionShape(const ndShapeInstance& shapeInstance);
	
	void UpdatePlayerStatus(ndBodyPlayerCapsuleContactSolver& contactSolver);
	void ResolveStep(ndBodyPlayerCapsuleContactSolver& contactSolver, ndFloat32 timestep);
	void ResolveCollision(ndBodyPlayerCapsuleContactSolver& contactSolver, ndFloat32 timestep);
	ndFloat32 PredictTimestep(ndBodyPlayerCapsuleContactSolver& contactSolver, ndFloat32 timestep);
	dCollisionState TestPredictCollision(const ndBodyPlayerCapsuleContactSolver& contactSolver, const ndVector& veloc) const;
	void ResolveInterpenetrations(ndBodyPlayerCapsuleContactSolver& contactSolver, ndBodyPlayerCapsuleImpulseSolver& impulseSolver);

	void IntegrateVelocity(ndFloat32 timestep);

	protected: 
	D_COLLISION_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndMatrix m_localFrame;
	ndVector m_impulse;
	ndFloat32 m_mass;
	ndFloat32 m_invMass;
	ndFloat32 m_headingAngle;
	ndFloat32 m_forwardSpeed;
	ndFloat32 m_lateralSpeed;
	ndFloat32 m_stepHeight;
	ndFloat32 m_contactPatch;
	ndFloat32 m_height;
	ndFloat32 m_radius;
	ndFloat32 m_weistScale;
	ndFloat32 m_crouchScale;
	bool m_isAirbone;
	bool m_isOnFloor;
	bool m_isCrouched;
} D_GCC_NEWTON_ALIGN_32;

inline ndBodyPlayerCapsule* ndBodyPlayerCapsule::GetAsBodyPlayerCapsule()
{ 
	return this; 
}

inline void ndBodyPlayerCapsule::SetCollisionShape(const ndShapeInstance&)
{
	// ignore the changing collision shape;
}

inline void ndBodyPlayerCapsule::ApplyInputs(ndFloat32)
{
}

inline ndFloat32 ndBodyPlayerCapsule::ContactFrictionCallback(const ndVector&, const ndVector&, ndInt32, const ndBodyKinematic* const) const
{
	return ndFloat32 (2.0f);
}

inline ndFloat32 ndBodyPlayerCapsule::GetForwardSpeed() const 
{ 
	return -m_forwardSpeed; 
}

inline void ndBodyPlayerCapsule::SetForwardSpeed(ndFloat32 speed) 
{ 
	m_forwardSpeed = -speed; 
}

inline ndFloat32 ndBodyPlayerCapsule::GetLateralSpeed() const 
{ 
	return -m_lateralSpeed; 
}

inline void ndBodyPlayerCapsule::SetLateralSpeed(ndFloat32 speed) 
{ 
	m_lateralSpeed = -speed; 
}

inline ndFloat32 ndBodyPlayerCapsule::GetHeadingAngle() const
{
	return m_headingAngle;
}

inline void ndBodyPlayerCapsule::SetHeadingAngle(ndFloat32 angle)
{ 
	//m_headingAngle = dClamp(angle, ndFloat32(-dPi), ndFloat32(dPi)); 
	const ndFloat32 interpolation = ndFloat32(0.3f);
	ndFloat32 deltaAngle = AnglesAdd(angle, -m_headingAngle) * interpolation;
	ndFloat32 headingAngle = AnglesAdd(m_headingAngle, deltaAngle);
	//dTrace(("%f %f %f\n", angle * dRadToDegree, m_headingAngle * dRadToDegree, headingAngle * dRadToDegree));
	m_headingAngle = headingAngle;
}

inline void ndBodyPlayerCapsule::IntegrateVelocity(ndFloat32)
{
	m_accel = ndVector::m_zero;
	m_alpha = ndVector::m_zero;
}

inline bool ndBodyPlayerCapsule::IsOnFloor() const 
{ 
	return m_isOnFloor; 
}

inline void ndBodyPlayerCapsule::IntegrateExternalForce(ndFloat32)
{
	// do nothing
}

#endif