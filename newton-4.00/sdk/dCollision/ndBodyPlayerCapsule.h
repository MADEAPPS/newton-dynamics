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

#ifndef __D_BODY_PLAYER_CAPSULE_H__
#define __D_BODY_PLAYER_CAPSULE_H__

#include "ndCollisionStdafx.h"
#include "ndBodyKinematic.h"

class ndBodyPlayerCapsuleContactSolver;
class ndBodyPlayerCapsuleImpulseSolver;

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsule : public ndBodyKinematic
{
	public:
	D_COLLISION_API ndBodyPlayerCapsule(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache);
	D_COLLISION_API ndBodyPlayerCapsule(const dMatrix& localAxis, dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight);
	D_COLLISION_API virtual ~ndBodyPlayerCapsule();

	ndBodyPlayerCapsule* GetAsBodyPlayerCapsule();

	dFloat32 GetForwardSpeed() const;
	void SetForwardSpeed(dFloat32 speed);

	dFloat32 GetLateralSpeed() const;
	void SetLateralSpeed(dFloat32 speed);

	dFloat32 GetHeadingAngle() const;
	void SetHeadingAngle(dFloat32 angle);

	bool IsOnFloor() const;

	virtual void ApplyInputs(dFloat32 timestep);
	virtual dFloat32 ContactFrictionCallback(const dVector& position, const dVector& normal, dInt32 contactId, const ndBodyKinematic* const otherbody) const;

	private:
	enum dCollisionState
	{
		m_colliding,
		m_freeMovement,
		m_deepPenetration,
	};

	D_COLLISION_API virtual void IntegrateExternalForce(dFloat32 timestep);
	virtual void SetCollisionShape(const ndShapeInstance& shapeInstance);
	
	void UpdatePlayerStatus(ndBodyPlayerCapsuleContactSolver& contactSolver);
	void ResolveStep(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep);
	void ResolveCollision(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep);
	dFloat32 PredictTimestep(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep);
	dCollisionState TestPredictCollision(const ndBodyPlayerCapsuleContactSolver& contactSolver, const dVector& veloc) const;
	void ResolveInterpenetrations(ndBodyPlayerCapsuleContactSolver& contactSolver, ndBodyPlayerCapsuleImpulseSolver& impulseSolver);

	void IntegrateVelocity(dFloat32 timestep);

	protected: 
	D_COLLISION_API void Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 nodeid, const dTree<dUnsigned32, const ndShape*>& shapesCache) const;

	dMatrix m_localFrame;
	dVector m_impulse;
	dFloat32 m_mass;
	dFloat32 m_invMass;
	dFloat32 m_headingAngle;
	dFloat32 m_forwardSpeed;
	dFloat32 m_lateralSpeed;
	dFloat32 m_stepHeight;
	dFloat32 m_contactPatch;
	dFloat32 m_height;
	dFloat32 m_radius;
	dFloat32 m_weistScale;
	dFloat32 m_crouchScale;
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

inline void ndBodyPlayerCapsule::ApplyInputs(dFloat32)
{
}

inline dFloat32 ndBodyPlayerCapsule::ContactFrictionCallback(const dVector&, const dVector&, dInt32, const ndBodyKinematic* const) const
{
	return dFloat32 (2.0f);
}

inline dFloat32 ndBodyPlayerCapsule::GetForwardSpeed() const 
{ 
	return -m_forwardSpeed; 
}

inline void ndBodyPlayerCapsule::SetForwardSpeed(dFloat32 speed) 
{ 
	m_forwardSpeed = -speed; 
}

inline dFloat32 ndBodyPlayerCapsule::GetLateralSpeed() const 
{ 
	return -m_lateralSpeed; 
}

inline void ndBodyPlayerCapsule::SetLateralSpeed(dFloat32 speed) 
{ 
	m_lateralSpeed = -speed; 
}

inline dFloat32 ndBodyPlayerCapsule::GetHeadingAngle() const
{
	return m_headingAngle;
}

inline void ndBodyPlayerCapsule::SetHeadingAngle(dFloat32 angle)
{ 
	//m_headingAngle = dClamp(angle, dFloat32(-dPi), dFloat32(dPi)); 
	const dFloat32 interpolation = dFloat32(0.3f);
	dFloat32 deltaAngle = AnglesAdd(angle, -m_headingAngle) * interpolation;
	dFloat32 headingAngle = AnglesAdd(m_headingAngle, deltaAngle);
	//dTrace(("%f %f %f\n", angle * dRadToDegree, m_headingAngle * dRadToDegree, headingAngle * dRadToDegree));
	m_headingAngle = headingAngle;
}

inline void ndBodyPlayerCapsule::IntegrateVelocity(dFloat32)
{
	m_residualVeloc = m_veloc;
	m_residualOmega = m_omega;
}

inline bool ndBodyPlayerCapsule::IsOnFloor() const 
{ 
	return m_isOnFloor; 
}

#endif