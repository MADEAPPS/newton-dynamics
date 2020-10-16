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
	D_COLLISION_API ndBodyPlayerCapsule(const dMatrix& localAxis, dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight);
	D_COLLISION_API virtual ~ndBodyPlayerCapsule();

	ndBodyPlayerCapsule* ndBodyPlayerCapsule::GetAsBodyPlayerCapsule();

	dFloat32 GetHeadingAngle() const;
	virtual void ApplyInputs(dFloat32 timestep);

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

	protected: 
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

inline void ndBodyPlayerCapsule::SetCollisionShape(const ndShapeInstance& shapeInstance)
{
	// ignore the changing collision shape;
}

inline void ndBodyPlayerCapsule::ApplyInputs(dFloat32 timestep)
{
}

inline dFloat32 ndBodyPlayerCapsule::GetHeadingAngle() const
{ 
	return m_headingAngle; 
}
#endif