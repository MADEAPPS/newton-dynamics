/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
#include "ndBodyKinematicBase.h"

class ndBodyPlayerCapsuleContactSolver;
class ndBodyPlayerCapsuleImpulseSolver;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndBodyPlayerCapsule : public ndBodyKinematicBase
{
	public:
	D_CLASS_REFLECTION(ndBodyPlayerCapsule, ndBodyKinematicBase)

	D_COLLISION_API ndBodyPlayerCapsule();
	D_COLLISION_API ndBodyPlayerCapsule(const ndMatrix& localAxis, ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight);
	D_COLLISION_API virtual ~ndBodyPlayerCapsule();

	D_COLLISION_API ndBodyPlayerCapsule* GetAsBodyPlayerCapsule() override;

	D_COLLISION_API ndFloat32 GetForwardSpeed() const;
	D_COLLISION_API void SetForwardSpeed(ndFloat32 speed);

	D_COLLISION_API ndFloat32 GetLateralSpeed() const;
	D_COLLISION_API void SetLateralSpeed(ndFloat32 speed);

	D_COLLISION_API ndFloat32 GetHeadingAngle() const;
	D_COLLISION_API void SetHeadingAngle(ndFloat32 angle);

	D_COLLISION_API bool IsOnFloor() const;

	D_COLLISION_API virtual void ApplyInputs(ndFloat32 timestep);
	D_COLLISION_API virtual ndFloat32 ContactFrictionCallback(const ndVector& position, const ndVector& normal, ndInt32 contactId, const ndBodyKinematic* const otherbody) const;

	private:
	enum dCollisionState
	{
		m_colliding,
		m_freeMovement,
		m_deepPenetration,
	};

	D_COLLISION_API virtual void SpecialUpdate(ndFloat32 timestep) override;
	D_COLLISION_API virtual void IntegrateExternalForce(ndFloat32 timestep) override;
	D_COLLISION_API void IntegrateVelocity(ndFloat32 timestep) override;
	D_COLLISION_API virtual void SetCollisionShape(const ndShapeInstance& shapeInstance) override;
	D_COLLISION_API void Init(const ndMatrix& localAxis, ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight);

	void UpdatePlayerStatus(ndBodyPlayerCapsuleContactSolver& contactSolver);
	void ResolveStep(ndBodyPlayerCapsuleContactSolver& contactSolver, ndFloat32 timestep);
	void ResolveCollision(ndBodyPlayerCapsuleContactSolver& contactSolver, ndFloat32 timestep);
	ndFloat32 PredictTimestep(ndBodyPlayerCapsuleContactSolver& contactSolver, ndFloat32 timestep);
	dCollisionState TestPredictCollision(const ndBodyPlayerCapsuleContactSolver& contactSolver, const ndVector& veloc) const;
	void ResolveInterpenetrations(ndBodyPlayerCapsuleContactSolver& contactSolver, ndBodyPlayerCapsuleImpulseSolver& impulseSolver);

	protected: 
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
	
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif