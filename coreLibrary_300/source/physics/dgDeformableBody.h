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

#ifndef _DG_DEFORMABLE_BODY_H_
#define _DG_DEFORMABLE_BODY_H_

#include "dgPhysicsStdafx.h"

#include "dgBody.h"

DG_MSC_VECTOR_ALIGMENT
class dgDeformableBody: public dgBody
{
	public:
	dgDeformableBody();
	dgDeformableBody(dgWorld* const world, const dgTree<const dgCollision*, dgInt32>* const collisionNode, dgDeserialize serializeCallback, void* const userData);
	virtual ~dgDeformableBody();

	virtual void Serialize (const dgTree<dgInt32, const dgCollision*>* const collisionNode, dgSerialize serializeCallback, void* const userData);

	protected:
	virtual bool IsDeformable() const;
	virtual bool IsInEquilibrium  () const;

    virtual void SetMatrix(const dgMatrix& matrix);
	virtual void ApplyExtenalForces (dgFloat32 timestep, dgInt32 threadIndex);
	virtual void SetVelocity (const dgVector& velocity);

	virtual void SetMassMatrix (dgFloat32 mass, dgFloat32 Ix, dgFloat32 Iy, dgFloat32 Iz);
	virtual void SetMassProperties (dgFloat32 mass, const dgCollisionInstance* const collision);

	virtual const dgVector& GetForce() const;
	virtual const dgVector& GetTorque() const {return m_dummy;}
	virtual void AddDampingAcceleration() {}

	virtual void AddForce (const dgVector& force);
	virtual void SetForce (const dgVector& force);
	virtual void SetTorque (const dgVector& torque) {} 
	virtual void AddTorque (const dgVector& torque) {}

	virtual dgFloat32 GetLinearDamping () const {return dgFloat32 (0.0f);}
	virtual dgVector GetAngularDamping () const {return m_dummy;}
	virtual void SetLinearDamping (dgFloat32 linearDamp) {}
	virtual void SetAngularDamping (const dgVector& angularDamp) {}

	virtual const dgMatrix& GetInertiaMatrix () const {return dgGetZeroMatrix();}
	virtual dgMatrix CalculateInertiaMatrix () const {return dgGetZeroMatrix();}
	virtual dgMatrix CalculateInvInertiaMatrix () const {return dgGetZeroMatrix();}

	virtual void AttachCollision (dgCollisionInstance* const collision);

	virtual dgVector PredictLinearVelocity(dgFloat32 timestep) const {return m_veloc;}
	virtual dgVector PredictAngularVelocity(dgFloat32 timestep) const {return m_omega;}

	virtual void SetCollidable (bool state) {m_collidable = state;}
	virtual void AddImpulse (const dgVector& pointVeloc, const dgVector& pointPosit) {dgAssert(0);}
	virtual void ApplyImpulsePair (const dgVector& linearImpulse, const dgVector& angularImpulse) {dgAssert(0);}
	virtual void ApplyImpulsesAtPoint (dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const impulseArray, const dgFloat32* const pointArray) {dgAssert(0);}

	virtual void SetExtForceAndTorqueCallback (OnApplyExtForceAndTorque callback);
	virtual OnApplyExtForceAndTorque GetExtForceAndTorqueCallback () const;

	dgVector m_force;
	dgVector m_torque;
	OnApplyExtForceAndTorque m_applyExtForces;
	static dgVector m_dummy;

} DG_GCC_VECTOR_ALIGMENT;


inline bool dgDeformableBody::IsDeformable() const
{
	return true;
}


#endif 

