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

#ifndef __ND_BODY_DYNAMIC_BODY_H__
#define __ND_BODY_DYNAMIC_BODY_H__

#include "ndNewtonStdafx.h"

#define D_MAX_SPEED_ATT	ndFloat32(0.02f)
//#define D_FREEZE_ACCEL	ndFloat32(0.1f)
#define D_FREEZE_ACCEL		ndFloat32(1.0f)
#define D_FREEZE_SPEED		ndFloat32(0.032f)

#define D_FREEZE_ACCEL2		(D_FREEZE_ACCEL * D_FREEZE_ACCEL)
#define D_FREEZE_SPEED2		(D_FREEZE_SPEED * D_FREEZE_SPEED)

#define D_FREEZE_MAG		D_FREEZE_ACCEL
#define D_FREEZE_MAG2		(D_FREEZE_MAG * D_FREEZE_MAG)

#define D_ERR_TOLERANCE		ndFloat32(1.0e-2f)
#define D_ERR_TOLERANCE2	(D_ERR_TOLERANCE * D_ERR_TOLERANCE)

D_MSV_NEWTON_ALIGN_32
class ndBodyDynamic: public ndBodyKinematic
{
	public:
	D_CLASS_REFLECTION(ndBodyDynamic);
	D_NEWTON_API ndBodyDynamic();
	D_NEWTON_API ndBodyDynamic(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API virtual ~ndBodyDynamic ();

	D_NEWTON_API virtual ndBodyDynamic* GetAsBodyDynamic() { return this; }
	D_NEWTON_API virtual void ApplyExternalForces(ndInt32 threadIndex, ndFloat32 timestep);
	D_NEWTON_API virtual void AddDampingAcceleration(ndFloat32 timestep);
	D_NEWTON_API virtual void IntegrateVelocity(ndFloat32 timestep);

	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	D_NEWTON_API void SetForce(const ndVector& force);
	D_NEWTON_API void SetTorque(const ndVector& torque);

	D_NEWTON_API void AddImpulse(const ndVector& pointVeloc, const ndVector& pointPosit, ndFloat32 timestep);
	D_NEWTON_API void ApplyImpulsePair(const ndVector& linearImpulse, const ndVector& angularImpulse, ndFloat32 timestep);
	D_NEWTON_API void ApplyImpulsesAtPoint(ndInt32 count, const ndVector* const impulseArray, const ndVector* const pointArray, ndFloat32 timestep);

	D_NEWTON_API ndFloat32 GetLinearDamping() const;
	D_NEWTON_API void SetLinearDamping(ndFloat32 linearDamp);

	D_NEWTON_API ndVector GetAngularDamping() const;
	D_NEWTON_API void SetAngularDamping(const ndVector& angularDamp);

	virtual ndVector GetForce() const;
	virtual ndVector GetTorque() const;
	
	private:
	void SaveExternalForces();
	D_NEWTON_API virtual void IntegrateGyroSubstep(const ndVector& timestep);
	D_NEWTON_API virtual ndJacobian IntegrateForceAndToque(const ndVector& force, const ndVector& torque, const ndVector& timestep) const;

	ndVector m_externalForce;
	ndVector m_externalTorque;
	ndVector m_impulseForce;
	ndVector m_impulseTorque;
	ndVector m_savedExternalForce;
	ndVector m_savedExternalTorque;
	ndVector m_dampCoef;
	ndVector m_cachedDampCoef;
	ndFloat32 m_cachedTimeStep;

	friend class ndDynamicsUpdate;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateOpencl;
} D_GCC_NEWTON_ALIGN_32 ;

class ndBodySentinel : public ndBodyDynamic
{
	ndBodySentinel* GetAsBodySentinel() { return this; }
};

inline ndVector ndBodyDynamic::GetForce() const
{
	return m_externalForce;
}

inline ndVector ndBodyDynamic::GetTorque() const
{
	return m_externalTorque;
}

inline void ndBodyDynamic::SaveExternalForces()
{
	m_savedExternalForce = m_externalForce;
	m_savedExternalTorque = m_externalTorque;
}

#endif 


