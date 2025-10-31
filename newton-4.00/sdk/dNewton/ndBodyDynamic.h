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

#ifndef __ND_BODY_DYNAMIC_BODY_H__
#define __ND_BODY_DYNAMIC_BODY_H__

#include "ndNewtonStdafx.h"

#define D_MAX_SPEED_ATT		ndFloat32(0.02f)
#define D_FREEZE_ACCEL		ndFloat32(1.0f)
#define D_FREEZE_SPEED		ndFloat32(0.032f)

#define D_FREEZE_ACCEL2		(D_FREEZE_ACCEL * D_FREEZE_ACCEL)
#define D_FREEZE_SPEED2		(D_FREEZE_SPEED * D_FREEZE_SPEED)

#define D_FREEZE_MAG		D_FREEZE_ACCEL
#define D_FREEZE_MAG2		(D_FREEZE_MAG * D_FREEZE_MAG)

#define D_ERR_TOLERANCE		ndFloat32(1.0e-2f)
#define D_ERR_TOLERANCE2	(D_ERR_TOLERANCE * D_ERR_TOLERANCE)

class ndModel;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndBodyDynamic: public ndBodyKinematic
{
	public:
	D_CLASS_REFLECTION(ndBodyDynamic, ndBodyKinematic)
	D_NEWTON_API ndBodyDynamic();
	D_NEWTON_API ndBodyDynamic(const ndBodyDynamic& src);
	D_NEWTON_API virtual ~ndBodyDynamic ();

	D_NEWTON_API virtual ndBodyDynamic* GetAsBodyDynamic() override { return this; }
	D_NEWTON_API virtual void ApplyExternalForces(ndInt32 threadIndex, ndFloat32 timestep) override;
	D_NEWTON_API virtual void AddDampingAcceleration(ndFloat32 timestep) override;
	D_NEWTON_API virtual void IntegrateVelocity(ndFloat32 timestep) override;
	D_NEWTON_API virtual void InitSurrogateBody(ndBodyKinematic* const surrogate) const override;

	D_NEWTON_API void SetForce(const ndVector& force) override;
	D_NEWTON_API void SetTorque(const ndVector& torque) override;

	D_NEWTON_API void AddImpulse(const ndVector& pointVeloc, const ndVector& pointPosit, ndFloat32 timestep) override;
	D_NEWTON_API void ApplyImpulsePair(const ndVector& linearImpulse, const ndVector& angularImpulse, ndFloat32 timestep) override;
	D_NEWTON_API void ApplyImpulsesAtPoint(ndInt32 count, const ndVector* const impulseArray, const ndVector* const pointArray, ndFloat32 timestep) override;

	D_NEWTON_API ndFloat32 GetLinearDamping() const override;
	D_NEWTON_API void SetLinearDamping(ndFloat32 linearDamp) override;

	D_NEWTON_API ndVector GetCachedDamping() const override;
	D_NEWTON_API ndVector GetAngularDamping() const override;
	D_NEWTON_API void SetAngularDamping(const ndVector& angularDamp) override;

	D_NEWTON_API ndFloat32 GetSleepAccel() const;
	D_NEWTON_API void SetSleepAccel(ndFloat32 accelMag2);

	D_NEWTON_API ndVector GetForce() const override;
	D_NEWTON_API ndVector GetTorque() const override;

	D_NEWTON_API ndModel* GetModel() const;
	D_NEWTON_API void SetModel(ndModel* const model);
	
	private:
	void SaveExternalForces();
	virtual void SetAcceleration(const ndVector& accel, const ndVector& alpha) override;
	D_NEWTON_API virtual void IntegrateGyroSubstep(const ndVector& timestep) override;
	D_NEWTON_API virtual ndJacobian IntegrateForceAndToque(const ndVector& force, const ndVector& torque, const ndVector& timestep) const override;
	D_NEWTON_API virtual void EvaluateSleepState(ndFloat32 freezeSpeed2, ndFloat32 freezeAccel2) override;

	ndVector m_externalForce;
	ndVector m_externalTorque;
	ndVector m_impulseForce;
	ndVector m_impulseTorque;
	ndVector m_savedExternalForce;
	ndVector m_savedExternalTorque;
	ndVector m_dampCoef;
	ndVector m_cachedDampCoef;
	ndVector m_sleepAccelTest2;
	ndModel* m_model;
	ndFloat32 m_cachedTimeStep;

	static ndVector m_sleepAccelTestScale2;

	friend class ndDynamicsUpdate;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateSycl;
	friend class ndDynamicsUpdateCuda;
} D_GCC_NEWTON_CLASS_ALIGN_32 ;


#endif 


