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

#ifndef __D_MULTIBODY_VEHICLE_ROTOR_H__
#define __D_MULTIBODY_VEHICLE_ROTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndMultiBodyVehicleRotor: public ndJointBilateralConstraint
{
	public:
	ND_JOINT_RELECTION(ndMultiBodyVehicleRotor);
	ndMultiBodyVehicleRotor(ndBodyKinematic* const motor, ndWorld* const world);

	dFloat32 GetRpm() const;
	dFloat32 GetMaxRpm() const;
	bool GetStart() const;

	D_NEWTON_API void SetStart(bool startkey);
	D_NEWTON_API void SetThrottle(dFloat32 param);
	D_NEWTON_API void SetMaxRpm(dFloat32 maxRpm);
	D_NEWTON_API void SetIdleRpm(dFloat32 idleRpm);
	D_NEWTON_API void SetEngineTorque(dFloat32 torque);
	D_NEWTON_API void SetGasValve(dFloat32 radPerSeconds);

	private:
	void JacobianDerivative(ndConstraintDescritor& desc);
	dFloat32 CalculateAcceleration(ndConstraintDescritor& desc);

	dFloat32 m_omega;
	dFloat32 m_maxOmega;
	dFloat32 m_idleOmega;
	dFloat32 m_throttle;
	dFloat32 m_gasValve;
	dFloat32 m_engineTorque;
	bool m_startEngine;
	friend class ndJointVehicleMotorGearBox;
};

inline dFloat32 ndMultiBodyVehicleRotor::GetRpm() const
{
	return m_omega * dFloat32 (9.55f);
}

inline dFloat32 ndMultiBodyVehicleRotor::GetMaxRpm() const
{
	return m_maxOmega * dFloat32(9.55f);
}

inline bool ndMultiBodyVehicleRotor::GetStart() const
{
	return m_startEngine;
}

#endif