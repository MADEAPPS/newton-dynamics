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

#ifndef __ND_MULTIBODY_VEHICLE_MOTOR_H__
#define __ND_MULTIBODY_VEHICLE_MOTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndMultiBodyVehicle;

class ndMultiBodyVehicleMotor: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndMultiBodyVehicleMotor);
	D_NEWTON_API ndMultiBodyVehicleMotor(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel);

	bool GetStart() const;
	ndFloat32 GetRpm() const;

	D_NEWTON_API void SetStart(bool startkey);
	D_NEWTON_API void SetThrottle(ndFloat32 param);

	D_NEWTON_API void SetRpmLimits(ndFloat32 idle, ndFloat32 redLineRpm);

	D_NEWTON_API void SetFuelRate(ndFloat32 radPerSecondsStep);
	D_NEWTON_API void SetTorque(ndFloat32 torqueInNewtonMeters);

	private:
	void AlignMatrix();
	void DebugJoint(ndConstraintDebugCallback&) const {}
	void JacobianDerivative(ndConstraintDescritor& desc);
	void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc);

	protected:
	ndFloat32 m_omega;
	ndFloat32 m_maxOmega;
	ndFloat32 m_idleOmega;
	ndFloat32 m_throttle;
	ndFloat32 m_engineTorque;
	ndFloat32 m_fuelValveRate;
	ndMultiBodyVehicle* m_vehicelModel;
	bool m_startEngine;
	friend class ndMultiBodyVehicle;
	friend class ndMultiBodyVehicleGearBox;
};

inline bool ndMultiBodyVehicleMotor::GetStart() const
{
	return m_startEngine;
}

inline ndFloat32 ndMultiBodyVehicleMotor::GetRpm() const
{
	return m_omega * dRadPerSecToRpm;
}

#endif