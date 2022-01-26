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
	D_NEWTON_API ndMultiBodyVehicleMotor(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel);

	D_NEWTON_API ndFloat32 GetRpm() const;
	D_NEWTON_API void SetMaxRpm(ndFloat32 redLineRpm);
	D_NEWTON_API void SetOmegaAccel(ndFloat32 rpmStep);
	D_NEWTON_API void SetFrictionLose(ndFloat32 newtonMeters);
	D_NEWTON_API void SetTorqueAndRpm(ndFloat32 rpm, ndFloat32 newtonMeters);

	private:
	void AlignMatrix();
	void DebugJoint(ndConstraintDebugCallback&) const {}
	void JacobianDerivative(ndConstraintDescritor& desc);
	void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc);

	protected:
	ndFloat32 m_omega;
	ndFloat32 m_maxOmega;
	ndFloat32 m_omegaStep;
	ndFloat32 m_targetOmega;
	ndFloat32 m_engineTorque;
	ndFloat32 m_internalFriction;
	
	ndMultiBodyVehicle* m_vehicelModel;
	friend class ndMultiBodyVehicle;
	friend class ndMultiBodyVehicleGearBox;
};

#endif