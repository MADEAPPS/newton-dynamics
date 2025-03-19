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

#ifndef __ND_MULTIBODY_VEHICLE_MOTOR_H__
#define __ND_MULTIBODY_VEHICLE_MOTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndMultiBodyVehicle;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicleMotor: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndMultiBodyVehicleMotor, ndJointBilateralConstraint)

	D_NEWTON_API ndMultiBodyVehicleMotor();
	D_NEWTON_API ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndBodyKinematic* const chassis);
	D_NEWTON_API ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel);

	D_NEWTON_API void SetVehicleOwner(ndMultiBodyVehicle* const vehicle);

	D_NEWTON_API ndFloat32 GetRpm() const;
	D_NEWTON_API void SetMaxRpm(ndFloat32 redLineRpm);
	D_NEWTON_API void SetOmegaAccel(ndFloat32 rpmStep);
	D_NEWTON_API void SetFrictionLoss(ndFloat32 newtonMeters);
	D_NEWTON_API void SetTorqueAndRpm(ndFloat32 newtonMeters, ndFloat32 rpm);
	void DebugJoint(ndConstraintDebugCallback&) const {}

	private:
	void AlignMatrix();
	void JacobianDerivative(ndConstraintDescritor& desc);
	ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc);

	protected:
	ndMultiBodyVehicle* m_vehicle;
	ndFloat32 m_omega;
	ndFloat32 m_maxOmega;
	ndFloat32 m_omegaStep;
	ndFloat32 m_targetOmega;
	ndFloat32 m_engineTorque;
	ndFloat32 m_internalFriction;

	friend class ndMultiBodyVehicle;
	friend class ndMultiBodyVehicleGearBox;
} D_GCC_NEWTON_CLASS_ALIGN_32;
#endif