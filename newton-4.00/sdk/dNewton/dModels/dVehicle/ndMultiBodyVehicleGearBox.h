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

#ifndef __ND_MULTIBODY_VEHICLE_GEAR_BOX_H__
#define __ND_MULTIBODY_VEHICLE_GEAR_BOX_H__

#include "ndNewtonStdafx.h"
#include "ndJointGear.h"

class ndMultiBodyVehicle;

class ndMultiBodyVehicleGearBox : public ndJointGear
{
	public: 
	D_CLASS_REFLECTION(ndMultiBodyVehicleGearBox);
	D_NEWTON_API ndMultiBodyVehicleGearBox(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndMultiBodyVehicleGearBox(ndBodyKinematic* const motor, ndBodyKinematic* const differential, ndMultiBodyVehicle* const chassis);

	D_NEWTON_API void SetIdleOmega(ndFloat32 rpm);
	D_NEWTON_API void SetClutchTorque(ndFloat32 torqueInNewtonMeters);
	D_NEWTON_API void SetInternalLosesTorque(ndFloat32 torqueInNewtonMeters);

	protected:
	void DebugJoint(ndConstraintDebugCallback&) const {}
	void JacobianDerivative(ndConstraintDescritor& desc);
	void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndMultiBodyVehicle* m_chassis;
	ndFloat32 m_idleOmega;
	ndFloat32 m_clutchTorque;
	ndFloat32 m_driveTrainResistanceTorque;
	friend class ndMultiBodyVehicle;
};

#endif