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

#ifndef __D_MULTIBODY_VEHICLE_GEAR_BOX_H__
#define __D_MULTIBODY_VEHICLE_GEAR_BOX_H__

#include "ndNewtonStdafx.h"
#include "ndJointGear.h"

class ndMultiBodyVehicle;

class ndMultiBodyVehicleGearBox : public ndJointGear
{
	public: 
	ND_CLASS_RELECTION(ndMultiBodyVehicleGearBox);
	D_NEWTON_API ndMultiBodyVehicleGearBox(ndBodyKinematic* const motor, ndBodyKinematic* const differential, const ndMultiBodyVehicle* const chassis);

	void SetClutchTorque(dFloat32 torqueInNewtonMeters);

	protected:
	void DebugJoint(ndConstraintDebugCallback&) const {}
	void JacobianDerivative(ndConstraintDescritor& desc);

	const ndMultiBodyVehicle* m_chassis;
	dFloat32 m_clutchTorque;
};

inline void ndMultiBodyVehicleGearBox::SetClutchTorque(dFloat32 torqueInNewtonMeters)
{
	m_clutchTorque = dAbs(torqueInNewtonMeters);
}

#endif