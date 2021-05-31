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

#ifndef __D_MULTIBODY_VEHICLE_DIFFERENTIAL_H__
#define __D_MULTIBODY_VEHICLE_DIFFERENTIAL_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_MINIMUM_SLIP_OMEGA dFloat32 (10.0f)

class ndMultiBodyVehicleDifferential : public ndJointBilateralConstraint
{
	public:
	ND_CLASS_RELECTION(ndMultiBodyVehicleDifferential);
	D_NEWTON_API ndMultiBodyVehicleDifferential(ndBodyKinematic* const differential, ndBodyKinematic* const chassis);

	protected:
	void AlignMatrix();
	void JacobianDerivative(ndConstraintDescritor& desc);
	void DebugJoint(ndConstraintDebugCallback&) const {}

	dFloat32 GetSlipOmega() const;
	void SetSlipOmega(dFloat32 speed);

	friend class ndMultiBodyVehicle;
	dFloat32 m_limitedSlipOmega;
};

inline dFloat32 ndMultiBodyVehicleDifferential::GetSlipOmega() const
{
	return m_limitedSlipOmega;
}

inline void ndMultiBodyVehicleDifferential::SetSlipOmega(dFloat32 omega)
{
	m_limitedSlipOmega = dMax(D_MINIMUM_SLIP_OMEGA, dAbs(omega));
}

#endif