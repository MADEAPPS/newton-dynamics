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

#ifndef __ND_MULTIBODY_VEHICLE_DIFFERENTIAL_H__
#define __ND_MULTIBODY_VEHICLE_DIFFERENTIAL_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_MINIMUM_SLIP_OMEGA ndFloat32 (2.0f)

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicleDifferential : public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndMultiBodyVehicleDifferential, ndJointBilateralConstraint)

	D_NEWTON_API ndMultiBodyVehicleDifferential();
	D_NEWTON_API ndMultiBodyVehicleDifferential(ndBodyKinematic* const differential, ndBodyKinematic* const chassis, ndFloat32 slipOmegaLock);

	D_NEWTON_API ndFloat32 GetSlipOmega() const;
	D_NEWTON_API void SetSlipOmega(ndFloat32 speed);

	protected:
	void AlignMatrix();
	void DebugJoint(ndConstraintDebugCallback&) const override {}
	void JacobianDerivative(ndConstraintDescritor& desc) override;

	ndFloat32 m_limitedSlipOmega;

	friend class ndMultiBodyVehicle;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif