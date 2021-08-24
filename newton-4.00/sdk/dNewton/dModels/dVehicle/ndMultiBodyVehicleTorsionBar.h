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

#ifndef __D_MULTIBODY_VEHICLE_TORSION_BAR_H__
#define __D_MULTIBODY_VEHICLE_TORSION_BAR_H__

#include "ndNewtonStdafx.h"
#include "ndJointGear.h"

class ndMultiBodyVehicle;

class ndMultiBodyVehicleTorsionBar : public ndJointBilateralConstraint
{
	public: 
	D_CLASS_REFLECTION(ndMultiBodyVehicleTorsionBar);
	D_NEWTON_API ndMultiBodyVehicleTorsionBar(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndMultiBodyVehicleTorsionBar(const ndMultiBodyVehicle* const chassis, ndBodyDynamic* const fixedbody);

	D_NEWTON_API void AddAxel(const ndBodyKinematic* const leftTire, const ndBodyKinematic* const rightTire);
	D_NEWTON_API void SetTorsionTorque(dFloat32 springK, dFloat32 damperC, dFloat32 springDamperRegularizer);

	protected:
	class ndAxles
	{
		public:
		ndAxles()
			:m_leftTire(nullptr)
			,m_rightTire(nullptr)
			,m_axleAngle(dFloat32(0.0f))
		{
		}
		const ndBodyKinematic* m_leftTire;
		const ndBodyKinematic* m_rightTire;
		dFloat32 m_axleAngle;
	};

	void DebugJoint(ndConstraintDebugCallback&) const {}
	void JacobianDerivative(ndConstraintDescritor& desc);
	void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	ndAxles m_axles[2];
	dFloat32 m_springK;
	dFloat32 m_damperC;
	dFloat32 m_springDamperRegularizer;
	dInt32 m_axleCount;
	friend class ndMultiBodyVehicle;
};

#endif