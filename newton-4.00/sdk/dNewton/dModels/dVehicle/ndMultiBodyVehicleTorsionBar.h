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

#ifndef __ND_MULTIBODY_VEHICLE_TORSION_BAR_H__
#define __ND_MULTIBODY_VEHICLE_TORSION_BAR_H__

#include "ndNewtonStdafx.h"
#include "dJoints/ndJointGear.h"

class ndMultiBodyVehicle;

#if 0
D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicleTorsionBar : public ndJointBilateralConstraint
{
	public:
	class ndAxles
	{
		public:
		ndAxles()
			:m_leftTire(nullptr)
			,m_rightTire(nullptr)
			,m_axleAngle(ndFloat32(0.0f))
		{
		}
		const ndBodyKinematic* m_leftTire;
		const ndBodyKinematic* m_rightTire;
		ndFloat32 m_axleAngle;
	};

	D_CLASS_REFLECTION(ndMultiBodyVehicleTorsionBar, ndJointBilateralConstraint)
	D_NEWTON_API ndMultiBodyVehicleTorsionBar();
	D_NEWTON_API ndMultiBodyVehicleTorsionBar(const ndMultiBodyVehicle* const chassis, ndBodyKinematic* const fixedbody);

	D_NEWTON_API const ndFixSizeArray<ndAxles, 2>& GetAxels() const;
	D_NEWTON_API void AddAxel(const ndBodyKinematic* const leftTire, const ndBodyKinematic* const rightTire);

	D_NEWTON_API void SetTorsionTorque(ndFloat32 springK, ndFloat32 damperC, ndFloat32 springDamperRegularizer);
	D_NEWTON_API void GetTorsionTorque(ndFloat32& springK, ndFloat32& damperC, ndFloat32& springDamperRegularizer) const;
	void DebugJoint(ndConstraintDebugCallback&) const {}

	protected:
	void JacobianDerivative(ndConstraintDescritor& desc);

	ndFixSizeArray<ndAxles, 2> m_axles;
	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_springDamperRegularizer;
	friend class ndMultiBodyVehicle;

} D_GCC_NEWTON_CLASS_ALIGN_32;
#endif

#endif