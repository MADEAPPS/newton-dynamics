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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleTorsionBar.h"

#if 0
ndMultiBodyVehicleTorsionBar::ndMultiBodyVehicleTorsionBar()
	:ndJointBilateralConstraint()
	,m_axles()
	,m_springK(ndFloat32(10.0f))
	,m_damperC(ndFloat32(1.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
{
	m_maxDof = 1;
	//const ndBodyKinematic* const chassis = vehicle->m_chassis;
	//ndAssert(chassis);
	//const ndMatrix worldMatrix(vehicle->m_localFrame * chassis->GetMatrix());
	//CalculateLocalMatrix(worldMatrix, m_localMatrix0, m_localMatrix1);
	SetSolverModel(m_jointkinematicCloseLoop);
}


ndMultiBodyVehicleTorsionBar::ndMultiBodyVehicleTorsionBar(const ndMultiBodyVehicle* const vehicle, ndBodyKinematic* const fixedbody)
	:ndJointBilateralConstraint(1, vehicle->m_chassis->GetAsBodyKinematic(), fixedbody, ndGetIdentityMatrix())
	,m_axles()
	,m_springK(ndFloat32 (10.0f))
	,m_damperC(ndFloat32(1.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
{
	const ndBodyKinematic* const chassis = vehicle->m_chassis;
	ndAssert(chassis);
	const ndMatrix worldMatrix (vehicle->m_localFrame * chassis->GetMatrix());
	CalculateLocalMatrix(worldMatrix, m_localMatrix0, m_localMatrix1);
	SetSolverModel(m_jointkinematicCloseLoop);
}

void ndMultiBodyVehicleTorsionBar::SetTorsionTorque(ndFloat32 springK, ndFloat32 damperC, ndFloat32 springDamperRegularizer)
{
	m_springK = ndAbs(springK);
	m_damperC = ndAbs(damperC);
	m_springDamperRegularizer = ndClamp (springDamperRegularizer, ND_SPRING_DAMP_MIN_REG, ndFloat32(0.99f));
}

void ndMultiBodyVehicleTorsionBar::GetTorsionTorque(ndFloat32& springK, ndFloat32& damperC, ndFloat32& springDamperRegularizer) const
{
	springK = m_springK;
	damperC = m_damperC;
	springDamperRegularizer = m_springDamperRegularizer;
}

void ndMultiBodyVehicleTorsionBar::AddAxel(const ndBodyKinematic* const leftTire, const ndBodyKinematic* const rightTire)
{
	if (m_axles.GetCount() < m_axles.GetCapacity())
	{
		ndAxles axle;
		axle.m_axleAngle = ndFloat32(0.0f);
		axle.m_leftTire = leftTire;
		axle.m_rightTire = rightTire;
		m_axles.PushBack(axle);
	}
}

const ndFixSizeArray<ndMultiBodyVehicleTorsionBar::ndAxles, 2>& ndMultiBodyVehicleTorsionBar::GetAxels() const
{
	return m_axles;
}

void ndMultiBodyVehicleTorsionBar::JacobianDerivative(ndConstraintDescritor& desc)
{
	if (m_axles.GetCount())
	{
		ndMatrix matrix0;
		ndMatrix matrix1;
	
		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);
	
		ndFloat32 angle = ndFloat32(0.0f);
		ndFloat32 omega = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < m_axles.GetCount(); ++i)
		{
			ndVector dir(m_axles[i].m_rightTire->GetMatrix().m_posit - m_axles[i].m_leftTire->GetMatrix().m_posit);
			dir = dir.Normalize();
			angle += matrix0.m_right.CrossProduct(dir).DotProduct(matrix0.m_front).GetScalar();
			omega += (angle - m_axles[i].m_axleAngle) * desc.m_invTimestep;
			m_axles[i].m_axleAngle = angle;
		}
		angle = angle / (ndFloat32)m_axles.GetCount();
		omega = omega / (ndFloat32)m_axles.GetCount();
		//dTrace(("%f\n", angle * dRadToDegree));
		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		ndFloat32 accel = -CalculateSpringDamperAcceleration(desc.m_timestep, 300.0f, angle, ndFloat32(10.0f), omega);
		SetMotorAcceleration(desc, accel);
		SetDiagonalRegularizer(desc, ndFloat32 (0.2f));
	}
}

#endif