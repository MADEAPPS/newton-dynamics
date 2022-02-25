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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleTorsionBar.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndMultiBodyVehicleTorsionBar)

ndMultiBodyVehicleTorsionBar::ndMultiBodyVehicleTorsionBar(const ndMultiBodyVehicle* const vehicle, ndBodyKinematic* const fixedbody)
	:ndJointBilateralConstraint(1, vehicle->m_chassis, fixedbody, dGetIdentityMatrix())
	,m_springK(ndFloat32 (10.0f))
	,m_damperC(ndFloat32(1.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_axleCount(0)
{
	const ndMatrix worldMatrix (vehicle->m_localFrame * vehicle->m_chassis->GetMatrix());
	CalculateLocalMatrix(worldMatrix, m_localMatrix0, m_localMatrix1);
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndMultiBodyVehicleTorsionBar::ndMultiBodyVehicleTorsionBar(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_springK(ndFloat32(10.0f))
	,m_damperC(ndFloat32(1.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_axleCount(0)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_springK = xmlGetFloat(xmlNode, "springK");
	m_damperC = xmlGetFloat(xmlNode, "damperC");
	m_springDamperRegularizer = xmlGetFloat(xmlNode, "springDamperRegularizer");
	m_axleCount = xmlGetInt(xmlNode, "axleCount");
}

void ndMultiBodyVehicleTorsionBar::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "springK", m_springK);
	xmlSaveParam(childNode, "damperC", m_damperC);
	xmlSaveParam(childNode, "springDamperRegularizer", m_springDamperRegularizer);
	xmlSaveParam(childNode, "axleCount", m_axleCount);
}

void ndMultiBodyVehicleTorsionBar::SetTorsionTorque(ndFloat32 springK, ndFloat32 damperC, ndFloat32 springDamperRegularizer)
{
	m_springK = dAbs(springK);
	m_damperC = dAbs(damperC);
	m_springDamperRegularizer = dClamp (springDamperRegularizer, ndFloat32 (0.001), ndFloat32(0.99f));
}

void ndMultiBodyVehicleTorsionBar::AddAxel(const ndBodyKinematic* const leftTire, const ndBodyKinematic* const rightTire)
{
	if (m_axleCount < ndInt32 (sizeof(m_axles) / sizeof(m_axles[0])))
	{
		m_axles[m_axleCount].m_axleAngle = ndFloat32(0.0f);
		m_axles[m_axleCount].m_leftTire = leftTire;
		m_axles[m_axleCount].m_rightTire = rightTire;
		m_axleCount++;
	}
}

void ndMultiBodyVehicleTorsionBar::JacobianDerivative(ndConstraintDescritor& desc)
{
	if (m_axleCount)
	{
		ndMatrix matrix0;
		ndMatrix matrix1;
	
		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);
	
		ndFloat32 angle = ndFloat32(0.0f);
		ndFloat32 omega = ndFloat32(0.0f);
		for (ndInt32 i = 0; i < m_axleCount; i++)
		{
			ndVector dir(m_axles[i].m_rightTire->GetMatrix().m_posit - m_axles[i].m_leftTire->GetMatrix().m_posit);
			dir = dir.Normalize();
			angle += matrix0.m_right.CrossProduct(dir).DotProduct(matrix0.m_front).GetScalar();
			omega += (angle - m_axles[i].m_axleAngle) * desc.m_invTimestep;
			m_axles[i].m_axleAngle = angle;
		}
		angle = angle / m_axleCount;
		omega = omega / m_axleCount;
		//dTrace(("%f\n", angle * dRadToDegree));
		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		ndFloat32 accel = -CalculateSpringDamperAcceleration(desc.m_timestep, 300.0f, angle, ndFloat32(10.0f), omega);
		SetMotorAcceleration(desc, accel);
		SetDiagonalRegularizer(desc, ndFloat32 (0.2f));
	}
}

