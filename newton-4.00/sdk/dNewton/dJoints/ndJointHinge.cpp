/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointHinge)

ndJointHinge::ndJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
{
}

ndJointHinge::ndJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotInChild)
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
{
	ndMatrix tmp;
	CalculateLocalMatrix(pinAndPivotInChild, m_localMatrix0, tmp);
	CalculateLocalMatrix(pinAndPivotInParent, tmp, m_localMatrix1);
}

ndJointHinge::ndJointHinge(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_angle = xmlGetFloat(xmlNode, "angle");
	m_omega = xmlGetFloat(xmlNode, "omega");
	m_springK = xmlGetFloat(xmlNode, "springK");
	m_damperC = xmlGetFloat(xmlNode, "damperC");
	m_minLimit = xmlGetFloat(xmlNode, "minLimit");
	m_maxLimit = xmlGetFloat(xmlNode, "maxLimit");
	m_springDamperRegularizer = xmlGetFloat(xmlNode, "springDamperRegularizer");
}

ndJointHinge::~ndJointHinge()
{
}

void ndJointHinge::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "angle", m_angle);
	xmlSaveParam(childNode, "omega", m_omega);
	xmlSaveParam(childNode, "springK", m_springK);
	xmlSaveParam(childNode, "damperC", m_damperC);
	xmlSaveParam(childNode, "minLimit", m_minLimit);
	xmlSaveParam(childNode, "maxLimit", m_maxLimit);
	xmlSaveParam(childNode, "springDamperRegularizer", m_springDamperRegularizer);
}

ndFloat32 ndJointHinge::GetAngle() const
{
	return m_angle;
}

ndFloat32 ndJointHinge::GetOmega() const
{
	return m_omega;
}

void ndJointHinge::SetLimits(ndFloat32 minLimit, ndFloat32 maxLimit)
{
	dAssert(minLimit <= 0.0f);
	dAssert(maxLimit >= 0.0f);
	m_minLimit = minLimit;
	m_maxLimit = maxLimit;

	// adding one extra dof, this makes the mass matrix ill conditioned, 
	// but it could work with the direct solver
	m_maxDof = ((maxLimit - minLimit) < ndFloat32(1.0e9f)) ? 7 : 6;
}

void ndJointHinge::GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit)
{
	minLimit = m_minLimit;
	maxLimit = m_maxLimit;
}

void ndJointHinge::SetAsSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_springK = dAbs(spring);
	m_damperC = dAbs(damper);
	m_springDamperRegularizer = dClamp(regularizer, ndFloat32(1.0e-2f), ndFloat32(0.99f));
}

void ndJointHinge::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	const ndInt32 subdiv = 8;
	const ndFloat32 radius = debugCallback.m_debugScale;
	ndVector arch[subdiv + 1];

	ndFloat32 deltaTwist = m_maxLimit - m_minLimit;
	if ((deltaTwist > ndFloat32(1.0e-3f)) && (deltaTwist < ndFloat32(2.0f) * ndPi))
	{
		ndMatrix pitchMatrix(matrix1);
		pitchMatrix.m_posit = matrix1.m_posit;

		ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f));

		ndFloat32 angleStep = dMin(deltaTwist, ndFloat32(2.0f * ndPi)) / subdiv;
		ndFloat32 angle0 = m_minLimit;

		ndVector color(ndFloat32(0.4f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		for (ndInt32 i = 0; i <= subdiv; i++)
		{
			arch[i] = pitchMatrix.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(pitchMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}

		for (ndInt32 i = 0; i < subdiv; i++)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}


void ndJointHinge::SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& )
{
	// add spring damper row
	AddAngularRowJacobian(desc, matrix0.m_front, -m_angle);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
}

ndJacobianPair ndJointHinge::GetPinJacobian() const
{
	ndMatrix matrix (GetLocalMatrix0() * GetBody0()->GetMatrix());
	ndJacobianPair pair;
	pair.m_jacobianM0.m_linear = ndVector::m_zero;
	pair.m_jacobianM1.m_linear = ndVector::m_zero;
	pair.m_jacobianM0.m_angular = matrix.m_front;
	pair.m_jacobianM1.m_angular = matrix.m_front * ndVector::m_negOne;
	return pair;
}

void ndJointHinge::ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	// two rows to restrict rotation around around the parent coordinate system
	const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);

	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);

	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	const ndFloat32 deltaAngle = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
	m_angle += deltaAngle;
	m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
}

ndFloat32 ndJointHinge::PenetrationOmega(ndFloat32 penetration) const
{
	ndFloat32 param = dClamp(penetration, ndFloat32(0.0f), D_MAX_HINGE_PENETRATION) / D_MAX_HINGE_PENETRATION;
	ndFloat32 omega = D_MAX_HINGE_RECOVERY_SPEED * param;
	return omega;
}

bool ndJointHinge::SubmitConstraintLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	bool ret = false;
	if ((m_minLimit > (ndFloat32(-1.0f) * ndDegreeToRad)) && (m_maxLimit < (ndFloat32(1.0f) * ndDegreeToRad)))
	{
		AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_angle);
	}
	else
	{
		const ndFloat32 angle = m_angle + m_omega * desc.m_timestep;
		if (angle < m_minLimit)
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_minLimit;
			const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationOmega(-penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetLowerFriction(desc, ndFloat32(0.0f));
			ret = dAbs(stopAccel) > ND_MAX_STOP_ACCEL;
		}
		else if (angle > m_maxLimit)
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_maxLimit;
			const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationOmega(penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, ndFloat32(0.0f));
			ret = dAbs(stopAccel) > ND_MAX_STOP_ACCEL;
		}
	}
	return ret;
}

void ndJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	bool hitLimit = SubmitConstraintLimits(desc, matrix0, matrix1);
	if (!hitLimit)
	{
		if ((m_springK > ndFloat32(0.0f)) || (m_damperC > ndFloat32(0.0f)))
		{
			// spring damper with limits
			SubmitSpringDamper(desc, matrix0, matrix1);
		}
	}
}


