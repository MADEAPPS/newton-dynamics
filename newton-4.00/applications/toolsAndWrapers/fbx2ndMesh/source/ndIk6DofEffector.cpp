/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndIk6DofEffector.h"
#include "ndSkeletonContainer.h"

ndIk6DofEffector::ndIk6DofEffector()
	:ndJointBilateralConstraint()
	,m_targetFrame(ndGetIdentityMatrix())
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularMaxTorque(D_LCP_MAX_VALUE)
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearMaxForce(D_LCP_MAX_VALUE)
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_rotationType(m_disabled)
	,m_controlDofOptions(0xff)
{
	m_maxDof = 6;
}

ndIk6DofEffector::ndIk6DofEffector(const ndMatrix& pinAndPivotChild, const ndMatrix& pinAndPivotParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotChild, pinAndPivotParent)
	,m_targetFrame(pinAndPivotChild * pinAndPivotParent.OrthoInverse())
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularMaxTorque(D_LCP_MAX_VALUE)
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearMaxForce(D_LCP_MAX_VALUE)
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_rotationType(m_disabled)
	,m_controlDofOptions(0xff)
{
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndIk6DofEffector::~ndIk6DofEffector()
{
}

void ndIk6DofEffector::EnableAxisX(bool state)
{
	m_axisX = ndUnsigned8(state ? 1 : 0);
}

void ndIk6DofEffector::EnableAxisY(bool state)
{
	m_axisY = ndUnsigned8(state ? 1 : 0);
}

void ndIk6DofEffector::EnableAxisZ(bool state)
{
	m_axisZ = ndUnsigned8(state ? 1 : 0);
}

bool ndIk6DofEffector::GetAxisX() const
{
	return m_axisX ? true : false;
}

bool ndIk6DofEffector::GetAxisY() const
{
	return m_axisY ? true : false;
}

bool ndIk6DofEffector::GetAxisZ() const
{
	return m_axisZ ? true : false;
}

void ndIk6DofEffector::EnableRotationAxis(ndRotationType type)
{
	m_rotationType = type;
}

ndIk6DofEffector::ndRotationType ndIk6DofEffector::GetRotationAxis() const
{
	return m_rotationType;
}

ndMatrix ndIk6DofEffector::GetOffsetMatrix() const
{
	return m_targetFrame;
}

void ndIk6DofEffector::SetOffsetMatrix(const ndMatrix& matrix)
{
	m_targetFrame = matrix;
}

ndMatrix ndIk6DofEffector::GetEffectorMatrix() const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	ndJointBilateralConstraint::CalculateGlobalMatrix(matrix0, matrix1);
	const ndMatrix matrix(matrix0 * matrix1.OrthoInverse());
	return matrix;
}

ndMatrix ndIk6DofEffector::CalculateGlobalBaseMatrix1() const
{
	return ndJointBilateralConstraint::CalculateGlobalMatrix1();
}

ndMatrix ndIk6DofEffector::CalculateGlobalMatrix1() const
{
	return m_targetFrame * CalculateGlobalBaseMatrix1();
}

ndFloat32 ndIk6DofEffector::GetMaxForce() const
{
	return m_linearMaxForce;
}

void ndIk6DofEffector::SetMaxForce(ndFloat32 force)
{
	m_linearMaxForce = ndAbs(force);
}

ndFloat32 ndIk6DofEffector::GetMaxTorque() const
{
	return m_angularMaxTorque;
}

void ndIk6DofEffector::SetMaxTorque(ndFloat32 torque)
{
	m_angularMaxTorque = ndAbs(torque);
}

void ndIk6DofEffector::SetLinearSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_linearSpring = ndAbs(spring);
	m_linearDamper = ndAbs(damper);
	m_linearRegularizer = ndClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIk6DofEffector::GetLinearSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_linearSpring;
	damper = m_linearDamper;
	regularizer = m_linearRegularizer;
}

void ndIk6DofEffector::SetAngularSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_angularSpring = ndAbs(spring);
	m_angularDamper = ndAbs(damper);
	m_angularRegularizer = ndClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIk6DofEffector::GetAngularSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_angularSpring;
	damper = m_angularDamper;
	regularizer = m_angularRegularizer;
}

void ndIk6DofEffector::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	//const ndMatrix matrix0(m_localMatrix0 * m_body0->GetMatrix());
	//const ndMatrix matrix1(m_localMatrix1 * m_body1->GetMatrix());
	//const ndMatrix targetFrame(m_targetFrame * matrix1);
	const ndMatrix matrix0(CalculateGlobalMatrix0());
	const ndMatrix matrix1(ndJointBilateralConstraint::CalculateGlobalMatrix1());
	const ndMatrix targetFrame(CalculateGlobalMatrix1());
	
	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);
	debugCallback.DrawFrame(targetFrame);
	debugCallback.DrawPoint(targetFrame.m_posit, ndVector(1.0f, 1.0f, 0.0f, 0.0f), 8.0f);
}

void ndIk6DofEffector::SubmitShortestPathAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	const ndQuaternion rotation(matrix0.OrthoInverse() * matrix1);
	const ndVector pin(rotation & ndVector::m_triplexMask);
	const ndFloat32 dirMag2 = pin.DotProduct(pin).GetScalar();
	const ndFloat32 tol = ndFloat32(3.0f * ndPi / 180.0f);
	if (dirMag2 > (tol * tol))
	{
		const ndMatrix basis(ndGramSchmidtMatrix(pin));
		const ndFloat32 dirMag = ndSqrt(dirMag2);
		const ndFloat32 angle = ndAtan2(dirMag, rotation.m_w);
		AddAngularRowJacobian(desc, basis[0], angle);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
		SetLowerFriction(desc, -m_angularMaxTorque);
		SetHighFriction(desc, m_angularMaxTorque);

		AddAngularRowJacobian(desc, basis[1], ndFloat32(0.0f));
		SetLowerFriction(desc, -m_angularMaxTorque);
		SetHighFriction(desc, m_angularMaxTorque);

		AddAngularRowJacobian(desc, basis[2], ndFloat32(0.0f));
		SetLowerFriction(desc, -m_angularMaxTorque);
		SetHighFriction(desc, m_angularMaxTorque);
	}
	else
	{
		ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		AddAngularRowJacobian(desc, matrix1.m_up, angle0);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
		SetLowerFriction(desc, -m_angularMaxTorque);
		SetHighFriction(desc, m_angularMaxTorque);

		ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		AddAngularRowJacobian(desc, matrix1.m_right, angle1);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
		SetLowerFriction(desc, -m_angularMaxTorque);
		SetHighFriction(desc, m_angularMaxTorque);

		ndFloat32 angle2 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
		AddAngularRowJacobian(desc, matrix1.m_front, angle2);
		SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
		SetLowerFriction(desc, -m_angularMaxTorque);
		SetHighFriction(desc, m_angularMaxTorque);
	}
}

void ndIk6DofEffector::SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	switch (m_rotationType)
	{
		case m_fixAxis:
		{
			const ndMatrix matrix11(m_targetFrame * matrix1);
			const ndVector& pin = matrix11.m_front;

			const ndFloat32 angle = CalculateAngle(matrix0[1], matrix11[1], matrix11[0]);
			AddAngularRowJacobian(desc, pin, angle);
			SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
			break;
		}

		case m_shortestPath:
		{
			const ndMatrix matrix(m_targetFrame * matrix1);
			SubmitShortestPathAxis(matrix0, matrix, desc);
			break;
		}

		case m_disabled:
		default:;
	}
}

void ndIk6DofEffector::SubmitLinearAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	const ndMatrix& axisDir = matrix1;
	const ndVector posit0(matrix0.m_posit);
	const ndVector posit1(matrix1.TransformVector(m_targetFrame.m_posit));
	
	for (ndInt32 i = 0; i < 3; ++i)
	{
		if (m_controlDofOptions & (1 << i))
		{
			const ndVector pin = axisDir[i];
			AddLinearRowJacobian(desc, posit0, posit1, pin);
			SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
			SetLowerFriction(desc, -m_linearMaxForce);
			SetHighFriction(desc, m_linearMaxForce);
		}
	}
}

void ndIk6DofEffector::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearAxis(matrix0, matrix1, desc);
	SubmitAngularAxis(matrix0, matrix1, desc);
}