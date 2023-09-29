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
#include "ndIkSwivelPositionEffector.h"

ndIkSwivelPositionEffector::ndIkSwivelPositionEffector()
	:ndJointBilateralConstraint()
	,m_localTargetPosit(ndVector::m_wOne)
	,m_swivelAngle(ndFloat32(0.0f))
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularMaxTorque(D_LCP_MAX_VALUE)
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearMaxForce(D_LCP_MAX_VALUE)
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_minWorkSpaceRadio(ndFloat32(0.0f))
	,m_maxWorkSpaceRadio(ndFloat32(0.0f))
	,m_rotationOrder(m_pitchYawRoll)
	,m_enableSwivelControl(true)
{
	m_maxDof = 6;
}

ndIkSwivelPositionEffector::ndIkSwivelPositionEffector(const ndVector& childPivot, const ndMatrix& pinAndPivotParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotParent)
	,m_localTargetPosit(ndVector::m_wOne)
	,m_swivelAngle(ndFloat32(0.0f))
	,m_angularSpring(ndFloat32(1000.0f))
	,m_angularDamper(ndFloat32(50.0f))
	,m_angularMaxTorque(D_LCP_MAX_VALUE)
	,m_angularRegularizer(ndFloat32(5.0e-3f))
	,m_linearSpring(ndFloat32(1000.0f))
	,m_linearDamper(ndFloat32(50.0f))
	,m_linearMaxForce(D_LCP_MAX_VALUE)
	,m_linearRegularizer(ndFloat32(5.0e-3f))
	,m_minWorkSpaceRadio(ndFloat32(0.0f))
	,m_maxWorkSpaceRadio(ndFloat32(0.0f))
	,m_rotationOrder(m_pitchYawRoll)
	,m_enableSwivelControl(true)
{
	ndMatrix temp;
	ndMatrix matrix1 (pinAndPivotParent);

	ndVector offset(pinAndPivotParent.UntransformVector(childPivot) & ndVector::m_triplexMask);
	m_maxWorkSpaceRadio = ndFloat32(0.99f) * ndSqrt(offset.DotProduct(offset).GetScalar());
	m_localTargetPosit.m_x = m_maxWorkSpaceRadio;

	matrix1.m_posit = childPivot;
	CalculateLocalMatrix(matrix1, m_localMatrix0, temp);

	SetSolverModel(m_jointkinematicCloseLoop);
}

ndIkSwivelPositionEffector::~ndIkSwivelPositionEffector()
{
}

bool ndIkSwivelPositionEffector::GetSwivelMode() const
{
	return m_enableSwivelControl;
}

void ndIkSwivelPositionEffector::SetSwivelMode(bool active)
{
	m_enableSwivelControl = active;
}

ndIkSwivelPositionEffector::ndRotationOrder ndIkSwivelPositionEffector::GetRotationOrder() const
{
	return m_rotationOrder;
}

void ndIkSwivelPositionEffector::SetRotationOrder(ndRotationOrder order)
{
	m_rotationOrder = order;
}

ndFloat32 ndIkSwivelPositionEffector::GetSwivelAngle() const
{
	return m_swivelAngle;
}

void ndIkSwivelPositionEffector::SetSwivelAngle(const ndFloat32 angle)
{
	m_swivelAngle = angle;
}

ndVector ndIkSwivelPositionEffector::GetLocalTargetPosition() const
{
	return m_localTargetPosit;
}

void ndIkSwivelPositionEffector::SetLocalTargetPosition(const ndVector& posit)
{
	ndVector target (posit & ndVector::m_triplexMask);
	ndFloat32 dist2 = target.DotProduct(target).GetScalar();
	if (dist2 > m_maxWorkSpaceRadio * m_maxWorkSpaceRadio)
	{
		target = target.Normalize().Scale(m_maxWorkSpaceRadio);
	}
	else if (dist2 < m_minWorkSpaceRadio * m_minWorkSpaceRadio)
	{
		target = target.Normalize().Scale(m_minWorkSpaceRadio);
	}
	m_localTargetPosit = target | ndVector::m_wOne;
}

ndMatrix ndIkSwivelPositionEffector::GetReferenceFrame() const
{
	return GetLocalMatrix1() * GetBody1()->GetMatrix();
}

ndVector ndIkSwivelPositionEffector::GetGlobalPosition() const
{
	return GetBody0()->GetMatrix().TransformVector(GetLocalMatrix0().m_posit);
}

void ndIkSwivelPositionEffector::SetWorkSpaceConstraints(ndFloat32 minRadio, ndFloat32 maxRadio)
{
	m_minWorkSpaceRadio = minRadio;
	m_maxWorkSpaceRadio = maxRadio;
	// make sure the taget is with in the workspace constraint
	SetLocalTargetPosition(GetLocalTargetPosition());
}

void ndIkSwivelPositionEffector::GetWorkSpaceConstraints(ndFloat32& minRadio, ndFloat32& maxRadio) const
{
	minRadio = m_minWorkSpaceRadio;
	maxRadio = m_maxWorkSpaceRadio;
}

ndFloat32 ndIkSwivelPositionEffector::GetMaxForce() const
{
	return m_linearMaxForce;
}

void ndIkSwivelPositionEffector::SetMaxForce(ndFloat32 force)
{
	m_linearMaxForce = ndAbs(force);
}

ndFloat32 ndIkSwivelPositionEffector::GetMaxTorque() const
{
	return m_angularMaxTorque;
}

void ndIkSwivelPositionEffector::SetMaxTorque(ndFloat32 torque)
{
	m_angularMaxTorque = ndAbs(torque);
}

void ndIkSwivelPositionEffector::SetLinearSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_linearSpring = ndAbs(spring);
	m_linearDamper = ndAbs(damper);
	m_linearRegularizer = ndClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIkSwivelPositionEffector::GetLinearSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_linearSpring;
	damper = m_linearDamper;
	regularizer = m_linearRegularizer;
}

void ndIkSwivelPositionEffector::SetAngularSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_angularSpring = ndAbs(spring);
	m_angularDamper = ndAbs(damper);
	m_angularRegularizer = ndClamp(regularizer, ndFloat32(1.0e-4f), ndFloat32(0.99f));
}

void ndIkSwivelPositionEffector::GetAngularSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_angularSpring;
	damper = m_angularDamper;
	regularizer = m_angularRegularizer;
}

ndMatrix ndIkSwivelPositionEffector::CalculateSwivelFrame(const ndMatrix& matrix1) const
{
	ndMatrix swivelMatrix;
	ndVector pin((m_localTargetPosit & ndVector::m_triplexMask).Normalize());
	if (m_rotationOrder == m_pitchYawRoll)
	{
		ndFloat32 yaw = -ndAsin(pin.m_z);
		ndFloat32 roll = ndAtan2(pin.m_y, pin.m_x);
		swivelMatrix = ndYawMatrix(yaw) * ndRollMatrix(roll) * matrix1;
	}
	else
	{
		ndAssert(0);
		//ndFloat32 yaw = -ndAsin(pin.m_z);
		//ndFloat32 roll = ndAtan2(pin.m_y, pin.m_x);
		//swivelMatrix = ndYawMatrix(yaw) * ndRollMatrix(roll) * matrix1;
	}
	swivelMatrix.m_posit = matrix1.TransformVector(m_localTargetPosit);
	return swivelMatrix;
}

void ndIkSwivelPositionEffector::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	debugCallback.DrawFrame(matrix1, ndFloat32 (0.5f));
	debugCallback.DrawLine(matrix0.m_posit, matrix1.m_posit, ndVector(ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.0f), ndFloat32(1.0f)));

	ndMatrix swivelMatrix0(CalculateSwivelFrame(matrix1));
	ndMatrix swivelMatrix1(ndPitchMatrix(m_swivelAngle) * swivelMatrix0);
	debugCallback.DrawFrame(swivelMatrix0);
	debugCallback.DrawFrame(swivelMatrix1);
	debugCallback.DrawPoint(swivelMatrix0.m_posit, ndVector(1.0f, 1.0f, 0.0f, 0.0f), 8.0f);
}

void ndIkSwivelPositionEffector::SubmitAngularAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	const ndMatrix swivelMatrix1(ndPitchMatrix(m_swivelAngle) * CalculateSwivelFrame(matrix1));
	const ndVector& pin = swivelMatrix1.m_front;
	const ndFloat32 angle = CalculateAngle(matrix0[1], swivelMatrix1[1], swivelMatrix1[0]);
	
	AddAngularRowJacobian(desc, pin, angle);
	SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
}

//#pragma optimize( "", off ) //for debugging purpose
void ndIkSwivelPositionEffector::SubmitLinearAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
//static ndVector xxx0;
//static ndVector xxx1;
//GetDynamicState(xxx0, xxx1);

	const ndVector posit0(matrix0.m_posit);
	const ndVector posit1(matrix1.TransformVector(m_localTargetPosit));
	const ndMatrix& axisDir = matrix1;
	for (ndInt32 i = 0; i < 3; ++i)
	{
		const ndVector pin = axisDir[i];
		AddLinearRowJacobian(desc, posit0, posit1, pin);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
		SetHighFriction(desc, m_linearMaxForce);
		SetLowerFriction(desc, -m_linearMaxForce);
	}
}

ndVector ndIkSwivelPositionEffector::GetEffectorPosit() const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	ndVector posit(matrix1.UntransformVector(matrix0.m_posit));

	const ndMatrix swivelMatrix(ndPitchMatrix(m_swivelAngle) * CalculateSwivelFrame(matrix1));
	posit.m_w = CalculateAngle(matrix0[1], swivelMatrix[1], swivelMatrix[0]);
	return posit;
}

void ndIkSwivelPositionEffector::GetDynamicState(ndVector& posit, ndVector& veloc) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	matrix1.m_posit = matrix1.TransformVector(m_localTargetPosit);

	const ndMatrix& axisDir = matrix1;
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());
	const ndVector veloc0(m_body0->GetVelocity());
	const ndVector veloc1(m_body1->GetVelocity());

	ndPointParam param;
	InitPointParam(param, matrix0.m_posit, matrix1.m_posit);
	for (ndInt32 i = 0; i < 3; ++i)
	{
		const ndVector& pin = axisDir[i];
		const ndVector r0CrossDir(param.m_r0.CrossProduct(pin));
		const ndVector r1CrossDir(param.m_r1.CrossProduct(pin));
		posit[i] = (pin * param.m_posit0 - pin * param.m_posit1).AddHorizontal().GetScalar();
		veloc[i] = (pin * veloc0 + r0CrossDir * omega0 - pin * veloc1 - r1CrossDir * omega1).AddHorizontal().GetScalar();
	}

	const ndMatrix swivelMatrix(ndPitchMatrix(m_swivelAngle) * CalculateSwivelFrame(matrix1));
	posit.m_w = CalculateAngle(matrix0[1], swivelMatrix[1], swivelMatrix[0]);
	veloc.m_w = (omega0 * swivelMatrix.m_front - omega1 * swivelMatrix.m_front).AddHorizontal().GetScalar();
}

void ndIkSwivelPositionEffector::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	SubmitLinearAxis(desc, matrix0, matrix1);
	if (m_enableSwivelControl)
	{
		SubmitAngularAxis(desc, matrix0, matrix1);
	}
}