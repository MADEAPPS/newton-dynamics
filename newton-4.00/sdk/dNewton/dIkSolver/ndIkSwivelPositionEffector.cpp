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
#include "ndSkeletonContainer.h"
#include "ndIkSwivelPositionEffector.h"

ndIkSwivelPositionEffector::ndIkSwivelPositionEffector()
	:ndJointBilateralConstraint()
	,m_restPosition(ndVector::m_wOne)
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
	,m_enableSwivelControl(true)
{
	m_maxDof = 6;
}

ndIkSwivelPositionEffector::ndIkSwivelPositionEffector(
	const ndMatrix& pinAndPivotParentInGlobalSpace, ndBodyKinematic* const parent,
	const ndVector& childPivotInGlobalSpace, ndBodyKinematic* const child)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotParentInGlobalSpace)
	,m_restPosition(ndVector::m_wOne)
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
	,m_enableSwivelControl(true)
{
	ndMatrix temp;
	ndMatrix matrix(pinAndPivotParentInGlobalSpace);
	matrix.m_posit = childPivotInGlobalSpace;
	CalculateLocalMatrix(matrix, m_localMatrix0, temp);
	ndAssert((temp[0].DotProduct(m_localMatrix0[0]).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
	ndAssert((temp[1].DotProduct(m_localMatrix0[1]).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
	ndAssert((temp[2].DotProduct(m_localMatrix0[2]).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
	
	ndVector offset(pinAndPivotParentInGlobalSpace.UntransformVector(childPivotInGlobalSpace) & ndVector::m_triplexMask);
	m_maxWorkSpaceRadio = ndFloat32(0.99f) * ndSqrt(offset.DotProduct(offset).GetScalar());
	m_restPosition = offset.Normalize().Scale(m_maxWorkSpaceRadio) | ndVector::m_wOne;
	SetLocalTargetPosition(m_restPosition);
	
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndIkSwivelPositionEffector::~ndIkSwivelPositionEffector()
{
}

#if 0

ndVector ndIkSwivelPositionEffector::GetGlobalPosition() const
{
	return GetBody0()->GetMatrix().TransformVector(GetLocalMatrix0().m_posit);
}
#endif

bool ndIkSwivelPositionEffector::GetSwivelMode() const
{
	return m_enableSwivelControl;
}

void ndIkSwivelPositionEffector::SetSwivelMode(bool active)
{
	m_enableSwivelControl = active;
}

ndFloat32 ndIkSwivelPositionEffector::GetSwivelAngle() const
{
	return m_swivelAngle;
}

void ndIkSwivelPositionEffector::SetSwivelAngle(ndFloat32 angle)
{
	m_swivelAngle = angle;
}

ndVector ndIkSwivelPositionEffector::GetRestPosit() const
{
	return m_restPosition;
}

void ndIkSwivelPositionEffector::SetRestPosit(const ndVector& posit)
{
	m_restPosition = posit & ndVector::m_wOne;
}

ndVector ndIkSwivelPositionEffector::GetLocalTargetPosition() const
{
	return m_localTargetPosit;
}

void ndIkSwivelPositionEffector::SetLocalTargetPosition(const ndVector& posit)
{
	ndVector target (posit & ndVector::m_triplexMask);
	ndFloat32 dist2 = target.DotProduct(target).GetScalar();
	if (dist2 < ndFloat32(1.0e-4f))
	{
		target = m_restPosition;
		dist2 = target.DotProduct(target).GetScalar();
	}
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

void ndIkSwivelPositionEffector::SetWorkSpaceConstraints(ndFloat32 minRadio, ndFloat32 maxRadio)
{
	m_minWorkSpaceRadio = minRadio;
	m_maxWorkSpaceRadio = maxRadio;
	// make sure the target is with in the workspace constraint
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
	ndFloat32 yaw = -ndAsin(pin.m_z);
	ndFloat32 roll = ndAtan2(pin.m_y, pin.m_x);
	swivelMatrix = ndYawMatrix(yaw) * ndRollMatrix(roll) * matrix1;
	swivelMatrix.m_posit = matrix1.TransformVector(m_localTargetPosit);
	return swivelMatrix;
}

ndVector ndIkSwivelPositionEffector::GetEffectorPosit() const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	ndVector posit(matrix1.UntransformVector(matrix0.m_posit));

	//const ndMatrix swivelMatrix(ndPitchMatrix(-m_swivelAngle) * CalculateSwivelFrame(matrix1));
	//posit.m_w = CalculateAngle(matrix0[1], swivelMatrix[1], swivelMatrix[0]);
	return posit;
}

void ndIkSwivelPositionEffector::ClearMemory()
{
	ndJointBilateralConstraint::ClearMemory();
	m_swivelAngle = ndFloat32(0.0f);
	SetLocalTargetPosition(m_restPosition);
}

ndInt32 ndIkSwivelPositionEffector::GetKinematicState(ndKinematicState* const state) const
{
	ndVector posit;
	ndVector veloc;
	ndMatrix matrix0;
	ndMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	//matrix1.m_posit = matrix1.TransformVector(m_localTargetPosit);

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

	const ndMatrix swivelMatrix(ndPitchMatrix(-m_swivelAngle) * CalculateSwivelFrame(matrix1));
	posit.m_w = CalculateAngle(matrix0[1], swivelMatrix[1], swivelMatrix[0]);
	veloc.m_w = (omega0 * swivelMatrix.m_front - omega1 * swivelMatrix.m_front).AddHorizontal().GetScalar();

	for (ndInt32 i = 0; i < 4; i++)
	{
		state[i].m_posit = posit[i];
		state[i].m_velocity = veloc[i];
	}
	return 4;
}

ndFloat32 ndIkSwivelPositionEffector::CalculateLookAtSwivelAngle(const ndVector& upDir) const
{
	ndFloat32 swivelAngle = GetSwivelAngle();
	const ndMatrix swivelMatrix(CalculateSwivelFrame(m_localMatrix1 * m_body1->GetMatrix()));

	ndVector side (swivelMatrix.m_front.CrossProduct(upDir));
	if (ndAbs(side.DotProduct(side).GetScalar()) > ndFloat32 (0.01f))
	{
		ndMatrix targetswivelMatrix(swivelMatrix);
		targetswivelMatrix.m_right = side.Normalize();
		targetswivelMatrix.m_up = targetswivelMatrix.m_right.CrossProduct(targetswivelMatrix.m_front);
		const ndMatrix swivelPitch(swivelMatrix * targetswivelMatrix.OrthoInverse());
		swivelAngle = ndAtan2(swivelPitch.m_up.m_z, swivelPitch.m_up.m_y);
	}

	return swivelAngle;
}

void ndIkSwivelPositionEffector::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	//debugCallback.DrawFrame(matrix0);
	//debugCallback.DrawFrame(matrix1);

	debugCallback.DrawLine(matrix0.m_posit, matrix1.m_posit, ndVector(ndFloat32(0.89f), ndFloat32(0.70f), ndFloat32(0.13f), ndFloat32(1.0f)));
	
	const ndVector origin((matrix0.m_posit + matrix1.m_posit) * ndVector::m_half);
	ndMatrix swivelMatrix1(CalculateSwivelFrame(matrix1));
	swivelMatrix1.m_posit = origin;
	debugCallback.DrawFrame(swivelMatrix1);
	
	const ndMatrix swivelMatrix0(ndPitchMatrix(-m_swivelAngle) * swivelMatrix1);
	debugCallback.DrawFrame(swivelMatrix0);
}

void ndIkSwivelPositionEffector::SubmitLinearAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
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

void ndIkSwivelPositionEffector::SubmitAngularAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	const ndMatrix swivelMatrix1(ndPitchMatrix(-m_swivelAngle) * CalculateSwivelFrame(matrix1));
	const ndVector& pin = swivelMatrix1.m_front;
	const ndFloat32 angle = CalculateAngle(matrix0[1], swivelMatrix1[1], swivelMatrix1[0]);

	AddAngularRowJacobian(desc, pin, angle);
	SetMassSpringDamperAcceleration(desc, m_angularRegularizer, m_angularSpring, m_angularDamper);
	SetHighFriction(desc, m_angularMaxTorque);
	SetLowerFriction(desc, -m_angularMaxTorque);
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
