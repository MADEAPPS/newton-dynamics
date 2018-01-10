/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// dCustomBallAndSocket.cpp: implementation of the dCustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomRagdollMotor.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor);
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_1dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_2dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_3dof)
IMPLEMENT_CUSTOM_JOINT(dCustomRagdollMotor_EndEffector)


dEffectorTreeRoot::dEffectorTreeRoot(NewtonBody* const rootBody, dEffectorTreeInterface* const childNode)
	:dEffectorTreeInterface(rootBody)
{
	m_pose.m_childNode = childNode;
}

dEffectorTreeRoot::~dEffectorTreeRoot()
{
	dAssert(m_pose.m_childNode);
	delete m_pose.m_childNode;
}

void dEffectorTreeRoot::Evaluate(dEffectorPose& output, dFloat timestep, int threadIndex)
{
	dAssert(m_pose.m_childNode);
	m_pose.m_childNode->Evaluate(output, timestep, threadIndex);

	for (dEffectorPose::dListNode* srcNode = m_pose.GetFirst(); srcNode; srcNode = srcNode->GetNext()) {
		const dEffectorTransform& src = srcNode->GetInfo();
		dMatrix matrix(src.m_rotation, src.m_posit);
		//src.m_effector->SetTargetMatrix(matrix);
		src.m_effector->m_targetMatrix = matrix;
		NewtonBodySetSleepState(src.m_effector->GetBody0(), 0);
	}
}

void dEffectorTreeFixPose::Evaluate(dEffectorPose& output, dFloat timestep, int threadIndex)
{
	// just copy the base pose to the output frame
	for (dEffectorPose::dListNode* srcNode = m_pose.GetFirst(), *dstNode = output.GetFirst(); srcNode; srcNode = srcNode->GetNext(), dstNode = dstNode->GetNext()) {
		dEffectorTransform& dst = dstNode->GetInfo();
		const dEffectorTransform& src = srcNode->GetInfo();
		dAssert(dst.m_effector == src.m_effector);
		dst.m_rotation = src.m_rotation;
		dst.m_posit = src.m_posit;
	}
}

dEffectorTreeTwoWayBlender::dEffectorTreeTwoWayBlender(NewtonBody* const rootBody, dEffectorTreeInterface* const node0, dEffectorTreeInterface* const node1, dEffectorPose& output)
	:dEffectorTreeInterface(rootBody)
	,m_node0(node0)
	,m_node1(node1)
	,m_pose1()
	,m_param(1.0f)
{
	m_pose1.m_childNode = m_node1;
	if (m_pose1.GetCount() != output.GetCount()) {
		for (dEffectorPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			m_pose1.Append(node->GetInfo());
		}
	}
}

dEffectorTreeTwoWayBlender::~dEffectorTreeTwoWayBlender()
{
	delete m_node0;
	delete m_node1;
}

void dEffectorTreeTwoWayBlender::Evaluate(dEffectorPose& output, dFloat timestep, int threadIndex)
{
	if (m_param < 0.001f) {
		m_node0->Evaluate(output, timestep, threadIndex);
	} else if (m_param > 0.999f) {
		m_node1->Evaluate(output, timestep, threadIndex);
	} else {

		m_pose1.m_childNode = m_node1;
		if (m_pose1.GetCount() != output.GetCount()) {
			NewtonWorldCriticalSectionLock(NewtonBodyGetWorld(m_rootBody), threadIndex);
			m_pose1.RemoveAll();
			for (dEffectorPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
				m_pose1.Append(node->GetInfo());
			}
			NewtonWorldCriticalSectionUnlock(NewtonBodyGetWorld(m_rootBody));
		}

		m_node1->Evaluate(m_pose1, timestep, threadIndex);
		m_node0->Evaluate(output, timestep, threadIndex);
		for (dEffectorPose::dListNode* srcNode = m_pose1.GetFirst(), *dstNode = output.GetFirst(); srcNode; srcNode = srcNode->GetNext(), dstNode = dstNode->GetNext()) {
			dEffectorTransform& dst = dstNode->GetInfo();
			const dEffectorTransform& src = srcNode->GetInfo();
			dst.m_posit = dst.m_posit.Scale(1.0f - m_param) + src.m_posit.Scale(m_param);
			dQuaternion srcRotation(src.m_rotation);
			srcRotation.Scale(dSign(dst.m_rotation.DotProduct(src.m_rotation)));
			dst.m_rotation = dst.m_rotation.Slerp(srcRotation, m_param);
			dst.m_posit.m_w = 1.0f;
		}
	}
}


dCustomRagdollMotor::dCustomRagdollMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	,m_motorTorque(100.0f)
	,m_motorMode(true)
{
}

dCustomRagdollMotor::~dCustomRagdollMotor()
{
}

void dCustomRagdollMotor::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_motorTorque, sizeof(dFloat));
}

void dCustomRagdollMotor::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomBallAndSocket::Serialize(callback, userData);

	callback(userData, &m_motorTorque, sizeof(dFloat));
}

void dCustomRagdollMotor::SetJointTorque(dFloat torque)
{
	m_motorTorque = dAbs(torque);
}

dFloat dCustomRagdollMotor::GetJointTorque() const
{
	return m_motorTorque;
}

void dCustomRagdollMotor::DisableMotor()
{
	m_motorMode = false;
}

void dCustomRagdollMotor::EnableMotor()
{
	m_motorMode = true;
}




void dCustomRagdollMotor::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
}

dCustomRagdollMotor_1dof::dCustomRagdollMotor_1dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_minTwistAngle(-30.0f * 3.141592f / 180.0f)
	,m_maxTwistAngle( 30.0f * 3.141592f / 180.0f)
{
}

void dCustomRagdollMotor_1dof::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	m_minTwistAngle = 0.0f;
	m_maxTwistAngle = 0.0f;

	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_1dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_1dof::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = dMax(-dAbs(minAngle), dFloat(-170.0f * 3.141582f / 180.0f));
	m_maxTwistAngle = dMin( dAbs(maxAngle), dFloat( 170.0f * 3.141582f / 180.0f));
}

void dCustomRagdollMotor_1dof::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}


void dCustomRagdollMotor_1dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	dCustomRagdollMotor::Debug(debugDisplay);

	// vis limits
	const int subdiv = 16;
	const float radius = 0.25f;

	dVector point (0.0f, radius, 0.0f, 0.0f);
	dFloat angleStep = (m_maxTwistAngle - m_minTwistAngle) / subdiv;
	dFloat angle0 = m_minTwistAngle;

	dVector arch[subdiv + 1];
	debugDisplay->SetColor(dVector (1.0f, 1.0f, 0.0f, 0.0f));
	for (int i = 0; i <= subdiv; i++) {
		dVector p (matrix1.TransformVector(dPitchMatrix(angle0).RotateVector(point)));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}
}


void dCustomRagdollMotor_1dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	// two rows to restrict rotation around around the parent coordinate system
	CalculateGlobalMatrix(matrix0, matrix1);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);
	dFloat angle = CalculateAngle(matrix1.m_up, matrix0.m_up, matrix1.m_front);
	if (angle < m_minTwistAngle) {
		dFloat relAngle = angle - m_minTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, -relAngle, &matrix1.m_front[0]);
		if (m_motorMode) {
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
		}
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (angle > m_maxTwistAngle) {
		dFloat relAngle = angle - m_maxTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, -relAngle, &matrix1.m_front[0]);
		if (m_motorMode) {
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
		}
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		dVector omega0(0.0f);
		dVector omega1(0.0f);

		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);
		dVector relOmega(omega1 - omega0);
		dFloat invTimestep = 0.5f / timestep;

		dFloat accel = relOmega.DotProduct3(matrix1.m_front) * invTimestep;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowAsInverseDynamics(m_joint);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
	}
}



dCustomRagdollMotor_3dof::dCustomRagdollMotor_3dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	,m_coneAngle(30.0f * 3.141592f / 180.0f)
	,m_coneAngleOffset(0.0f)
	,m_minTwistAngle(-30.0f * 3.141592f / 180.0f)
	,m_maxTwistAngle(30.0f * 3.141592f / 180.0f)
{
}

void dCustomRagdollMotor_3dof::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
	callback(userData, &m_coneAngleOffset, sizeof(m_coneAngleOffset));
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_3dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
	callback(userData, &m_coneAngleOffset, sizeof(m_coneAngleOffset));
	callback(userData, &m_minTwistAngle, sizeof(m_minTwistAngle));
	callback(userData, &m_maxTwistAngle, sizeof(m_maxTwistAngle));
}

void dCustomRagdollMotor_3dof::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = dMax(-dAbs(minAngle), dFloat(-60.0f * 3.141582f / 180.0f));
	m_maxTwistAngle = dMin( dAbs(maxAngle), dFloat( 60.0f * 3.141582f / 180.0f));
}

void dCustomRagdollMotor_3dof::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}

void dCustomRagdollMotor_3dof::SetConeAngle(dFloat angle)
{
	m_coneAngle = dMin(dAbs(angle), dFloat(150.0f * 3.141582f / 180.0f));
}

dFloat dCustomRagdollMotor_3dof::GetConeAngle() const
{
	return m_coneAngle;
}

void dCustomRagdollMotor_3dof::SetConeAngleOffset(dFloat angle)
{
	m_coneAngleOffset = angle;
}


void dCustomRagdollMotor_3dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::Debug(debugDisplay);

	const int subdiv = 24;
	const float radius = 0.25f;

	dVector point(radius * dCos(m_coneAngle), radius * dSin(m_coneAngle), 0.0f, 0.0f);
	dFloat angleStep = 3.141692f * 2.0f / subdiv;
	dFloat angle0 = 0.0f;

	dVector arch[subdiv + 1];
	debugDisplay->SetColor(dVector(1.0f, 1.0f, 0.0f, 0.0f));

	dMatrix limitAligment (dRollMatrix(m_coneAngleOffset));
	for (int i = 0; i <= subdiv; i++) {
		//dVector p(matrix1.TransformVector(dPitchMatrix(angle0).RotateVector(point)));
		dVector conePoint(limitAligment.RotateVector(dPitchMatrix(angle0).RotateVector(point)));
		dVector p(matrix1.TransformVector(conePoint));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}

	// select an axis for the twist. 
	// any on the unit arc from coneDir0 to coneDir1 would do - average seemed best after some tests
	dVector coneDir0 (matrix0.m_front);
	dVector coneDir1 (matrix1.m_front);
	dFloat dot = coneDir0.DotProduct3(coneDir1);
	if (dot < 0.999f) {
		dVector pin (coneDir1.CrossProduct(coneDir0));
		dVector axis = pin.Scale (1.0f / dSqrt (pin.DotProduct3(pin)));
		dFloat angle = dAcos(dClamp(dot, dFloat(-1.0f), dFloat(1.0f)));
		dQuaternion rot (axis, angle);
		dVector posit (matrix1.m_posit);
		matrix1 = matrix1 * dMatrix (rot, dVector(0.0f, 0.0f, 0.0f, 1.0f));
		matrix1.m_posit = posit;
	}
	
	debugDisplay->SetColor(dVector(1.0f, 0.0f, 1.0f, 0.0f));
	debugDisplay->DrawLine(matrix0.m_posit, matrix0.m_posit - matrix0.m_up.Scale (radius * 1.25f));

	matrix1 = dRollMatrix(-90.0f * 3.141592f / 180.0f) * matrix1;
	const int subdiv1 = 12;
	point = dVector (radius, 0.0f, 0.0f, 0.0f);
	angleStep = (m_maxTwistAngle - m_minTwistAngle) / subdiv1;
	angle0 = m_minTwistAngle;

	dVector arch1[subdiv + 1];
	debugDisplay->SetColor(dVector(0.0f, 1.0f, 1.0f, 0.0f));
	for (int i = 0; i <= subdiv1; i++) {
		dVector p(matrix1.TransformVector(dYawMatrix(angle0).RotateVector(point)));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv1; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}
}

void dCustomRagdollMotor_3dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dVector omega1(0.0f);

	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

	CalculateGlobalMatrix(matrix0, matrix1);
	matrix0 = dRollMatrix(-m_coneAngleOffset) * matrix0;

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);

	const dVector relOmega (omega1 - omega0);
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat dot = coneDir0.DotProduct3(coneDir1);

	dFloat correctionFactor = 0.3f;
	dFloat invTimestep = 1.0f / timestep;
	dFloat coneAngle = dAcos(dClamp(dot, dFloat(-1.0f), dFloat(1.0f)));
	dFloat angle = coneAngle - m_coneAngle;

	if (angle > 0.0f) {
		dVector swingAxis(coneDir0.CrossProduct(coneDir1));
		dAssert(swingAxis.DotProduct3(swingAxis) > 0.0f);
		swingAxis = swingAxis.Scale(1.0f / dSqrt(swingAxis.DotProduct3(swingAxis)));
		NewtonUserJointAddAngularRow(m_joint, angle, &swingAxis[0]);
		dFloat accel = (correctionFactor * angle * invTimestep + relOmega.DotProduct3(swingAxis)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		if (m_motorMode) {
			NewtonUserJointSetRowAsInverseDynamics(m_joint);

			dVector sideDir(swingAxis.CrossProduct(coneDir0));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
		}

	} else if (m_motorMode) {

		if (coneAngle > 1.0f * 3.141592f / 180.0f) {
			dVector swingAxis = (coneDir0.CrossProduct(coneDir1));
			dAssert(swingAxis.DotProduct3(swingAxis) > 0.0f);
			swingAxis = swingAxis.Scale(1.0f / dSqrt(swingAxis.DotProduct3(swingAxis)));

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &swingAxis[0]);
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

			dVector sideDir(swingAxis.CrossProduct(coneDir0));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
		} else {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
		}
	}

	if (dot < 0.999f) {
		dVector pin(coneDir1.CrossProduct(coneDir0));
		dVector axis = pin.Scale(1.0f / dSqrt(pin.DotProduct3(pin)));
		angle = dAcos(dClamp(dot, dFloat(-1.0f), dFloat(1.0f)));
		dQuaternion rot(axis, angle);
		matrix1 = matrix1 * dMatrix(rot, dVector(0.0f, 0.0f, 0.0f, 1.0f));
	}

	dFloat twistAngle = CalculateAngle (matrix0.m_up, matrix1.m_up, matrix1.m_front);
	if (twistAngle < m_minTwistAngle) {
		twistAngle = twistAngle - m_minTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &matrix1.m_front[0]);
		dFloat accel = (correctionFactor * twistAngle * invTimestep + relOmega.DotProduct3(matrix1.m_front)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		if (m_motorMode) {
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
		}
		NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
	} else if (twistAngle > m_maxTwistAngle) {
		twistAngle = twistAngle - m_maxTwistAngle;
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &matrix1.m_front[0]);
		dFloat accel = (correctionFactor * twistAngle * invTimestep + relOmega.DotProduct3(matrix1.m_front)) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, accel);
		if (m_motorMode) {
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
		}
		NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
	} else if (m_motorMode) {
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowAsInverseDynamics(m_joint);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
	}
}

dCustomRagdollMotor_EndEffector::dCustomRagdollMotor_EndEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, NewtonBody* const referenceBody, const dMatrix& attachmentPointInGlobalSpace)
	:dCustomJoint (invDynSolver, invDynNode)
	,m_targetMatrix(dGetIdentityMatrix())
	,m_referenceBody(referenceBody)
	,m_linearSpeed(1.0f)
	,m_angularSpeed(1.0f)
	,m_linearFriction(1000.0f)
	,m_angularFriction(1000.0f)
	,m_isSixdof(true)
{
	SetAsThreedof();
	CalculateLocalMatrix(attachmentPointInGlobalSpace, m_localMatrix0, m_localMatrix1);
	SetTargetMatrix(attachmentPointInGlobalSpace);
	SetSolverModel(2);
}

dCustomRagdollMotor_EndEffector::dCustomRagdollMotor_EndEffector(NewtonBody* const body, NewtonBody* const referenceBody, const dMatrix& attachmentPointInGlobalSpace)
	:dCustomJoint(6, body, NULL)
	,m_targetMatrix(dGetIdentityMatrix())
	,m_referenceBody(referenceBody)
	,m_linearSpeed(1.0f)
	,m_angularSpeed(1.0f)
	,m_linearFriction(1000.0f)
	,m_angularFriction(1000.0f)
	,m_isSixdof(true)
{
	SetAsThreedof();
	CalculateLocalMatrix(attachmentPointInGlobalSpace, m_localMatrix0, m_localMatrix1);
	SetTargetMatrix(attachmentPointInGlobalSpace);
	SetSolverModel(2);
}

dCustomRagdollMotor_EndEffector::~dCustomRagdollMotor_EndEffector()
{
}


void dCustomRagdollMotor_EndEffector::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	dAssert(0);
}

void dCustomRagdollMotor_EndEffector::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dAssert(0);
}


void dCustomRagdollMotor_EndEffector::SetAsSixdof()
{
	m_isSixdof = true;
}

void dCustomRagdollMotor_EndEffector::SetAsThreedof()
{
	m_isSixdof = false;
}


void dCustomRagdollMotor_EndEffector::SetMaxLinearFriction(dFloat friction)
{
	m_linearFriction = dAbs(friction);
}

void dCustomRagdollMotor_EndEffector::SetMaxAngularFriction(dFloat friction)
{
	m_angularFriction = dAbs(friction);
}


void dCustomRagdollMotor_EndEffector::SetLinearSpeed(dFloat speed)
{
	m_linearSpeed = dAbs (speed);
}

void dCustomRagdollMotor_EndEffector::SetAngularSpeed(dFloat speed)
{
	m_angularSpeed = dAbs (speed);
}


void dCustomRagdollMotor_EndEffector::SetTargetRotation(const dQuaternion& rotation)
{
	dAssert(0);
//	NewtonBodySetSleepState(m_body0, 0);
//	m_targetMatrix = dMatrix (rotation, m_targetMatrix.m_posit);
}

void dCustomRagdollMotor_EndEffector::SetTargetPosit(const dVector& posit)
{
	dMatrix parentMatrix;
//	NewtonBodySetSleepState(m_body0, 0);

	NewtonBodyGetMatrix(m_referenceBody, &parentMatrix[0][0]);
	m_targetMatrix.m_posit = parentMatrix.UntransformVector(posit);
}

void dCustomRagdollMotor_EndEffector::SetTargetMatrix(const dMatrix& matrix)
{
	dMatrix parentMatrix;
	NewtonBodySetSleepState(m_body0, 0);

	NewtonBodyGetMatrix(m_referenceBody, &parentMatrix[0][0]);
	m_targetMatrix = matrix * parentMatrix.Inverse();
}

dMatrix dCustomRagdollMotor_EndEffector::GetBodyMatrix() const
{
	dMatrix matrix0;
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);
	return m_localMatrix0 * matrix0;
}


dMatrix dCustomRagdollMotor_EndEffector::GetTargetMatrix() const
{
	dMatrix parentMatrix;
	NewtonBodyGetMatrix(m_referenceBody, &parentMatrix[0][0]);
	return m_targetMatrix * parentMatrix;
}

void dCustomRagdollMotor_EndEffector::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix parentMatrix;
	NewtonBodyGetMatrix(m_referenceBody, &parentMatrix[0][0]);
	debugDisplay->DrawFrame(GetBodyMatrix());
	debugDisplay->DrawFrame(m_targetMatrix * parentMatrix);
}

void dCustomRagdollMotor_EndEffector::SubmitConstraints(dFloat timestep, int threadIndex)
{
	// check if this is an impulsive time step
	dMatrix parentMatrix;
	dMatrix matrix0(GetBodyMatrix());
	dVector veloc(0.0f);
	dVector omega(0.0f);
	dVector com(0.0f);
	dVector veloc0(0.0f);
	dVector veloc1(0.0f);
	dAssert(timestep > 0.0f);
	const dFloat damp = 0.3f;
	const dFloat invTimestep = 1.0f / timestep;

	
	NewtonBodyGetMatrix(m_referenceBody, &parentMatrix[0][0]);
	parentMatrix = m_targetMatrix * parentMatrix;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	NewtonBodyGetPointVelocity(m_referenceBody, &parentMatrix.m_posit[0], &veloc1[0]);

	dVector relVeloc(veloc1 - veloc0);
	dVector relPosit(parentMatrix.m_posit - matrix0.m_posit);

//if (relVeloc.DotProduct3(relVeloc) > 1.0)
//relVeloc =veloc1 - veloc0;

	for (int i = 0; i < 3; i++) {
		// Restrict the movement on the pivot point along all tree orthonormal direction
		dFloat speed = relVeloc.DotProduct3(parentMatrix[i]);
		dFloat dist = relPosit.DotProduct3(parentMatrix[i]) * damp;
		dFloat relSpeed = dClamp (dist * invTimestep + speed, -m_linearSpeed, m_linearSpeed);
		dFloat relAccel = relSpeed * invTimestep;
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &parentMatrix[i][0]);
		NewtonUserJointSetRowAcceleration(m_joint, relAccel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_linearFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_linearFriction);
	}


	if (m_isSixdof) {
		dQuaternion rotation(matrix0.Inverse() * parentMatrix);
		if (dAbs(rotation.m_q0) < 0.99998f) {
			dMatrix rot(dGrammSchmidt(dVector(rotation.m_q1, rotation.m_q2, rotation.m_q3)));
			dFloat angle = 2.0f * dAcos(dClamp(rotation.m_q0, dFloat(-1.0f), dFloat(1.0f)));
			NewtonUserJointAddAngularRow(m_joint, angle, &rot.m_front[0]);
			dFloat alpha = NewtonUserJointGetRowAcceleration (m_joint);
			if (dAbs (alpha) > m_angularSpeed * invTimestep) {
				alpha = dClamp (alpha, -m_angularSpeed * invTimestep, m_angularSpeed * invTimestep);
				NewtonUserJointSetRowAcceleration(m_joint, alpha);
			}
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &rot.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &rot.m_right[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
		} else {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
		}
	}
}





dCustomRagdollMotor_2dof::dCustomRagdollMotor_2dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomRagdollMotor(pinAndPivotFrame, child, parent)
	, m_coneAngle(30.0f * 3.141592f / 180.0f)
{
}

void dCustomRagdollMotor_2dof::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
}

void dCustomRagdollMotor_2dof::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomRagdollMotor::Serialize(callback, userData);
	callback(userData, &m_coneAngle, sizeof(m_coneAngle));
}

void dCustomRagdollMotor_2dof::SetConeAngle(dFloat angle)
{
	m_coneAngle = dMin(dAbs(angle), dFloat(150.0f * 3.141582f / 180.0f));
}

dFloat dCustomRagdollMotor_2dof::GetConeAngle() const
{
	return m_coneAngle;
}

void dCustomRagdollMotor_2dof::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;

	CalculateGlobalMatrix(matrix0, matrix1);
	dCustomRagdollMotor::Debug(debugDisplay);

	const int subdiv = 24;
	const float radius = 0.25f;

	dVector point(radius * dCos(m_coneAngle), radius * dSin(m_coneAngle), 0.0f, 0.0f);
	dFloat angleStep = 3.141692f * 2.0f / subdiv;
	dFloat angle0 = 0.0f;

	dVector arch[subdiv + 1];
	debugDisplay->SetColor(dVector(1.0f, 1.0f, 0.0f, 0.0f));
	for (int i = 0; i <= subdiv; i++) {
		dVector conePoint(dPitchMatrix(angle0).RotateVector(point));
		dVector p(matrix1.TransformVector(conePoint));
		arch[i] = p;
		debugDisplay->DrawLine(matrix1.m_posit, p);
		angle0 += angleStep;
	}

	for (int i = 0; i < subdiv; i++) {
		debugDisplay->DrawLine(arch[i], arch[i + 1]);
	}
}

void dCustomRagdollMotor_2dof::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

/*
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	dFloat project = matrix1.m_front.DotProduct3(matrix0.m_front);
	if (dAbs(project) > 0.9995f) {
		dFloat twistAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
		NewtonUserJointAddAngularRow(m_joint, twistAngle, &matrix1.m_front[0]);

		if (m_motorMode) {
			dVector omega0(0.0f);
			dVector omega1(0.0f);
			NewtonBodyGetOmega(m_body0, &omega0[0]);
			NewtonBodyGetOmega(m_body1, &omega1[0]);
			dVector relOmega(omega1 - omega0);
			dFloat invTimestep = 0.5f / timestep;

			dFloat accel = relOmega.DotProduct3(matrix0.m_right) * invTimestep;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

			accel = relOmega.DotProduct3(matrix0.m_up) * invTimestep;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowAsInverseDynamics(m_joint);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
		}
	} else {
		dMatrix matrix1_(dGetIdentityMatrix());
		dMatrix matrix0_(dPitchMatrix(30.0f * 3.141592f / 180.0f) * dRollMatrix(25.0f * 3.141592f / 180.0f) * dYawMatrix(68.0f * 3.141592f / 180.0f) * matrix1_);

		matrix1_ = matrix1;
		matrix0_ = matrix0;
		dMatrix localMatrix(matrix0_ * matrix1_.Inverse());
		dFloat twistAngle_ = dAtan2 (-localMatrix.m_right.m_y, localMatrix.m_up.m_y);

dQuaternion  xxxx(matrix0_.Inverse() * dPitchMatrix(twistAngle_) * matrix0_);
dVector pin(xxxx.m_q1, xxxx.m_q2, xxxx.m_q3, 0.0f);
pin = pin.Normalize();

		dVector omega0(0.0f);
		dVector omega1(0.0f);
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);
		dVector relOmega(omega1 - omega0);
		dFloat invTimestep = 1.0f / timestep;


		dFloat alpha = (10.0 + relOmega.DotProduct3(matrix0.m_front)) * invTimestep;
		NewtonUserJointAddAngularRow(m_joint, -twistAngle_, &matrix0.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, alpha);


		dFloat coneAngle = dAcos(dClamp(project, dFloat(-1.0f), dFloat(1.0f)));
		dVector conePlane(matrix1.m_front.CrossProduct(matrix0.m_front).Normalize());
		dMatrix coneMatrix(dQuaternion(conePlane, coneAngle), matrix1.m_posit);
		matrix1 = matrix1 * coneMatrix;
		dFloat twistAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
//		NewtonUserJointAddAngularRow(m_joint, twistAngle, &matrix1.m_front[0]);

		dFloat angle = coneAngle - m_coneAngle;
		if (angle > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, -angle, &conePlane[0]);
			if (m_motorMode) {
				NewtonUserJointSetRowAsInverseDynamics(m_joint);
			}
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

			dVector sideDir(conePlane.CrossProduct(matrix1.m_front));
			dAssert(sideDir.DotProduct3(sideDir) > 0.999f);
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			if (m_motorMode) {
				NewtonUserJointSetRowAsInverseDynamics(m_joint);
			}
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

		} else if (m_motorMode) {
			dVector omega0(0.0f);
			dVector omega1(0.0f);
			NewtonBodyGetOmega(m_body0, &omega0[0]);
			NewtonBodyGetOmega(m_body1, &omega1[0]);
			dVector relOmega(omega1 - omega0);
			dFloat invTimestep = 0.25f / timestep;

			if (coneAngle > 1.0f * 3.141592f / 180.0f) {
				dFloat accel = relOmega.DotProduct3(conePlane) * invTimestep;
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &conePlane[0]);
				NewtonUserJointSetRowAcceleration(m_joint, accel);
				NewtonUserJointSetRowAsInverseDynamics(m_joint);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

				dVector sideDir(conePlane.CrossProduct(matrix1.m_front));
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
				NewtonUserJointSetRowAsInverseDynamics(m_joint);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
			} else {

				dFloat accel = relOmega.DotProduct3(matrix0.m_right) * invTimestep;
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_right[0]);
				NewtonUserJointSetRowAcceleration(m_joint, accel);
				NewtonUserJointSetRowAsInverseDynamics(m_joint);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

				accel = relOmega.DotProduct3(matrix0.m_up) * invTimestep;
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_up[0]);
				NewtonUserJointSetRowAcceleration(m_joint, accel);
				NewtonUserJointSetRowAsInverseDynamics(m_joint);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
			}
		}
//		dTrace(("%f %f\n", -twistAngle * 180.0f / 3.141592f, twistAngle_ * 180.0f / 3.141592f));
	}
*/

	dMatrix matrix0;
	dMatrix matrix1;
	
	//matrix1 = dGetIdentityMatrix();
	//matrix0 = dPitchMatrix(30.0f * 3.141592f / 180.0f) * dRollMatrix(25.0f * 3.141592f / 180.0f) * dYawMatrix(68.0f * 3.141592f / 180.0f) * matrix1;
	//dMatrix localMatrix(matrix0 * matrix1.Inverse());
	//dFloat twistAngle = dAtan2(-localMatrix.m_right.m_y, localMatrix.m_up.m_y);

	dVector omega0(0.0f);
	dVector omega1(0.0f);

	CalculateGlobalMatrix(matrix0, matrix1);
	dMatrix localMatrix(matrix0 * matrix1.Inverse());

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);
	dVector relOmega(omega0 - omega1);

	const dFloat damp = 0.4f;
	const dFloat invTimestep = 1.0f / timestep;

	// calculate twisting axis acceleration
	dFloat twistAngle = dAtan2(-localMatrix.m_right.m_y, localMatrix.m_up.m_y);
	dFloat alpha = (twistAngle * damp * invTimestep + relOmega.DotProduct3(matrix0.m_front)) * invTimestep;
	NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
	NewtonUserJointSetRowAcceleration(m_joint, -alpha);

dFloat xxxxx = dAbs (twistAngle * 180.0f / 3.141592f);
dTrace(("%f\n", twistAngle * 180.0f / 3.141592f));
}
