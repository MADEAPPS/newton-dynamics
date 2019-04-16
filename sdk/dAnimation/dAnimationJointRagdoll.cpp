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

#include "dAnimationStdAfx.h"
#include "dAnimationJointRagdoll.h"
#include "dAnimationModelManager.h"

class dAnimationJointRagdoll::dRagdollMotor: public dCustomBallAndSocket
{
	public:
	dRagdollMotor(dAnimationJointRagdoll* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dCustomBallAndSocket(pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
		,m_owner(owner)
		,m_dof(3)
	{
	}

	int GetDOF() const 
	{ 
		return m_dof; 
	}

	dAnimationJointRagdoll* m_owner;
	int m_dof;
};


class dAnimationJointRagdoll::dRagdollMotor_2dof : public dRagdollMotor
{
	public:
	dRagdollMotor_2dof(dAnimationJointRagdoll* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dRagdollMotor(owner, pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
	{
		m_dof = 2;
		m_coneFriction = 100.0f;
		m_twistFriction = 100.0f;
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);
		SubmitLinearRows(0x07, matrix0, matrix1);

		const dVector& motorAccel = m_owner->m_rowAccel;
		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;

		dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
		if (cosAngleCos < 0.9999f) {
			dMatrix coneRotation(dGetIdentityMatrix());
			dVector lateralDir(coneDir1.CrossProduct(coneDir0));
			dFloat mag2 = lateralDir.DotProduct3(lateralDir);
			if (mag2 > 1.0e-4f) {
				lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
				coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
			} else {
				dAssert(0);
				lateralDir = matrix0.m_up.Scale(-1.0f);
				coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat(180.0f) * dDegreeToRad), matrix1.m_posit);
			}

			dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
			dAssert(dAbs(pitchMatrix[0][0] - dFloat(1.0f)) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][1]) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][2]) < dFloat(1.0e-3f));
			dFloat pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);

#if 1
			dFloat coneAngle = dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)));
			dTrace(("cone:%f pitch:%f\n", coneAngle * dRadToDegree, pitchAngle * dRadToDegree));
#endif

			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0[0][0]);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[1]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

		} else {

			// using small angular aproximation to get the joint angle;
			dFloat pitchAngle = CalculateAngle(&matrix0[1][0], &matrix1[1][0], &matrix0[0][0]);
			NewtonUserJointAddAngularRow(m_joint, -pitchAngle, &matrix0[0][0]);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[1][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[2][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[1]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
		}
	}
};


class dAnimationJointRagdoll::dRagdollMotor_3dof: public dRagdollMotor
{
	public:
	dRagdollMotor_3dof(dAnimationJointRagdoll* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dRagdollMotor(owner, pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
	{
		m_dof = 3;
		m_coneFriction = 100.0f;
		m_twistFriction = 100.0f;
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);
		SubmitLinearRows(0x07, matrix0, matrix1);

		const dVector& motorAccel = m_owner->m_rowAccel;
		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;

		dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
		if (cosAngleCos < 0.9999f) {
			dMatrix coneRotation(dGetIdentityMatrix());
			dVector lateralDir(coneDir1.CrossProduct(coneDir0));
			dFloat mag2 = lateralDir.DotProduct3(lateralDir);
			if (mag2 > 1.0e-4f) {
				lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
				coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
			} else {
				dAssert(0);
				lateralDir = matrix0.m_up.Scale(-1.0f);
				coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat(180.0f) * dDegreeToRad), matrix1.m_posit);
			}

#if 0
			dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
			dAssert(dAbs(pitchMatrix[0][0] - dFloat(1.0f)) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][1]) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][2]) < dFloat(1.0e-3f));

			dFloat pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
			dFloat coneAngle = -dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)));
			dTrace(("cone:%f pitch:%f\n", coneAngle * dRadToDegree, pitchAngle * dRadToDegree));
#endif

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[1]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[2]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
		} else {
			// using small angular aproximation to get the joint angle;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[1][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[1]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[2][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[2]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
		}
	}
};



dAnimationJointRagdoll::dAnimationJointRagdoll(dRagdollMotorType type, const dMatrix& pinAndPivotInGlobalSpace, NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent)
	:dAnimationJoint(body, bindMarix, parent)
	,m_rowAccel(0.0f)
	,m_rows(0)
{
//	dJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
	dMatrix parentRollMatrix(dGetIdentityMatrix() * pinAndPivotInGlobalSpace);
type = m_twoDof;

	if (type == m_threeDof) {
		m_joint = new dRagdollMotor_3dof(this, pinAndPivotInGlobalSpace, parentRollMatrix, body, parent->GetBody());
	} else if (type == m_twoDof) {
		m_joint = new dRagdollMotor_2dof(this, pinAndPivotInGlobalSpace, parentRollMatrix, body, parent->GetBody());
	} else {
		dAssert(0);
		dAssert(type == m_oneDof);
	}

	//dFloat friction = definition.m_friction * 0.25f;
	//joint->EnableCone(true);
	//joint->SetConeFriction(friction);
	//joint->SetConeLimits(jointLimits.m_coneAngle * dDegreeToRad);
	//
	//joint->EnableTwist(true);
	//joint->SetTwistFriction(friction);
	//joint->SetTwistLimits(jointLimits.m_minTwistAngle * dDegreeToRad, jointLimits.m_maxTwistAngle * dDegreeToRad);

	m_proxyJoint = this;
	Init(GetProxyBody(), parent->GetProxyBody());
}

dAnimationJointRagdoll::~dAnimationJointRagdoll()
{
}

void dAnimationJointRagdoll::RigidBodyToStates()
{
	dAnimationJoint::RigidBodyToStates();
	m_proxyBody.SetForce(dVector(0.0f));
	m_proxyBody.SetTorque(dVector(0.0f));
}

void dAnimationJointRagdoll::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	m_rowAccel = dVector (0.0f);
	NewtonImmediateModeConstraint descriptor;

	dRagdollMotor* const ragDollJoint = (dRagdollMotor*) m_joint;
	NewtonJoint* const newtonJoint = ragDollJoint->GetJoint();
	const int rows = NewtonUserJointSubmitImmediateModeConstraint(newtonJoint, &descriptor, constraintParams->m_timestep);
	const int dof = ragDollJoint->GetDOF();
	dAssert (rows == 6);
	dAssert (rows >= dof);

	const int fixdof = rows - dof;
	for (int i = 0; i < fixdof; i++) {
		constraintParams->m_jacobians[i].m_jacobian_J01.m_linear = dVector(descriptor.m_jacobian01[i][0], descriptor.m_jacobian01[i][1], descriptor.m_jacobian01[i][2], dFloat(0.0f));
		constraintParams->m_jacobians[i].m_jacobian_J01.m_angular = dVector(descriptor.m_jacobian01[i][3], descriptor.m_jacobian01[i][4], descriptor.m_jacobian01[i][5], dFloat(0.0f));
		constraintParams->m_jacobians[i].m_jacobian_J10.m_linear = dVector(descriptor.m_jacobian10[i][0], descriptor.m_jacobian10[i][1], descriptor.m_jacobian10[i][2], dFloat(0.0f));
		constraintParams->m_jacobians[i].m_jacobian_J10.m_angular = dVector(descriptor.m_jacobian10[i][3], descriptor.m_jacobian10[i][4], descriptor.m_jacobian10[i][5], dFloat(0.0f));
		constraintParams->m_jointAccel[i] = descriptor.m_jointAccel[i];
		constraintParams->m_jointLowFrictionCoef[i] = descriptor.m_minFriction[i];
		constraintParams->m_jointHighFrictionCoef[i] = descriptor.m_maxFriction[i];
		constraintParams->m_normalIndex[i] = 0;
	}

	for (int i = fixdof; i < rows; i++) {
		const int j = i - fixdof;
		m_jacobial01[j].m_linear = dVector(descriptor.m_jacobian01[i][0], descriptor.m_jacobian01[i][1], descriptor.m_jacobian01[i][2], dFloat(0.0f));
		m_jacobial01[j].m_angular = dVector(descriptor.m_jacobian01[i][3], descriptor.m_jacobian01[i][4], descriptor.m_jacobian01[i][5], dFloat(0.0f));
	
		m_jacobial10[j].m_linear = dVector(descriptor.m_jacobian10[i][0], descriptor.m_jacobian10[i][1], descriptor.m_jacobian10[i][2], dFloat(0.0f));
		m_jacobial10[j].m_angular = dVector(descriptor.m_jacobian10[i][3], descriptor.m_jacobian10[i][4], descriptor.m_jacobian10[i][5], dFloat(0.0f));
	}

	m_rows = rows;
	m_dof = fixdof;
	m_count = fixdof;
	constraintParams->m_count = fixdof;
}

void dAnimationJointRagdoll::UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const
{
	dAssert (0);
}

void dAnimationJointRagdoll::UpdateJointAcceleration()
{
	dAnimationBody* const body0 = GetProxyBody();
	dAnimationBody* const body1 = m_parent->GetProxyBody();
	const dVector& accel0 = body0->GetForce().Scale(body0->GetInvMass());
	const dVector& alpha0 = body0->GetInvInertia().RotateVector(body0->GetTorque());
	const dVector& accel1 = body1->GetForce().Scale(body1->GetInvMass());
	const dVector& alpha1 = body1->GetInvInertia().RotateVector(body1->GetTorque());

	dRagdollMotor* const ragDollJoint = (dRagdollMotor*)m_joint;
	const int dof = ragDollJoint->GetDOF();
	const int fixdof = m_rows - dof;

	m_rowAccel = dVector(0.0f);
	for (int i = fixdof; i < m_rows; i++) {
		const int j = i - fixdof;
		const dComplementaritySolver::dJacobian& jacobial01 = m_jacobial01[j];
		const dComplementaritySolver::dJacobian& jacobial10 = m_jacobial10[j];
		dVector accel(accel0 * jacobial01.m_linear + alpha0 * jacobial01.m_angular + accel1 * jacobial10.m_linear + alpha1 * jacobial10.m_angular);
		m_rowAccel[j] = accel.m_x + accel.m_y + accel.m_z;
	}
	dAnimationJoint::UpdateJointAcceleration();
}