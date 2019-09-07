/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "dAnimationRagdollJoint.h"
#include "dAnimationModelManager.h"

class dAnimationRagdollJoint::dRagdollMotor: public dCustomBallAndSocket
{
	public:
	dRagdollMotor(dAnimationRagdollJoint* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dCustomBallAndSocket(pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
		,m_owner(owner)
//		,m_dof(3)
//		,m_motorTorque (100000.0f)
	{
	}

	int GetDOF() const 
	{ 
//		return m_dof; 
		return 0;
	}

	void Debug(dDebugDisplay* const debugDisplay) const
	{
		dMatrix matrix0;
		dMatrix matrix1;

		dCustomJoint::Debug(debugDisplay);

		dFloat scale = debugDisplay->GetScale();
		debugDisplay->SetScale(3.0f);

		CalculateGlobalMatrix(matrix0, matrix1);

		debugDisplay->DrawFrame(matrix0);
		debugDisplay->DrawFrame(matrix1);

		debugDisplay->SetScale(scale);
	}


	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);
		SubmitLinearRows(0x07, matrix0, matrix1);
		SubmitAngularConstraints(matrix0, matrix1, timestep);
	}

	virtual void SubmitAngularConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		dAssert(0);
	}

	dAnimationRagdollJoint* m_owner;
//	dFloat m_motorTorque;
//	int m_dof;
};

class dAnimationRagdollJoint::dRagdollMotor_0dof : public dRagdollMotor
{
	public:
	dRagdollMotor_0dof(dAnimationRagdollJoint* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dRagdollMotor(owner, pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
	{
		//m_dof = 1;
		//m_dof = 0;
	}

	void Debug(dDebugDisplay* const debugDisplay) const
	{
//		dRagdollMotor::Debug(debugDisplay);
	}

	virtual void SubmitAngularConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		//dVector euler0;
		//dVector euler1;
		//dMatrix localMatrix(matrix0 * matrix1.Inverse());
		//localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);

		// submit the angular rows.
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), &matrix1.m_front[0]);

		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	}
};

class dAnimationRagdollJoint::dRagdollMotor_1dof : public dRagdollMotor
{
	public:
	dRagdollMotor_1dof(dAnimationRagdollJoint* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dRagdollMotor(owner, pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
	{
//		m_dof = 1;
//		m_dof = 0;
	}

	void Debug(dDebugDisplay* const debugDisplay) const
	{
//		dRagdollMotor::Debug(debugDisplay);
	}

	virtual void SubmitAngularConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		//dVector euler0;
		//dVector euler1;
		//dMatrix localMatrix(matrix0 * matrix1.Inverse());
		//localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);

		// submit the angular rows.
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	}
};


class dAnimationRagdollJoint::dRagdollMotor_2dof: public dRagdollMotor
{
	public:
	dRagdollMotor_2dof(dAnimationRagdollJoint* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dRagdollMotor(owner, pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
	{
//		m_dof = 2;
//		m_dof = 0;
	}

	void Debug(dDebugDisplay* const debugDisplay) const
	{
//		dRagdollMotor::Debug(debugDisplay);
	}

	virtual void SubmitAngularConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		dVector omega0(0.0f);
		dVector omega1(0.0f);
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);

		dVector euler0;
		dVector euler1;
		dMatrix localMatrix(matrix0 * matrix1.Inverse());
		localMatrix.GetEulerAngles(euler0, euler1, m_pitchRollYaw);

		dVector relOmega(omega0 - omega1);

		// not happy with this method because it is a penalty system, 
		// but is hard to the the right axis angular derivative.
		dMatrix rollMatrix(dYawMatrix(euler0[1]) * matrix1);
		NewtonUserJointAddAngularRow(m_joint, -euler0[2], &rollMatrix.m_right[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		dFloat rollOmega = relOmega.DotProduct3(rollMatrix.m_right);
		dFloat alphaRollError = -(euler0[2] + rollOmega * timestep) / (timestep * timestep);
		NewtonUserJointSetRowAcceleration(m_joint, alphaRollError);

		//NewtonUserJointAddAngularRow(m_joint, 0, &matrix0.m_front[0]);
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		//NewtonUserJointSetRowAcceleration(m_joint, m_owner->m_rowAccel[0]);
		//NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
		//NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
		
		//NewtonUserJointAddAngularRow(m_joint, 0, &matrix1.m_up[0]);
		//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		//NewtonUserJointSetRowAcceleration(m_joint, m_owner->m_rowAccel[1]);
		//NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
		//NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
	}
};


class dAnimationRagdollJoint::dRagdollMotor_3dof: public dRagdollMotor
{
	public:
	dRagdollMotor_3dof(dAnimationRagdollJoint* const owner, const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:dRagdollMotor(owner, pinAndPivotFrame0, pinAndPivotFrame1, child, parent)
	{
//		m_dof = 3;
#ifdef D_TEST_JOINT
//		m_dof = 0;
#endif
	}

	void Debug(dDebugDisplay* const debugDisplay) const
	{
//		dRagdollMotor::Debug(debugDisplay);
	}

	virtual void SubmitAngularConstraints(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
#ifdef D_TEST_JOINT
		return;
#else
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
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[1]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[2]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
		} else {
			// using small angular aproximation to get the joint angle;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[1][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[1]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[2][0]);
			NewtonUserJointSetRowAcceleration(m_joint, motorAccel[2]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_motorTorque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_motorTorque);
		}
#endif
	}
};



dAnimationRagdollJoint::dAnimationRagdollJoint(dRagdollMotorType type, const dMatrix& pinAndPivotInGlobalSpace, NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent)
	:dAnimationJoint(body, bindMarix, parent)
	,m_rowAccel(0.0f)
	,m_rows(0)
{
//	dJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
	dMatrix parentRollMatrix(dGetIdentityMatrix() * pinAndPivotInGlobalSpace);

	if (type == m_threeDof) {
		m_joint = new dRagdollMotor_3dof(this, pinAndPivotInGlobalSpace, parentRollMatrix, body, parent->GetBody());
	} else if (type == m_twoDof) {
		m_joint = new dRagdollMotor_2dof(this, pinAndPivotInGlobalSpace, parentRollMatrix, body, parent->GetBody());
	} else if (type == m_oneDof) {
		m_joint = new dRagdollMotor_1dof(this, pinAndPivotInGlobalSpace, parentRollMatrix, body, parent->GetBody());
	} else if (type == m_zeroDof) {
		m_joint = new dRagdollMotor_0dof(this, pinAndPivotInGlobalSpace, parentRollMatrix, body, parent->GetBody());
	} else {
		dAssert(0);
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

dAnimationRagdollJoint::~dAnimationRagdollJoint()
{
}

void dAnimationRagdollJoint::RigidBodyToStates()
{
	dAnimationJoint::RigidBodyToStates();
	m_proxyBody.SetForce(dVector(0.0f));
	m_proxyBody.SetTorque(dVector(0.0f));
}

void dAnimationRagdollJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dAssert(0);
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

void dAnimationRagdollJoint::UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const
{
	dAssert (0);
}

void dAnimationRagdollJoint::UpdateJointAcceleration()
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