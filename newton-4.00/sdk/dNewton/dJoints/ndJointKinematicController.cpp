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


#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointKinematicController.h"


#if 0
IMPLEMENT_CUSTOM_JOINT(ndJointKinematicController)



void ndJointKinematicController::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	dAssert (0);
}

void ndJointKinematicController::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dAssert (0);
}

void ndJointKinematicController::ResetAutoSleep ()
{
	ndBodyKinematicSetAutoSleep(GetBody0(), 0);
}


void ndJointKinematicController::Init (const dMatrix& matrix)
{
	CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);

	ndBodyKinematic* const body = GetBody0(); 
	m_autoSleepState = ndBodyKinematicGetAutoSleep(body) ? true : false;
	ndBodyKinematicSetSleepState(body, 0);

	SetControlMode(m_full6dof);
	CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);
	SetMaxLinearFriction(1.0f);
	SetMaxAngularFriction(1.0f);
	SetAngularViscuosFrictionCoefficient(1.0f);
	SetMaxSpeed(30.0f);
	SetMaxOmega(10.0f * 360.0f * dDegreeToRad);

	// set as soft joint
	SetSolverModel(3);
}

ndJointKinematicController::dControlModes ndJointKinematicController::GetControlMode() const
{
	return m_controlMode;
}

void ndJointKinematicController::SetControlMode(dControlModes mode)
{
	m_controlMode = mode;
}

void ndJointKinematicController::SetMaxSpeed(dFloat speedInMetersPerSeconds)
{
	m_maxSpeed = dAbs (speedInMetersPerSeconds);
}

void ndJointKinematicController::SetMaxOmega(dFloat speedInRadiansPerSeconds)
{
	m_maxOmega = dAbs (speedInRadiansPerSeconds);
}

void ndJointKinematicController::SetMaxLinearFriction(dFloat frictionForce)
{
	m_maxLinearFriction = dAbs (frictionForce);
}

void ndJointKinematicController::SetMaxAngularFriction(dFloat frictionTorque)
{
	m_maxAngularFriction = dAbs (frictionTorque);
}

void ndJointKinematicController::SetAngularViscuosFrictionCoefficient(dFloat coefficient)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	ndBodyKinematicGetMass(m_body0, &mass, &Ixx, &Iyy, &Izz);
	m_angularFrictionCoefficient = dAbs(coefficient) * dMax(Ixx, dMax(Iyy, Izz));
}

void ndJointKinematicController::CheckSleep() const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	dMatrix matrix (matrix1 * matrix0.Inverse());
	matrix.m_posit.m_w = 0.0f;
	if (matrix.m_posit.DotProduct3(matrix.m_posit) > dFloat(1.0e-6f)) {
		ndBodyKinematicSetSleepState(m_body0, 0);
	} else {
		switch (m_controlMode)
		{
			case m_full6dof:
			case m_linearPlusAngularFriction:
			{
				dFloat trace = matrix[0][0] * matrix[1][1] * matrix[2][2];
				if (trace < 0.9995f) {
					ndBodyKinematicSetSleepState(m_body0, 0);
				}
				break;
			} 

			case m_linearAndCone:
			{
				dFloat cosAngle = matrix1[0].DotProduct3(matrix0[0]);
				if (cosAngle > 0.998f) {
					ndBodyKinematicSetSleepState(m_body0, 0);
				}
				break;
			}

			case m_linearAndTwist:
			{
				 dFloat pitchAngle = 0.0f;
				 dFloat cosAngle = matrix1[0].DotProduct3(matrix0[0]);
				 if (cosAngle > 0.998f) {
					pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
				 } else {
					dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
					dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
					lateralDir = lateralDir.Normalize();
					dFloat coneAngle = dAcos(dClamp(cosAngle, dFloat(-1.0f), dFloat(1.0f)));
					dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
					dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
					pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
				}

				if (dAbs (pitchAngle) > (1.0 * dDegreeToRad)) {
					ndBodyKinematicSetSleepState(m_body0, 0);
				}
				break;
			}
		}
	}
}

void ndJointKinematicController::SetTargetRotation(const dQuaternion& rotation)
{
	m_localMatrix1 = dMatrix (rotation, m_localMatrix1.m_posit);
	m_localMatrix1.m_posit.m_w = 1.0f;
	CheckSleep();
}

void ndJointKinematicController::SetTargetPosit(const dVector& posit)
{
	m_localMatrix1.m_posit = posit;
	m_localMatrix1.m_posit.m_w = 1.0f;
	CheckSleep();
}

void ndJointKinematicController::SetTargetMatrix(const dMatrix& matrix)
{
	m_localMatrix1 = matrix;
	CheckSleep();
}

dMatrix ndJointKinematicController::GetTargetMatrix () const
{
	return m_localMatrix1;
}

dMatrix ndJointKinematicController::GetBodyMatrix () const
{
	dMatrix matrix0;
	ndBodyKinematicGetMatrix(m_body0, &matrix0[0][0]);
	return m_localMatrix0 * matrix0;
}

void ndJointKinematicController::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix1(GetTargetMatrix());
	if (m_body1) {
		ndBodyKinematicGetMatrix(m_body1, &matrix1[0][0]);
		matrix1 = GetTargetMatrix() * matrix1;
	}
	debugDisplay->DrawFrame(matrix1);
	debugDisplay->DrawFrame(GetBodyMatrix());
}

void ndJointKinematicController::SubmitLinearConstraints (const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
{
	dVector omega0(0.0f);
	dVector omega1(0.0f);
	dVector veloc0(0.0f);
	dVector veloc1(0.0f);
	dComplementaritySolver::dJacobian jacobian0;
	dComplementaritySolver::dJacobian jacobian1;

	ndBodyKinematicGetOmega(m_body0, &omega0[0]);
	ndBodyKinematicGetVelocity(m_body0, &veloc0[0]);
	if (m_body1) {
		ndBodyKinematicGetOmega(m_body1, &omega1[0]);
		ndBodyKinematicGetVelocity(m_body1, &veloc1[0]);
	}

	const dFloat invTimestep = 1.0f / timestep;

	const dFloat damp = 0.3f;
	const dFloat maxDistance = 2.0f * m_maxSpeed * timestep;
	for (int i = 0; i < 3; i++) {
		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1[i][0]);
		NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

		dVector pointPosit(matrix0.m_posit * jacobian0.m_linear + matrix1.m_posit * jacobian1.m_linear);
		dVector pointVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);

		dFloat dist = pointPosit.m_x + pointPosit.m_y + pointPosit.m_z;
		dFloat speed = pointVeloc.m_x + pointVeloc.m_y + pointVeloc.m_z;

		dFloat v = m_maxSpeed * dSign(dist);
		if ((dist < maxDistance) && (dist > -maxDistance)) {
			v = damp * dist * invTimestep;
		}
		dAssert(dAbs(v) <= m_maxSpeed);

		dFloat relAccel = (v + speed) * invTimestep;
		NewtonUserJointSetRowAcceleration(m_joint, -relAccel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxLinearFriction);
	}
}

void ndJointKinematicController::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearConstraints (matrix0, matrix1, timestep);

	if (m_controlMode != m_linear) {
		dVector omega0(0.0f);
		dVector omega1(0.0f);
		dVector veloc0(0.0f);
		dVector veloc1(0.0f);
		dComplementaritySolver::dJacobian jacobian0;
		dComplementaritySolver::dJacobian jacobian1;

		ndBodyKinematicGetOmega(m_body0, &omega0[0]);
		ndBodyKinematicGetVelocity(m_body0, &veloc0[0]);
		if (m_body1) {
			ndBodyKinematicGetOmega(m_body1, &omega1[0]);
			ndBodyKinematicGetVelocity(m_body1, &veloc1[0]);
		}

		const dFloat damp = 0.3f;
		const dFloat invTimestep = 1.0f / timestep;
		if (m_controlMode == m_linearPlusAngularFriction) {
			dFloat omegaMag2 = omega0.DotProduct3(omega0);
			dFloat angularFriction = m_maxAngularFriction + m_angularFrictionCoefficient * omegaMag2;
			for (int i = 0; i < 3; i++) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[i][0]);
				NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

				dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
				dFloat relSpeed = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
				dFloat relAccel = relSpeed * invTimestep;

				NewtonUserJointSetRowAcceleration(m_joint, -relAccel);
				NewtonUserJointSetRowMinimumFriction(m_joint, -angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, angularFriction);
			}
		} else {

			dFloat pitchAngle = 0.0f;
			const dFloat maxAngle = 2.0f * m_maxOmega * timestep;
			dFloat cosAngle = matrix1[0].DotProduct3(matrix0[0]);
			if (cosAngle > dFloat (0.998f)) {
				if ((m_controlMode == m_linearAndCone) || (m_controlMode == m_full6dof)) {
					for (int i = 1; i < 3; i++) {
						dFloat coneAngle = -damp * CalculateAngle(matrix0[0], matrix1[0], matrix1[i]);
						NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[i][0]);
						NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

						dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
						dFloat relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;

						dFloat w = m_maxOmega * dSign(coneAngle);
						if ((coneAngle < maxAngle) && (coneAngle > -maxAngle)) {
							w = damp * coneAngle * invTimestep;
						}

						dAssert(dAbs(w) <= m_maxOmega);
						dFloat relAlpha = (w + relOmega) * invTimestep;

						NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
						NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
						NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
					}
				}
				pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);

			} else {

				dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
				dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
				lateralDir = lateralDir.Normalize();
				dFloat coneAngle = dAcos(dClamp(cosAngle, dFloat(-1.0f), dFloat(1.0f)));
				dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);

				if ((m_controlMode == m_linearAndCone) || (m_controlMode == m_full6dof)) {
					NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
					NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

					dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
					dFloat relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;

					dFloat w = m_maxOmega * dSign(coneAngle);
					if ((coneAngle < maxAngle) && (coneAngle > -maxAngle)) {
						w = damp * coneAngle * invTimestep;
					}
					dFloat relAlpha = (w + relOmega) * invTimestep;

					NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
					NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
					NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);


					dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
					NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
					NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
					pointOmega = omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular;
					relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
					relAlpha = relOmega * invTimestep;

					NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
					NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
					NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
				}

				dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
				pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
			}

			if ((m_controlMode == m_linearAndTwist) || (m_controlMode == m_full6dof)) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
				NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
				dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
				dFloat relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;

				dFloat w = m_maxOmega * dSign(pitchAngle);
				if ((pitchAngle < maxAngle) && (pitchAngle > -maxAngle)) {
					w = damp * pitchAngle * invTimestep;
				}
				dAssert(dAbs(w) <= m_maxOmega);
				dFloat relAlpha = (w + relOmega) * invTimestep;

				NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
			}
		}
	}
}

#endif


ndJointKinematicController::ndJointKinematicController(ndBodyKinematic* const referenceBody, ndBodyKinematic* const body, const dVector& attachmentPointInGlobalSpace)
	:ndJointBilateralConstraint(referenceBody, body, dMatrix(attachmentPointInGlobalSpace))
{
	dAssert(GetKinematicBody1() == body);
	dMatrix matrix(body->GetMatrix());
	matrix.m_posit = attachmentPointInGlobalSpace;
	matrix.m_posit.m_w = dFloat32(1.0f);
	Init(matrix);
}

ndJointKinematicController::ndJointKinematicController(ndBodyKinematic* const referenceBody, ndBodyKinematic* const body, const dMatrix& attachmentMatrixInGlobalSpace)
	:ndJointBilateralConstraint(referenceBody, body, attachmentMatrixInGlobalSpace)
{
	Init(attachmentMatrixInGlobalSpace);
}

ndJointKinematicController::~ndJointKinematicController()
{
	m_body1->SetAutoSleep(m_autoSleepState);
}

void ndJointKinematicController::Init(const dMatrix& globalMatrix)
{
	CalculateLocalMatrix(globalMatrix, m_localMatrix0, m_localMatrix1);

	m_autoSleepState = m_body1->GetAutoSleep();
	m_body1->SetAutoSleep(false);

	//SetControlMode(m_full6dof);
	//CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);
	//SetMaxLinearFriction(1.0f);
	//SetMaxAngularFriction(1.0f);
	//SetAngularViscuosFrictionCoefficient(1.0f);
	//SetMaxSpeed(30.0f);
	//SetMaxOmega(10.0f * 360.0f * dDegreeToRad);
	//
	//// set as soft joint
	//SetSolverModel(3);
}
