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


// dCustomKinematicController.cpp: implementation of the dCustomKinematicController class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomKinematicController.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomKinematicController)

dCustomKinematicController::dCustomKinematicController(NewtonBody* const body, const dVector& handleInGlobalSpace, NewtonBody* const referenceBody)
	:dCustomJoint(6, body, referenceBody)
{
	dMatrix matrix;
	dAssert (GetBody0() == body);
	NewtonBodyGetMatrix (body, &matrix[0][0]);
	matrix.m_posit = handleInGlobalSpace;
	Init (matrix);
}

dCustomKinematicController::dCustomKinematicController (NewtonBody* const body, const dMatrix& attachmentMatrixInGlobalSpace, NewtonBody* const referenceBody)
	:dCustomJoint(6, body, referenceBody)
{
	Init (attachmentMatrixInGlobalSpace);
}

void dCustomKinematicController::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	dAssert (0);
}

void dCustomKinematicController::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dAssert (0);
}

void dCustomKinematicController::ResetAutoSleep ()
{
	NewtonBodySetAutoSleep(GetBody0(), 0);
}

dCustomKinematicController::~dCustomKinematicController()
{
	NewtonBodySetAutoSleep (m_body0, m_autoSleepState);
}

void dCustomKinematicController::Init (const dMatrix& matrix)
{
	CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);

	NewtonBody* const body = GetBody0(); 
	m_autoSleepState = NewtonBodyGetAutoSleep(body) ? true : false;
	NewtonBodySetSleepState(body, 0);

	SetPickMode(1);
	SetAsLinearAndAngular();
	CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);
	SetMaxLinearFriction(1.0f);
	SetMaxAngularFriction(1.0f);
	SetAngularViscuosFrictionCoefficient(1.0f);
	SetMaxSpeed(30.0f);
	SetMaxOmega(10.0f * 360.0f * dDegreeToRad);

	// set as soft joint
	SetSolverModel(3);
}

void dCustomKinematicController::SetAsLinear()
{
	m_controlAngularDof = false;
}

void dCustomKinematicController::SetAsLinearAndAngular()
{
	m_controlAngularDof = 1;
}

void dCustomKinematicController::SetPickMode (int mode)
{
	m_pickingMode = char (dClamp (mode, 0, 2));
}

void dCustomKinematicController::SetMaxSpeed(dFloat speedInMetersPerSeconds)
{
	m_maxSpeed = dAbs (speedInMetersPerSeconds);
}

void dCustomKinematicController::SetMaxOmega(dFloat speedInRadiansPerSeconds)
{
	m_maxOmega = dAbs (speedInRadiansPerSeconds);
}

void dCustomKinematicController::SetMaxLinearFriction(dFloat frictionForce)
{
	m_maxLinearFriction = dAbs (frictionForce);
}

void dCustomKinematicController::SetMaxAngularFriction(dFloat frictionTorque)
{
	m_maxAngularFriction = dAbs (frictionTorque);
}

void dCustomKinematicController::SetAngularViscuosFrictionCoefficient(dFloat coefficient)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBodyGetMass(m_body0, &mass, &Ixx, &Iyy, &Izz);
	m_angularFrictionCoefficient = dAbs(coefficient) * dMax(Ixx, dMax(Iyy, Izz));
}

void dCustomKinematicController::CheckSleep() const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	dMatrix matrix (matrix1 * matrix0.Inverse());
	matrix.m_posit.m_w = 0.0f;
	if (matrix.m_posit.DotProduct3(matrix.m_posit) > 1.0e-6f) {
		NewtonBodySetSleepState(m_body0, 0);
	} else if (m_controlAngularDof) {
		dFloat32 trace = matrix[0][0] * matrix[1][1] * matrix[2][2];
		if (trace < 0.9995f) {
			NewtonBodySetSleepState(m_body0, 0);
		}
	}
}

void dCustomKinematicController::SetTargetRotation(const dQuaternion& rotation)
{
	m_localMatrix1 = dMatrix (rotation, m_localMatrix1.m_posit);
	m_localMatrix1.m_posit.m_w = 1.0f;
	CheckSleep();
}

void dCustomKinematicController::SetTargetPosit(const dVector& posit)
{
	m_localMatrix1.m_posit = posit;
	m_localMatrix1.m_posit.m_w = 1.0f;
	CheckSleep();
}

void dCustomKinematicController::SetTargetMatrix(const dMatrix& matrix)
{
	m_localMatrix1 = matrix;
	CheckSleep();
}

dMatrix dCustomKinematicController::GetTargetMatrix () const
{
	return m_localMatrix1;
}

dMatrix dCustomKinematicController::GetBodyMatrix () const
{
	dMatrix matrix0;
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);
	return m_localMatrix0 * matrix0;
}

void dCustomKinematicController::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix1(GetTargetMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);
		matrix1 = GetTargetMatrix() * matrix1;
	}
	debugDisplay->DrawFrame(matrix1);
	debugDisplay->DrawFrame(GetBodyMatrix());
}

void dCustomKinematicController::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dVector omega1(0.0f);
	dVector veloc0(0.0f);
	dVector veloc1(0.0f);
	dComplementaritySolver::dJacobian jacobian0;
	dComplementaritySolver::dJacobian jacobian1;

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetVelocity(m_body0, &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
		NewtonBodyGetVelocity(m_body1, &veloc1[0]);
	}

	const dFloat invTimestep = 1.0f / timestep;
	CalculateGlobalMatrix(matrix0, matrix1);

	const dFloat damp = 0.3f;
	const dFloat maxDistance = 2.0f * m_maxSpeed * timestep;
	for (int i = 0; i < 3; i++) {
		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow(m_joint, &matrix1.m_posit[0], &matrix1.m_posit[0], &matrix1[i][0]);
		NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

		dVector pointPosit (matrix0.m_posit * jacobian0.m_linear + matrix1.m_posit * jacobian1.m_linear);
		dVector pointVeloc (veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);

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

	if (m_controlAngularDof) {
		if (m_pickingMode != 1) {
			dFloat omegaMag2 = omega0.DotProduct3(omega0);
			dFloat angularFriction = m_maxAngularFriction + m_angularFrictionCoefficient * omegaMag2;
			//dTrace(("angular friction %f %f %f\n", omegaMag2, m_maxAngularFriction, angularFriction));
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
			if (cosAngle > 0.998f) {
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
				pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);

			} else {

				dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
				dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
				lateralDir = lateralDir.Normalize();
				dFloat coneAngle = dAcos(dClamp(cosAngle, dFloat(-1.0f), dFloat(1.0f)));
				dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);

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

				dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
				pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
			}

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


