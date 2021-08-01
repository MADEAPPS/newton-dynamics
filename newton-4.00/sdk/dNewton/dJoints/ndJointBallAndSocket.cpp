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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointBallAndSocket.h"

#define D_BALL_AND_SOCKED_MAX_CONE_ANGLE dFloat32 (120.0f * dDegreeToRad)

ndJointBallAndSocket::ndJointBallAndSocket(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	//,m_twistAngle(0.0f)
	//,m_minTwistAngle(-180.0f * dDegreeToRad)
	//,m_maxTwistAngle(180.0f * dDegreeToRad)
	//,m_maxConeAngle(D_BALL_AND_SOCKED_MAX_CONE_ANGLE)
	//,m_coneFriction(0.0f)
	//,m_twistFriction(0.0f)
	,m_maxConeAngle( dFloat32 (1.0e10f))
	,m_minTwistAngle(-dFloat32(1.0e10f))
	,m_maxTwistAngle( dFloat32(1.0e10f))
{
	//CalculateLocalMatrix(pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

ndJointBallAndSocket::~ndJointBallAndSocket()
{
}


#if 0
ndJointBallAndSocket::ndJointBallAndSocket(const dMatrix& pinAndPivotFrame0, const dMatrix& pinAndPivotFrame1, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	//:dCustomJoint(6, child, parent)
	,m_twistAngle(0.0f)
	,m_minTwistAngle(-180.0f * dDegreeToRad)
	,m_maxTwistAngle(180.0f * dDegreeToRad)
	,m_maxConeAngle(D_BALL_AND_SOCKED_MAX_CONE_ANGLE)
	,m_coneFriction(0.0f)
	,m_twistFriction(0.0f)
{
	dMatrix	dummy;
	CalculateLocalMatrix(pinAndPivotFrame0, m_localMatrix0, dummy);
	CalculateLocalMatrix(pinAndPivotFrame1, dummy, m_localMatrix1);
}

void ndJointBallAndSocket::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_twistAngle, sizeof(dAngularIntegration));
	callback(userData, &m_minTwistAngle, sizeof(dFloat32));
	callback(userData, &m_maxTwistAngle, sizeof(dFloat32));
	callback(userData, &m_maxConeAngle, sizeof(dFloat32));
	callback(userData, &m_coneFriction, sizeof(dFloat32));
	callback(userData, &m_twistFriction, sizeof(dFloat32));
}

void ndJointBallAndSocket::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);

	callback(userData, &m_twistAngle, sizeof(dAngularIntegration));
	callback(userData, &m_minTwistAngle, sizeof(dFloat32));
	callback(userData, &m_maxTwistAngle, sizeof(dFloat32));
	callback(userData, &m_maxConeAngle, sizeof(dFloat32));
	callback(userData, &m_coneFriction, sizeof(dFloat32));
	callback(userData, &m_twistFriction, sizeof(dFloat32));
}

void ndJointBallAndSocket::EnableTwist(bool state)
{
	m_options.m_option3 = state;
}


void ndJointBallAndSocket::SetTwistFriction (dFloat32 frictionTorque)
{
	m_twistFriction = dAbs (frictionTorque);
}

dFloat32 ndJointBallAndSocket::GetTwistFriction (dFloat32 frictionTorque) const
{
	return m_twistFriction;
}

void ndJointBallAndSocket::SetConeLimits(dFloat32 maxAngle)
{
	m_maxConeAngle = dMin (dAbs (maxAngle), D_BALL_AND_SOCKED_MAX_CONE_ANGLE);
}

void ndJointBallAndSocket::SetConeFriction(dFloat32 frictionTorque)
{
	m_coneFriction = dAbs (frictionTorque);
}

dFloat32 ndJointBallAndSocket::GetConeFriction(dFloat32 frictionTorque) const
{
	return m_coneFriction;
}


void ndJointBallAndSocket::SubmitTwistAngle(const dVector& pin, dFloat32 angle, dFloat32 timestep)
{
	if ((m_maxTwistAngle - m_minTwistAngle) < (2.0f * dDegreeToRad)) {
		NewtonUserJointAddAngularRow(m_joint, -angle, &pin[0]);
	} else {
		dFloat32 restoringOmega = 0.25f;
		const dFloat32 step = dMax(dAbs(angle * timestep), dFloat32(5.0f * dDegreeToRad));
		if (angle < m_minTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat32 invtimestep = 1.0f / timestep;
			const dFloat32 error0 = angle - m_minTwistAngle;
			const dFloat32 error1 = error0 + restoringOmega * timestep;
			if (error1 > 0.0f) {
				restoringOmega = -error0 * invtimestep;
			}
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + restoringOmega * invtimestep;
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
		} else if (angle >= m_maxTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat32 invtimestep = 1.0f / timestep;
			const dFloat32 error0 = angle - m_maxTwistAngle;
			const dFloat32 error1 = error0 - restoringOmega * timestep;
			if (error1 < 0.0f) {
				restoringOmega = error0 * invtimestep;
			}
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - restoringOmega / timestep;
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
		} else if ((angle - step) <= m_minTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
		} else if ((angle + step) >= m_maxTwistAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
		} else if (m_twistFriction > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &pin[0]);
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_twistFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_twistFriction);
		}
	}
}

void ndJointBallAndSocket::SubmitAngularAxisCartisianApproximation(const dMatrix& matrix0, const dMatrix& matrix1, dFloat32 timestep)
{
	if (m_options.m_option4) {
		// two rows to restrict rotation around around the parent coordinate system
		//dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct3(matrix0.m_front), dFloat32(-1.0f), dFloat32(1.0f)));
		const dFloat32 angleError = GetMaxAngleError();
		dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		NewtonUserJointAddAngularRow(m_joint, angle0, &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		//NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
		if (dAbs(angle0) > angleError) {
			const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle0 / (timestep * timestep);
			NewtonUserJointSetRowAcceleration(m_joint, alpha);
		}

		dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		NewtonUserJointAddAngularRow(m_joint, angle1, &matrix1.m_right[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		//NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
		if (dAbs(angle1) > angleError) {
			const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle1 / (timestep * timestep);
			NewtonUserJointSetRowAcceleration(m_joint, alpha);
		}
	}

	if (m_options.m_option3) {
		dFloat32 pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		SubmitTwistAngle(matrix0.m_front, pitchAngle, timestep);
	}
}

void ndJointBallAndSocket::SubmitAngularAxis(const dMatrix& matrix0, const dMatrix& matrix1, dFloat32 timestep)
{
	dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
	dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
	lateralDir = lateralDir.Normalize();
	dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct3(matrix0.m_front), dFloat32(-1.0f), dFloat32(1.0f)));
	dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);

	if (m_options.m_option4) {
		const dFloat32 step = dMax(dAbs(coneAngle * timestep), dFloat32(5.0f * dDegreeToRad));
		if (coneAngle > m_maxConeAngle) {
			dFloat32 restoringOmega = 0.25f;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			const dFloat32 invtimestep = 1.0f / timestep;
			const dFloat32 error0 = coneAngle - m_maxConeAngle;
			const dFloat32 error1 = error0 - restoringOmega * timestep;
			if (error1 < 0.0f) {
				restoringOmega = error0 * invtimestep;
			}
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - restoringOmega / timestep;
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			//NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);

		} else if ((coneAngle + step) >= m_maxConeAngle) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			//NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
		} else if (m_coneFriction > 0.0f) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
			const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			//NewtonUserJointSetRowStiffness(m_joint, m_coneStiffness);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);

			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_coneFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_coneFriction);
		}
	}

	if (m_options.m_option3) {
		dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
		dFloat32 pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
		SubmitTwistAngle(matrix0.m_front, pitchAngle, timestep);
	}
}

void ndJointBallAndSocket::SubmitConstraints(dFloat32 timestep, dInt32 threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearRows(0x07, matrix0, matrix1);

	if (m_options.m_option3 || m_options.m_option4) {
		dFloat32 cosAngleCos = matrix1.m_front.DotProduct3(matrix0.m_front);
		if (cosAngleCos >= dFloat32(0.998f)) {
			// special case where the front axis are almost aligned
			// solve by using Cartesian approximation
			SubmitAngularAxisCartisianApproximation(matrix0, matrix1, timestep);
		} else {
			SubmitAngularAxis(matrix0, matrix1, timestep);
		}
	}
}
#endif

void ndJointBallAndSocket::SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle)
{
	m_minTwistAngle = -dAbs(minAngle);
	m_maxTwistAngle = dAbs(maxAngle);
}

void ndJointBallAndSocket::GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}


dFloat32 ndJointBallAndSocket::GetMaxConeAngle() const
{
	return m_maxConeAngle;
}

void ndJointBallAndSocket::SetMaxConeAngle(dFloat32 maxConeAngle)
{
	m_maxConeAngle = dAbs (maxConeAngle);
}

void ndJointBallAndSocket::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	const dInt32 subdiv = 8;
	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;
	dFloat32 cosAngleCos = coneDir0.DotProduct(coneDir1).GetScalar();
	dMatrix coneRotation(dGetIdentityMatrix());
	if (cosAngleCos < dFloat32(0.9999f))
	{
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat32 mag2 = lateralDir.DotProduct(lateralDir).GetScalar();
		if (mag2 > dFloat32 (1.0e-4f)) 
		{
			lateralDir = lateralDir.Scale(dFloat32 (1.0f) / dSqrt(mag2));
			coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat32(-1.0f), dFloat32(1.0f)))), matrix1.m_posit);
		}
		else 
		{
			lateralDir = matrix0.m_up.Scale(-dFloat32 (1.0f));
			coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat32 (180.0f) * dDegreeToRad), matrix1.m_posit);
		}
	}
	else if (cosAngleCos < -dFloat32 (0.9999f)) 
	{
		coneRotation[0][0] = dFloat32(-1.0f);
		coneRotation[1][1] = dFloat32(-1.0f);
	}
	
	const dFloat32 radius = debugCallback.m_debugScale;
	dVector arch[subdiv + 1];
	
	// show twist angle limits
	dFloat32 deltaTwist = m_maxTwistAngle - m_minTwistAngle;
	if ((deltaTwist > dFloat32(1.0e-3f)) && (deltaTwist < dFloat32 (2.0f) * dPi))
	{ 
		dMatrix pitchMatrix(matrix1 * coneRotation);
		pitchMatrix.m_posit = matrix1.m_posit;
	
		dVector point(dFloat32(0.0f), dFloat32(radius), dFloat32(0.0f), dFloat32(0.0f));
	
		dFloat32 angleStep = dMin(m_maxTwistAngle - m_minTwistAngle, dFloat32(2.0f * dPi)) / subdiv;
		dFloat32 angle0 = m_minTwistAngle;
	
		dVector color(dFloat32 (0.4f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		for (dInt32 i = 0; i <= subdiv; i++) 
		{
			arch[i] = pitchMatrix.TransformVector(dPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(pitchMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}
	
		for (dInt32 i = 0; i < subdiv; i++) 
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
	
	// show cone angle limits
	if ((m_maxConeAngle > dFloat32 (0.0f)) && (m_maxConeAngle < D_BALL_AND_SOCKED_MAX_CONE_ANGLE)) 
	{
		dVector color(dFloat32(0.3f), dFloat32(0.8f), dFloat32(0.0f), dFloat32(0.0f));
		dVector point(radius * dCos(m_maxConeAngle), radius * dSin(m_maxConeAngle), 0.0f, 0.0f);
		dFloat32 angleStep = dPi * dFloat32(2.0f) / subdiv;
	
		dFloat32 angle0 = dFloat32 (0.0f);
		for (dInt32 i = 0; i <= subdiv; i++) 
		{
			dVector conePoint(dPitchMatrix(angle0).RotateVector(point));
			dVector p(matrix1.TransformVector(conePoint));
			arch[i] = p;
			debugCallback.DrawLine(matrix1.m_posit, p, color);
			angle0 += angleStep;
		}
	
		for (dInt32 i = 0; i < subdiv; i++) 
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

void ndJointBallAndSocket::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
}


