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
#include "dAnimationJointRoot.h"
#include "dAnimationRagdollEffector.h"

dAnimationRagDollEffector::dAnimationRagDollEffector(dAnimationJoint* const joint)
	:dAnimationLoopJoint(joint->GetProxyBody(), joint->GetRoot()->GetStaticWorld())
	,m_localMatrix(dGetIdentityMatrix())
	,m_targetMatrix(dGetIdentityMatrix())
	,m_maxLinearSpeed(0.1f * 120.0f)
	,m_maxAngularSpeed(3.0f * dDegreeToRad * 120.0f)
	,m_linearFriction(100000.0f)
	,m_angularFriction(100000.0f)
{
	dMatrix matrix;
	NewtonBody* const body = joint->GetBody();
	NewtonBodyGetMatrix(body, &matrix[0][0]);

	m_localMatrix.m_posit = matrix.m_posit;
	m_localMatrix = m_localMatrix * matrix.Inverse();
}

dMatrix dAnimationRagDollEffector::GetMatrix() const
{
	dMatrix matrix;
	NewtonBodyGetMatrix(GetOwner0()->m_owner->GetBody(), &matrix[0][0]);
	return  m_localMatrix * matrix;
}

void dAnimationRagDollEffector::SetTarget(const dMatrix& targetMatrix)
{
/*
	dMatrix matrix;
	NewtonBodyGetMatrix(GetOwner0()->m_owner->GetBody(), &matrix[0][0]);
	matrix = m_localMatrix * matrix;
	//m_targetMatrix.m_posit = matrix.m_posit;

	//static dMatrix xxx (dPitchMatrix (95.0f * dDegreeToRad) * dYawMatrix (65.0f * dDegreeToRad) * dRollMatrix (85.0f * dDegreeToRad) * m_targetMatrix);
	//static dMatrix xxx (dPitchMatrix (0.0f * dDegreeToRad) * dYawMatrix (0.0f * dDegreeToRad) * dRollMatrix (0.0f * dDegreeToRad) * m_targetMatrix);
	m_targetMatrix = dPitchMatrix(0.0f * dDegreeToRad) * dYawMatrix(0.0f * dDegreeToRad) * dRollMatrix(0.0f * dDegreeToRad);
	m_targetMatrix.m_posit = matrix.m_posit;
*/
	m_targetMatrix = targetMatrix;
}

dFloat dAnimationRagDollEffector::ClipParam(dFloat value, dFloat maxValue) const
{
	if (value > maxValue) {
		value = maxValue;
	} else if (value < -maxValue) {
		value = -maxValue;
	} else {
		value *= 0.3f;
	}
	return value;
}

void dAnimationRagDollEffector::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	const dMatrix& matrix1 = m_targetMatrix;
	dMatrix matrix0(m_localMatrix * m_state0->GetMatrix());

	const dVector relPosit(matrix1.UnrotateVector(matrix1.m_posit - matrix0.m_posit));
	const dVector relPositDir = relPosit.Scale(1.0f / dSqrt(relPosit.DotProduct3(relPosit) + 1.0e-6f));

	int dofCount = 0;
	dAssert(m_maxLinearSpeed >= 0.0f);
	const dFloat invTimestep = constraintParams->m_timestepInv;
	const dFloat linearStep = m_maxLinearSpeed * constraintParams->m_timestep;
	for (int i = 0; i < 3; i++) {
		AddLinearRowJacobian(constraintParams, matrix0.m_posit, matrix1[i]);
		const dFloat stopAccel = CalculateRowZeroAccelaration(constraintParams);
		dFloat speed = ClipParam(relPosit[i], linearStep * dAbs(relPositDir[i])) * invTimestep;
		dFloat relAccel = speed * invTimestep + stopAccel;
		constraintParams->m_normalIndex[dofCount] = 0;
		constraintParams->m_jointAccel[dofCount] = relAccel;
		constraintParams->m_jointLowFrictionCoef[dofCount] = -m_linearFriction;
		constraintParams->m_jointHighFrictionCoef[dofCount] = m_linearFriction;
		dofCount++;
	}

	const dVector& coneDir0 = matrix0.m_front;
	const dVector& coneDir1 = matrix1.m_front;

	const dFloat angleStep = m_maxAngularSpeed * constraintParams->m_timestep;
	dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
	if (cosAngleCos < 0.9999f) {
		dMatrix coneRotation(dGetIdentityMatrix());
		dVector lateralDir(coneDir1.CrossProduct(coneDir0));
		dFloat mag2 = lateralDir.DotProduct3(lateralDir);
		if (mag2 > 1.0e-4f) {
			lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
			coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
		} else {
			lateralDir = matrix0.m_up.Scale(-1.0f);
			coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat(180.0f) * dDegreeToRad), matrix1.m_posit);
		}

		dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
		dAssert(dAbs(pitchMatrix[0][0] - dFloat(1.0f)) < dFloat(1.0e-3f));
		dAssert(dAbs(pitchMatrix[0][1]) < dFloat(1.0e-3f));
		dAssert(dAbs(pitchMatrix[0][2]) < dFloat(1.0e-3f));

		dFloat pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
		dFloat coneAngle = -dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)));
		//dTrace(("cone:%f pitch:%f\n", coneAngle * dRadToDegree, pitchAngle * dRadToDegree));

		dVector angleDir(pitchAngle, coneAngle, dFloat(0.0f), dFloat(0.0f));
		angleDir = angleDir.Scale(1.0f / dSqrt(angleDir.DotProduct3(angleDir) + 1.0e-6f));

		{
			AddAngularRowJacobian(constraintParams, matrix1[0], 0.0f);
			const dFloat stopAlpha = CalculateRowZeroAccelaration(constraintParams);
			dFloat omega = ClipParam(pitchAngle, angleStep * dAbs(angleDir[0])) * invTimestep;
			dFloat relAlpha = omega * invTimestep + stopAlpha;
			constraintParams->m_normalIndex[dofCount] = 0;
			constraintParams->m_jointAccel[dofCount] = relAlpha;
			constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
			constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
			dofCount++;
		}

		{
			AddAngularRowJacobian(constraintParams, lateralDir, 0.0f);
			const dFloat stopAlpha = CalculateRowZeroAccelaration(constraintParams);
			dFloat omega = ClipParam(coneAngle, angleStep * dAbs(angleDir[1])) * invTimestep;
			dFloat relAlpha = omega * invTimestep + stopAlpha;
			constraintParams->m_normalIndex[dofCount] = 0;
			constraintParams->m_jointAccel[dofCount] = relAlpha;
			constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
			constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
			dofCount++;
		}

		{
			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
			AddAngularRowJacobian(constraintParams, sideDir, 0.0f);
			const dFloat stopAlpha = CalculateRowZeroAccelaration(constraintParams);
			constraintParams->m_normalIndex[dofCount] = 0;
			constraintParams->m_jointAccel[dofCount] = stopAlpha;
			constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
			constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
			dofCount++;
		}
	} else {

		// using small angular aproximation to get the joint angle;
		{
			AddAngularRowJacobian(constraintParams, matrix1[0], 0.0f);
			const dFloat stopAlpha = CalculateRowZeroAccelaration(constraintParams);
			dFloat angle = CalculateAngle(&matrix0[1][0], &matrix1[1][0], &matrix0[0][0]);
			dFloat omega = ClipParam(angle, angleStep) * invTimestep;
			dFloat relAlpha = omega * invTimestep + stopAlpha;
			constraintParams->m_normalIndex[dofCount] = 0;
			constraintParams->m_jointAccel[dofCount] = relAlpha;
			constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
			constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
			dofCount++;
		}

		{
			AddAngularRowJacobian(constraintParams, matrix1[1], 0.0f);
			const dFloat stopAlpha = CalculateRowZeroAccelaration(constraintParams);
			dFloat angle = matrix1[2].DotProduct3(matrix1[0]);
			dFloat omega = ClipParam(angle, angleStep) * invTimestep;
			dFloat relAlpha = omega * invTimestep + stopAlpha;
			constraintParams->m_normalIndex[dofCount] = 0;
			constraintParams->m_jointAccel[dofCount] = relAlpha;
			constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
			constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
			dofCount++;
		}

		{
			AddAngularRowJacobian(constraintParams, matrix1[2], 0.0f);
			const dFloat stopAlpha = CalculateRowZeroAccelaration(constraintParams);
			dFloat angle = matrix1[1].DotProduct3(matrix1[0]);
			dFloat omega = ClipParam(angle, angleStep) * invTimestep;
			dFloat relAlpha = omega * invTimestep + stopAlpha;
			constraintParams->m_normalIndex[dofCount] = 0;
			constraintParams->m_jointAccel[dofCount] = relAlpha;
			constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
			constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
			dofCount++;
		}
	}

	m_dof = dofCount;
	m_count = dofCount;
	constraintParams->m_count = dofCount;
}

