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


// dCustom6DOF.cpp: implementation of the dCustom6DOF class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustom6DOF.h"

//dInitRtti(dCustom6DOF);

IMPLEMENT_CUSTOM_JOINT(dCustom6DOF);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH					50.0f
#define D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION	0.5f
#define D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION	10.0f

dCustom6DOF::dCustom6DOF (const dMatrix& pinsAndPivotChildFrame, const dMatrix& pinsAndPivotParentFrame___, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f)
	,m_maxLinearLimits(0.0f)
	,m_minAngularLimits(0.0f)
	,m_maxAngularLimits(0.0f)
	,m_pitch()
	,m_yaw()
	,m_roll()
{
	CalculateLocalMatrix (pinsAndPivotChildFrame, m_localMatrix0, m_localMatrix1);

}

dCustom6DOF::~dCustom6DOF()
{
}

dCustom6DOF::dCustom6DOF (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomJoint(child, parent, callback, userData)
{
	callback (userData, &m_minLinearLimits, sizeof (dVector));
	callback (userData, &m_maxLinearLimits, sizeof (dVector));
	callback (userData, &m_minAngularLimits, sizeof (dVector));
	callback (userData, &m_maxAngularLimits, sizeof (dVector));
	callback (userData, &m_pitch, sizeof (AngularIntegration));
	callback (userData, &m_yaw, sizeof (AngularIntegration));
	callback (userData, &m_roll, sizeof (AngularIntegration));
}

void dCustom6DOF::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_minLinearLimits, sizeof (dVector));
	callback(userData, &m_maxLinearLimits, sizeof (dVector));
	callback(userData, &m_minAngularLimits, sizeof (dVector));
	callback(userData, &m_maxAngularLimits, sizeof (dVector));
	callback(userData, &m_pitch, sizeof (AngularIntegration));
	callback(userData, &m_yaw, sizeof (AngularIntegration));
	callback(userData, &m_roll, sizeof (AngularIntegration));
}


void dCustom6DOF::SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minLinearLimits[i] =  (dAbs (minLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : minLinearLimits[i];
		m_maxLinearLimits[i] =  (dAbs (maxLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : maxLinearLimits[i];
	}

}

void dCustom6DOF::SetAngularLimits (const dVector& minAngularLimits, const dVector& maxAngularLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minAngularLimits[i] =  (dAbs (minAngularLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : minAngularLimits[i];
		m_maxAngularLimits[i] =  (dAbs (maxAngularLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : maxAngularLimits[i];
	}
}

void dCustom6DOF::GetLinearLimits (dVector& minLinearLimits, dVector& maxLinearLimits)
{
	minLinearLimits = m_minLinearLimits;
	maxLinearLimits = m_maxLinearLimits;
}

void dCustom6DOF::GetAngularLimits (dVector& minAngularLimits, dVector& maxAngularLimits)
{
	minAngularLimits = m_minAngularLimits;
	maxAngularLimits = m_maxAngularLimits;
}


void dCustom6DOF::GetInfo (NewtonJointRecord* const info) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	dFloat dist;

	strcpy (info->m_descriptionType, GetTypeName());

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	dVector p0 (matrix0.m_posit);
	dVector p1 (matrix1.m_posit);
	dVector dp (p0 - p1);
	for (int i = 0; i < 3; i ++) {
		if (!((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f))) {
			p1 += matrix1[i].Scale (dp.DotProduct3(matrix1[i]));
		}
	}

	for (int i = 0; i < 3; i ++) {
		if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
			info->m_minLinearDof[i] = 0.0f;
			info->m_maxLinearDof[i] = 0.0f;
		} else {
			dist = dp.DotProduct3(matrix1[i]);
			info->m_maxLinearDof[i] = m_maxLinearLimits[i] - dist;
			info->m_minLinearDof[i] = m_minLinearLimits[i] - dist;
		}
	}

	dVector eulerAngles (m_pitch.GetAngle(), m_yaw.GetAngle(), m_roll.GetAngle(), 0.0f);
	for (int i = 0; i < 3; i ++) {
		if ((m_minAngularLimits[i] == 0.0f) && (m_maxAngularLimits[i] == 0.0f)) {
			info->m_minAngularDof[i] = 0.0f;
			info->m_maxAngularDof[i] = 0.0f;
		} else {
			info->m_maxAngularDof[i] = (m_maxAngularLimits[i] - eulerAngles[i]) * 180.0f / 3.141592f;
			info->m_minAngularDof[i] = (m_minAngularLimits[i] - eulerAngles[i]) * 180.0f / 3.141592f;
		}
	}

	info->m_bodiesCollisionOn = GetBodiesCollisionState();

	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix1, sizeof (dMatrix));
}


void dCustom6DOF::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// add the linear limits
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	dVector dp (p0 - p1);

	for (int i = 0; i < 3; i ++) {
		if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0[i][0]);
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		} else {
			// it is a limited linear dof, check if it pass the limits
			dFloat dist = dp.DotProduct3(matrix1[i]);
			if (dist > m_maxLinearLimits[i]) {
				dVector q1 (p1 + matrix1[i].Scale (m_maxLinearLimits[i]));

				// clamp the error, so the not too much energy is added when constraint violation occurs
				dFloat maxDist = (p0 - q1).DotProduct3(matrix1[i]);
				if (maxDist > D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION) {
					q1 = p0 - matrix1[i].Scale(D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION);
				}

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &q1[0], &matrix0[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);
				// allow the object to return but not to kick going forward
				NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

			} else if (dist < m_minLinearLimits[i]) {
				dVector q1 (p1 + matrix1[i].Scale (m_minLinearLimits[i]));

				// clamp the error, so the not too much energy is added when constraint violation occurs
				dFloat maxDist = (p0 - q1).DotProduct3(matrix1[i]);
				if (maxDist < -D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION) {
					q1 = p0 - matrix1[i].Scale(-D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION);
				}

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &q1[0], &matrix0[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);
				// allow the object to return but not to kick going forward
				NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
			}
		}
	}

	dVector euler0(0.0f);
	dVector euler1(0.0f);
	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	localMatrix.GetEulerAngles(euler0, euler1);

	AngularIntegration pitchStep0 (AngularIntegration (euler0.m_x) - m_pitch);
	AngularIntegration pitchStep1 (AngularIntegration (euler1.m_x) - m_pitch);
	if (dAbs (pitchStep0.GetAngle()) > dAbs (pitchStep1.GetAngle())) {
		euler0 = euler1;
	}

	dVector euler (m_pitch.Update (euler0.m_x), m_yaw.Update (euler0.m_y), m_roll.Update (euler0.m_z), 0.0f);

//dTrace (("(%f %f %f) (%f %f %f)\n", m_pitch.m_angle * 180.0f / 3.141592f, m_yaw.m_angle * 180.0f / 3.141592f, m_roll.m_angle * 180.0f / 3.141592f,  euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f));

	bool limitViolation = false;
	for (int i = 0; i < 3; i ++) {
		if (euler[i] < m_minAngularLimits[i]) {
			limitViolation = true;
			euler[i] = m_minAngularLimits[i];
		} else if (euler[i] > m_maxAngularLimits[i]) {
			limitViolation = true;
			euler[i] = m_maxAngularLimits[i];
		}
	}

	if (limitViolation) {
		//dMatrix pyr (dPitchMatrix(m_pitch.m_angle) * dYawMatrix(m_yaw.m_angle) * dRollMatrix(m_roll.m_angle));
		dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
		dMatrix baseMatrix (p0y0r0 * matrix1);
        dMatrix rotation (matrix0.Inverse() * baseMatrix);

        dQuaternion quat (rotation);
        if (quat.m_q0 > dFloat (0.99995f)) {
			//dVector p0 (matrix0[3] + baseMatrix[1].Scale (MIN_JOINT_PIN_LENGTH));
			//dVector p1 (matrix0[3] + baseMatrix[1].Scale (MIN_JOINT_PIN_LENGTH));
			//NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &baseMatrix[2][0]);
			//NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

			//dVector q0 (matrix0[3] + baseMatrix[0].Scale (MIN_JOINT_PIN_LENGTH));
			//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &baseMatrix[1][0]);
			//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &baseMatrix[2][0]);

        } else {
            dMatrix basis (dGrammSchmidt (dVector (quat.m_q1, quat.m_q2, quat.m_q3, 0.0f)));

			dVector q0 (matrix0[3] + basis[1].Scale (MIN_JOINT_PIN_LENGTH));
			dVector q1 (matrix0[3] + rotation.RotateVector(basis[1].Scale (MIN_JOINT_PIN_LENGTH)));
			NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &basis[2][0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

			//dVector q0 (matrix0[3] + basis[0].Scale (MIN_JOINT_PIN_LENGTH));
			//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[1][0]);
			//NewtonUserJointAddLinearRow (m_joint, &q0[0], &q0[0], &basis[2][0]);
        }
	}
}

