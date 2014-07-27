/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// Custom6DOF.cpp: implementation of the Custom6DOF class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "Custom6DOF.h"

//dInitRtti(Custom6DOF);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


Custom6DOF::Custom6DOF (const dMatrix& pinsAndPivotChildFrame, const dMatrix& pinsAndPivotParentFrame___, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f, 0.0f, 0.0f, 0.0f)
	,m_maxLinearLimits(0.0f, 0.0f, 0.0f, 0.0f)
	,m_minAngularLimits(0.0f, 0.0f, 0.0f, 0.0f)
	,m_maxAngularLimits(0.0f, 0.0f, 0.0f, 0.0f)
	,m_maxMaxLinearErrorRamp(0.2f, 0.2f, 0.2f, 0.0f)
	,m_maxMaxAngularErrorRamp (1.0f * 3.141592f / 180.0f, 1.0f * 3.141592f / 180.0f, 1.0f * 3.141592f / 180.0f, 0.0f)
	,m_pitch()
	,m_yaw()
	,m_roll()
{
	CalculateLocalMatrix (pinsAndPivotChildFrame, m_localMatrix0, m_localMatrix1);

}

Custom6DOF::~Custom6DOF()
{
}


void Custom6DOF::SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minLinearLimits[i] =  (dAbs (minLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : minLinearLimits[i];
		m_maxLinearLimits[i] =  (dAbs (maxLinearLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : maxLinearLimits[i];
	}

}

void Custom6DOF::SetAngularLimits (const dVector& minAngularLimits, const dVector& maxAngularLimits)
{
	for (int i = 0; i < 3; i ++) {
		m_minAngularLimits[i] =  (dAbs (minAngularLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : minAngularLimits[i];
		m_maxAngularLimits[i] =  (dAbs (maxAngularLimits[i]) < dFloat (1.0e-5f)) ? 0.0f : maxAngularLimits[i];
	}
}

void Custom6DOF::GetLinearLimits (dVector& minLinearLimits, dVector& maxLinearLimits)
{
	minLinearLimits = m_minLinearLimits;
	maxLinearLimits = m_maxLinearLimits;
}

void Custom6DOF::GetAngularLimits (dVector& minAngularLimits, dVector& maxAngularLimits)
{
	minAngularLimits = m_minAngularLimits;
	maxAngularLimits = m_maxAngularLimits;
}


void Custom6DOF::GetInfo (NewtonJointRecord* const info) const
{
	dFloat dist;
	dMatrix matrix0;
	dMatrix matrix1;

	strcpy (info->m_descriptionType, "generic6dof");

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

	dVector p0 (matrix0.m_posit);
	dVector p1 (matrix1.m_posit);
	dVector dp (p0 - p1);
	for (int i = 0; i < 3; i ++) {
		if (!((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f))) {
			p1 += matrix1[i].Scale (dp % matrix1[i]);
		}
	}

	for (int i = 0; i < 3; i ++) {
		if ((m_minLinearLimits[i] == 0.0f) && (m_maxLinearLimits[i] == 0.0f)) {
			info->m_minLinearDof[i] = 0.0f;
			info->m_maxLinearDof[i] = 0.0f;
		} else {
			dist = dp % matrix1[i];
			info->m_maxLinearDof[i] = m_maxLinearLimits[i] - dist;
			info->m_minLinearDof[i] = m_minLinearLimits[i] - dist;
		}
	}

	dVector eulerAngles (m_pitch.m_angle, m_yaw.m_angle, m_roll.m_angle, 0.0f);
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


void Custom6DOF::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

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
			dFloat dist = dp % matrix1[i];
			if (dist > m_maxLinearLimits[i]) {
				dVector q1 (p1 + matrix1[i].Scale (m_maxLinearLimits[i]));

				// clamp the error, so the not too much energy is added when constraint violation occurs
				dFloat maxDist = (p0 - q1) % matrix1[i];
				if (maxDist > m_maxMaxLinearErrorRamp[i]) {
					q1 = p0 - matrix1[i].Scale(m_maxMaxLinearErrorRamp[i]);
				}

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &q1[0], &matrix0[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);
				// allow the object to return but not to kick going forward
				NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

			} else if (dist < m_minLinearLimits[i]) {
				dVector q1 (p1 + matrix1[i].Scale (m_minLinearLimits[i]));

				// clamp the error, so the not too much energy is added when constraint violation occurs
				dFloat maxDist = (p0 - q1) % matrix1[i];
				if (maxDist < -m_maxMaxLinearErrorRamp[i]) {
					q1 = p0 - matrix1[i].Scale(-m_maxMaxLinearErrorRamp[i]);
				}

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &q1[0], &matrix0[i][0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);
				// allow the object to return but not to kick going forward
				NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
			}
		}
	}

	dVector euler0;
	dVector euler1;
	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	localMatrix.GetEulerAngles(euler0, euler1);

	AngularIntegration pitchStep0 (AngularIntegration (euler0.m_x) - m_pitch);
	AngularIntegration pitchStep1 (AngularIntegration (euler1.m_x) - m_pitch);
	if (dAbs (pitchStep0.m_angle) > dAbs (pitchStep1.m_angle)) {
		euler0 = euler1;
	}

	m_yaw.CalculateJointAngle (euler0.m_y);
	m_roll.CalculateJointAngle (euler0.m_z);
	m_pitch.CalculateJointAngle (euler0.m_x);
	dVector eulerAngles (-m_pitch.m_angle, -m_yaw.m_angle, -m_roll.m_angle, 0.0f);
	for (int i = 0; i < 3; i ++) {
		if ((m_minAngularLimits[i] == 0.0f) && (m_maxAngularLimits[i] == 0.0f)) {
			NewtonUserJointAddAngularRow (m_joint, eulerAngles[i], &matrix0[i][0]);
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		} else {
			// it is a limited linear dof, check if it pass the limits
			if (eulerAngles[i] > m_maxAngularLimits[i]) {
				dFloat dist = eulerAngles[i] - m_maxAngularLimits[i];
				// clamp the error, so the not too much energy is added when constraint violation occurs
				if (dist > m_maxMaxAngularErrorRamp[i]) {
					dist = m_maxMaxAngularErrorRamp[i];
				}

				// tell joint error will minimize the exceeded angle error
				NewtonUserJointAddAngularRow (m_joint, dist, &matrix0[i][0]);

				// need high stiffness here
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);

				// allow the joint to move back freely
				NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);

			} else if (eulerAngles[i] < m_minAngularLimits[i]) {
				dFloat dist = eulerAngles[i] - m_minAngularLimits[i];
				// clamp the error, so the not too much energy is added when constraint violation occurs
				if (dist < -m_maxMaxAngularErrorRamp[i]) {
					dist = -m_maxMaxAngularErrorRamp[i];
				}

				// tell joint error will minimize the exceeded angle error
				NewtonUserJointAddAngularRow (m_joint, dist, &matrix0[i][0]);

				// need high stiffness here
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);

				// allow the joint to move back freely 
				NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);
			}
		}
	}
}









