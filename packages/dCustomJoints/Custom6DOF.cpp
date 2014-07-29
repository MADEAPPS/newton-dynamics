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

#define MIN_JOINT_PIN_LENGTH					50.0f
#define D_6DOF_ANGULAR_MAX_LINEAR_CORRECTION	0.5f
#define D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION	10.0f

Custom6DOF::Custom6DOF (const dMatrix& pinsAndPivotChildFrame, const dMatrix& pinsAndPivotParentFrame___, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent)
	,m_minLinearLimits(0.0f, 0.0f, 0.0f, 0.0f)
	,m_maxLinearLimits(0.0f, 0.0f, 0.0f, 0.0f)
	,m_minAngularLimits(0.0f, 0.0f, 0.0f, 0.0f)
	,m_maxAngularLimits(0.0f, 0.0f, 0.0f, 0.0f)
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


/*
static dMatrix CalculateUniversal_Angles (dMatrix matrix0, dMatrix matrix1, int x, int y, int z)
{
	dFloat sinAngle;
	dFloat cosAngle;

matrix1 = GetIdentityMatrix();
matrix0 = dPitchMatrix(1.0f * 3.141792 / 180.0f) * dYawMatrix(45.0f * 3.141792 / 180.0f) * dRollMatrix(3.0f * 3.141792 / 180.0f);

	// this assumes calculation assumes that the angle relative to the z axis is alway small
	// because it is the unconstitutionally strong angle
	dMatrix basisAndEulerAngles;
	basisAndEulerAngles[x] = matrix0[x];
	basisAndEulerAngles[y] = matrix1[y];
	basisAndEulerAngles[z] = basisAndEulerAngles[x] * basisAndEulerAngles[y];
	basisAndEulerAngles[z] = basisAndEulerAngles[z].Scale (1.0f / dSqrt (basisAndEulerAngles[z] % basisAndEulerAngles[z]));

	cosAngle = matrix0[y] % matrix1[y];
	sinAngle = (matrix0[y] * matrix1[y]) % basisAndEulerAngles[x];
	basisAndEulerAngles[3][x] = dAtan2 (sinAngle, cosAngle);

	cosAngle = matrix0[x] % matrix1[x];
	sinAngle = (matrix0[x] * matrix1[x]) % matrix1[y];
	basisAndEulerAngles[3][y] = dAtan2 (sinAngle, cosAngle);

	dVector dir3 (basisAndEulerAngles[z] * basisAndEulerAngles[x]);
	dir3 = dir3.Scale (1.0f / dSqrt (dir3 % dir3));
	cosAngle = dir3 % basisAndEulerAngles[y];
	sinAngle = (dir3 * basisAndEulerAngles[y]) % basisAndEulerAngles[z];
	basisAndEulerAngles[3][z] = dAtan2 (sinAngle, cosAngle);
	//	dAssert (dAbs (angle_z) < 0.1f);
	basisAndEulerAngles[3][3] = 1.0f;


dVector euler0;
dVector euler1;
dMatrix xxx (matrix0 * matrix1.Inverse());
xxx.GetEulerAngles(euler0, euler1);
dVector xxx1 = basisAndEulerAngles.m_posit.Scale (-1.0f);


dTrace (("(%f %f %f) (%f %f %f) (%f %f %f)\n", xxx1.m_x * 180.0f / 3.141592f, xxx1.m_y * 180.0f / 3.141592f, xxx1.m_z * 180.0f / 3.141592f, 
		euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f, 
		euler1.m_x * 180.0f / 3.141592f, euler1.m_y * 180.0f / 3.141592f, euler1.m_z * 180.0f / 3.141592f));

	return basisAndEulerAngles;
}
*/

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
				dFloat maxDist = (p0 - q1) % matrix1[i];
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

	dVector euler0;
	dVector euler1;
	dMatrix localMatrix (matrix0 * matrix1.Inverse());
	localMatrix.GetEulerAngles(euler0, euler1);

//CalculateUniversal_Angles (matrix0, matrix1, 0, 1, 2);

	AngularIntegration pitchStep0 (AngularIntegration (euler0.m_x) - m_pitch);
	AngularIntegration pitchStep1 (AngularIntegration (euler1.m_x) - m_pitch);
	if (dAbs (pitchStep0.m_angle) > dAbs (pitchStep1.m_angle)) {
		euler0 = euler1;
	}

	dVector euler (m_pitch.Update (euler0.m_x), m_yaw.Update (euler0.m_y), m_roll.Update (euler0.m_z), 0.0f);

//dAssert (dAbs (m_pitch.m_angle * 180.0f / 3.141592f) < 70.0f);
//dAssert (dAbs (m_yaw.m_angle * 180.0f / 3.141592f) < 70.0f);
//dAssert (dAbs (m_roll.m_angle * 180.0f / 3.141592f) < 70.0f);
dTrace (("(%f %f %f) (%f %f %f)\n", m_pitch.m_angle * 180.0f / 3.141592f, m_yaw.m_angle * 180.0f / 3.141592f, m_roll.m_angle * 180.0f / 3.141592f,  euler0.m_x * 180.0f / 3.141592f, euler0.m_y * 180.0f / 3.141592f, euler0.m_z * 180.0f / 3.141592f));

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
		dMatrix pyr (dPitchMatrix(m_pitch.m_angle) * dYawMatrix(m_yaw.m_angle) * dRollMatrix(m_roll.m_angle));
		dMatrix p0y0r0 (dPitchMatrix(euler[0]) * dYawMatrix(euler[1]) * dRollMatrix(euler[2]));
		dMatrix rotation (pyr * p0y0r0.Inverse());
		dMatrix baseMatrix (p0y0r0 * matrix1);
dMatrix xxxx (rotation * baseMatrix);
dMatrix xxxx1 (rotation * baseMatrix);
	}

/*
	dVector eulerAngles (-m_pitch.m_angle, -m_yaw.m_angle, -m_roll.m_angle, 0.0f);
	for (int i = 0; i < 1; i ++) {
		if ((m_minAngularLimits[i] == 0.0f) && (m_maxAngularLimits[i] == 0.0f)) {
			NewtonUserJointAddAngularRow (m_joint, eulerAngles[i], &matrix0[i][0]);
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		} else {
			// it is a limited linear dof, check if it pass the limits
			if (eulerAngles[i] > m_maxAngularLimits[i]) {
				dFloat dist = eulerAngles[i] - m_maxAngularLimits[i];
				// clamp the error, so the not too much energy is added when constraint violation occurs
				if (dist > D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION) {
					dist = D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION;
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
				if (dist < -D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION) {
					dist = -D_6DOF_ANGULAR_MAX_ANGULAR_CORRECTION;
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
*/
}









