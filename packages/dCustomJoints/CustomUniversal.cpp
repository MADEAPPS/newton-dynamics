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



// CustomUniversal.cpp: implementation of the CustomUniversal class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomUniversal.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH	50.0f

//dInitRtti(CustomUniversal);

CustomUniversal::CustomUniversal(const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent)
	:CustomJoint(6, child, parent)
	,m_curJointAngle_0()
	,m_curJointAngle_1()
{
	dMatrix matrix0;

	m_limit_0_On = true;
	m_angularMotor_0_On = false;
	m_angularDamp_0 = 0.5f;
	m_angularAccel_0 = -4.0f;
	m_jointOmega_0 = 0.0f;
	m_minAngle_0 = -45.0f * 3.141592f / 180.0f;
	m_maxAngle_0 =  45.0f * 3.141592f / 180.0f;

	m_limit_1_On = true;
	m_angularMotor_1_On = false; 
	m_angularDamp_1 = 0.3f;
	m_angularAccel_1 = -4.0f;
	m_jointOmega_1 = 0.0f;
	m_minAngle_1 = -45.0f * 3.141592f / 180.0f;
	m_maxAngle_1 =  45.0f * 3.141592f / 180.0f;


	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);

	dMatrix matrix1 (dGetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);
	}

	// calculate the relative matrix of the pin and pivot on each body
 	m_localMatrix0 = pinAndPivotFrame * matrix0.Inverse();
	m_localMatrix1 = pinAndPivotFrame * matrix1.Inverse();
}

CustomUniversal::~CustomUniversal()
{
}


void CustomUniversal::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, "universal");

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	dMatrix matrix0;
	dMatrix matrix1;
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	info->m_minLinearDof[0] = 0.0f;
	info->m_maxLinearDof[0] = 0.0f;

	info->m_minLinearDof[1] = 0.0f;
	info->m_maxLinearDof[1] = 0.0f;;

	info->m_minLinearDof[2] = 0.0f;
	info->m_maxLinearDof[2] = 0.0f;


	if (m_limit_0_On) {
		dFloat angle;
		dFloat sinAngle;
		dFloat cosAngle;

		sinAngle = (matrix0.m_front * matrix1.m_front) % matrix1.m_up;
		cosAngle = matrix0.m_front % matrix1.m_front;
		angle = dAtan2 (sinAngle, cosAngle);

		info->m_minAngularDof[0] = (m_minAngle_0 - angle) * 180.0f / 3.141592f ;
		info->m_maxAngularDof[0] = (m_maxAngle_0 - angle) * 180.0f / 3.141592f ;
	} else {
		info->m_minAngularDof[0] = -FLT_MAX ;
		info->m_maxAngularDof[0] =  FLT_MAX ;
	}

	//	 info->m_minAngularDof[1] = m_minAngle_1 * 180.0f / 3.141592f;
	//	 info->m_maxAngularDof[1] = m_maxAngle_1 * 180.0f / 3.141592f;

	if (m_limit_1_On) {
		dFloat angle;
		dFloat sinAngle;
		dFloat cosAngle;

		sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
		cosAngle = matrix0.m_up % matrix1.m_up;
		angle = dAtan2 (sinAngle, cosAngle);

		info->m_minAngularDof[1] = (m_minAngle_1 - angle) * 180.0f / 3.141592f ;
		info->m_maxAngularDof[1] = (m_maxAngle_1 - angle) * 180.0f / 3.141592f ;
	} else {
		info->m_minAngularDof[1] = -FLT_MAX ;
		info->m_maxAngularDof[1] =  FLT_MAX ;
	}

	info->m_minAngularDof[2] = 0.0f;
	info->m_maxAngularDof[2] = 0.0f;

	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix1, sizeof (dMatrix));
}

void CustomUniversal::EnableLimit_0(bool state)
{
	m_limit_0_On = state;
}

void CustomUniversal::EnableLimit_1(bool state)
{
	m_limit_1_On = state;
}

void CustomUniversal::EnableMotor_0(bool state)
{
	m_angularMotor_0_On = state;
}

void CustomUniversal::EnableMotor_1(bool state)
{
	m_angularMotor_1_On = state;
}


void CustomUniversal::SetLimis_0(dFloat minAngle, dFloat maxAngle)
{
	dAssert (minAngle < 0.0f);
	dAssert (maxAngle > 0.0f);

	m_minAngle_0 = minAngle;
	m_maxAngle_0 = maxAngle;
}

void CustomUniversal::SetLimis_1(dFloat minAngle, dFloat maxAngle)
{
	dAssert (minAngle < 0.0f);
	dAssert (maxAngle > 0.0f);

	m_minAngle_1 = minAngle;
	m_maxAngle_1 = maxAngle;
}


dFloat CustomUniversal::GetJointAngle_0 () const
{
	return m_curJointAngle_0.GetAngle();
}

dFloat CustomUniversal::GetJointAngle_1 () const
{
	return m_curJointAngle_1.GetAngle();
}

dFloat CustomUniversal::GetJointOmega_0 () const
{
	return m_jointOmega_0;
}

dFloat CustomUniversal::GetJointOmega_1 () const
{
	return m_jointOmega_1;
}

dVector CustomUniversal::GetPinAxis_0 () const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix (matrix0, matrix1);
	return matrix0.m_front;
}

dVector CustomUniversal::GetPinAxis_1 () const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix (matrix0, matrix1);
	return matrix1.m_up;
}

void CustomUniversal::CalculatePitchAngle (const dMatrix& matrix0, const dMatrix& matrix1, dFloat& sinAngle, dFloat& cosAngle) const
{
//	sinAngle = (matrix1.m_up * matrix0.m_up) % matrix0.m_front;
//	cosAngle = matrix1.m_up % matrix0.m_up;
	sinAngle = (matrix0.m_up * matrix1.m_up) % matrix1.m_front;
	cosAngle = matrix0.m_up % matrix1.m_up;
}

void CustomUniversal::CalculateYawAngle (const dMatrix& matrix0, const dMatrix& matrix1, dFloat& sinAngle, dFloat& cosAngle) const
{
//	sinAngle = (matrix1.m_front * matrix0.m_front) % matrix1.m_up;
//	cosAngle = matrix1.m_front % matrix0.m_front;
	sinAngle = (matrix0.m_front * matrix1.m_front) % matrix1.m_up;
	cosAngle = matrix0.m_front % matrix1.m_front;
}

void CustomUniversal::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// get the pin fixed to the first body
	const dVector& dir0 = matrix0.m_front;
	// get the pin fixed to the second body
	const dVector& dir1 = matrix1.m_up;

	// construct an orthogonal coordinate system with these two vectors
	dVector dir2 (dir0 * dir1);
	dir2 = dir2.Scale (1.0f / dSqrt (dir2 % dir2));

	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir0[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir1[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir2[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	dVector dir3 (dir2 * dir0);
	dVector q0 (p0 + dir3.Scale(MIN_JOINT_PIN_LENGTH));
	dVector q1 (p1 + dir1.Scale(MIN_JOINT_PIN_LENGTH));
	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &dir0[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	dFloat sinAngle_0;
	dFloat cosAngle_0;
	CalculatePitchAngle (matrix0, matrix1, sinAngle_0, cosAngle_0);
	m_curJointAngle_0.Update (cosAngle_0, sinAngle_0);

	dFloat sinAngle_1;
	dFloat cosAngle_1;
	CalculateYawAngle (matrix0, matrix1, sinAngle_1, cosAngle_1);
	m_curJointAngle_1.Update (cosAngle_1, sinAngle_1);

	dVector omega0 (0.0f, 0.0f, 0.0f, 0.0f);
	dVector omega1 (0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}

	// calculate the desired acceleration
	dVector relOmega (omega0 - omega1);
	m_jointOmega_0 = relOmega % matrix0.m_front;
	m_jointOmega_1 = relOmega % matrix1.m_up;
	
	// check is the joint limit are enable
	if (m_limit_0_On) {
		if (m_curJointAngle_0.GetAngle() < m_minAngle_0) {
			dFloat relAngle = m_curJointAngle_0.GetAngle() - m_minAngle_0;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

		} else if (m_curJointAngle_0.GetAngle() > m_maxAngle_0) {
			dFloat relAngle = m_curJointAngle_0.GetAngle() - m_maxAngle_0;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely 
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
		}

		// check is the joint limit motor is enable
	} else if (m_angularMotor_0_On) {
		// calculate the desired acceleration
//		dFloat relOmega = (omega0 - omega1) % matrix0.m_front;
		dFloat relAccel = m_angularAccel_0 - m_angularDamp_0 * m_jointOmega_0;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);

		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	}

	// if limit are enable ...
	if (m_limit_1_On) {
		if (m_curJointAngle_1.GetAngle() < m_minAngle_1) {
			dFloat relAngle = m_curJointAngle_1.GetAngle() - m_minAngle_1;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely 
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

		} else if (m_curJointAngle_1.GetAngle() > m_maxAngle_1) {
			dFloat relAngle = m_curJointAngle_1.GetAngle() - m_maxAngle_1;
			
			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
  		}
	} else if (m_angularMotor_1_On) {
		// calculate the desired acceleration
		dFloat relAccel = m_angularAccel_1 - m_angularDamp_1 * m_jointOmega_1;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix1.m_up[0]);
		
		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	}
}


