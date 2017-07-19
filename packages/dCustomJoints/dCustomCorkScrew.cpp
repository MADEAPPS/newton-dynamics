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

//********************************************************************
// dCustomCorkScrew.cpp: implementation of the dCustomCorkScrew class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomCorkScrew.h"



IMPLEMENT_CUSTOM_JOINT(dCustomCorkScrew);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dCustomCorkScrew::dCustomCorkScrew (const dMatrix& pinAndPivotFrame, NewtonBody* child, NewtonBody* parent)
	:dCustomJoint(6, child, parent)
	,m_curJointAngle()
{
	m_limitsLinearOn = false;
	m_limitsAngularOn = false;
	m_minLinearDist = -1.0f;
	m_maxLinearDist = 1.0f;
	m_minAngularDist = -1.0f;
	m_maxAngularDist = 1.0f;

	m_angularmotorOn = false;
	m_angularDamp = 0.1f;
	m_angularAccel = 5.0f;

	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

dCustomCorkScrew::~dCustomCorkScrew()
{
}

dCustomCorkScrew::dCustomCorkScrew (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomJoint(child, parent, callback, userData)
{
	callback (userData, &m_minLinearDist, sizeof (dFloat));
	callback (userData, &m_maxLinearDist, sizeof (dFloat));
	callback (userData, &m_minAngularDist, sizeof (dFloat));
	callback (userData, &m_maxAngularDist, sizeof (dFloat));
	callback (userData, &m_angularDamp, sizeof (dFloat));
	callback (userData, &m_angularAccel, sizeof (dFloat));
	callback (userData, &m_curJointAngle, sizeof (dAngularIntegration));

	int tmp[3];
	callback (userData, tmp, sizeof (tmp));
	m_limitsLinearOn = tmp[0] ? true : false; 
	m_limitsAngularOn = tmp[1] ? true : false; 
	m_angularmotorOn = tmp[2] ? true : false; 
}

void dCustomCorkScrew::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);

	callback (userData, &m_minLinearDist, sizeof (dFloat));
	callback (userData, &m_maxLinearDist, sizeof (dFloat));
	callback (userData, &m_minAngularDist, sizeof (dFloat));
	callback (userData, &m_maxAngularDist, sizeof (dFloat));
	callback (userData, &m_angularDamp, sizeof (dFloat));
	callback (userData, &m_angularAccel, sizeof (dFloat));
	callback (userData, &m_curJointAngle, sizeof (dAngularIntegration));

	int tmp[3];
	tmp[0] = m_limitsLinearOn ; 
	tmp[1] = m_limitsAngularOn; 
	tmp[2] = m_angularmotorOn; 
	callback (userData, tmp, sizeof (tmp));
}

void dCustomCorkScrew::EnableLinearLimits(bool state)
{
	m_limitsLinearOn = state;
}

void dCustomCorkScrew::EnableAngularLimits(bool state)
{
	m_limitsAngularOn = state;
}


void dCustomCorkScrew::SetLinearLimis(dFloat minDist, dFloat maxDist)
{
	//dAssert (minDist < 0.0f);
	//dAssert (maxDist > 0.0f);

	m_minLinearDist = minDist;
	m_maxLinearDist = maxDist;
}

void dCustomCorkScrew::SetAngularLimis(dFloat minDist, dFloat maxDist)
{
	//dAssert (minDist < 0.0f);
	//dAssert (maxDist > 0.0f);

	m_minAngularDist = minDist;
	m_maxAngularDist = maxDist;
}




void dCustomCorkScrew::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	dVector p0(matrix0.m_posit);
	dVector p1(matrix1.m_posit + matrix1.m_front.Scale((p0 - matrix1.m_posit).DotProduct3(matrix1.m_front)));
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);
	
	// two rows to restrict rotation around around the parent coordinate system
	dFloat sinAngle;
	dFloat cosAngle;
//	CalculateYawAngle(matrix0, matrix1, sinAngle, cosAngle);
//	NewtonUserJointAddAngularRow(m_joint, -dAtan2(sinAngle, cosAngle), &matrix1.m_up[0]);
//	CalculateRollAngle(matrix0, matrix1, sinAngle, cosAngle);
//	NewtonUserJointAddAngularRow(m_joint, -dAtan2(sinAngle, cosAngle), &matrix1.m_right[0]);
	
	// two rows to restrict rotation around around the parent coordinate system
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	CalculateAngle(matrix1.m_up, matrix0.m_up, matrix1.m_front, sinAngle, cosAngle);
	dFloat angle = -m_curJointAngle.Update(cosAngle, sinAngle);

	// if limit are enable ...
	if (m_limitsLinearOn) {
		dFloat dist = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix0.m_front);
		if (dist < m_minLinearDist) {
			// get a point along the up vector and set a constraint  
			NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
			
			
		} else if (dist > m_maxLinearDist) {
			// get a point along the up vector and set a constraint  
			NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);
		}
	}


	if (m_limitsAngularOn) {
		// the joint angle can be determine by getting the angle between any two non parallel vectors
		if (angle < m_minAngularDist) {
			dFloat relAngle = angle - m_minAngularDist;
			// the angle was clipped save the new clip limit
			//m_curJointAngle.m_angle = m_minAngularDist;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely 
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);

		} else if (angle > m_maxAngularDist) {
			dFloat relAngle = angle - m_maxAngularDist;

			// the angle was clipped save the new clip limit
			//m_curJointAngle.m_angle = m_maxAngularDist;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

			// allow the joint to move back freely
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
		}
	}

	if (m_angularmotorOn) {
		dVector omega0 (0.0f);
		dVector omega1 (0.0f);

		// get relative angular velocity
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		if (m_body1) {
			NewtonBodyGetOmega(m_body1, &omega1[0]);
		}

		// calculate the desired acceleration
		dFloat relOmega = (omega0 - omega1).DotProduct3(matrix0.m_front);
		dFloat relAccel = m_angularAccel - m_angularDamp * relOmega;

		// if the motor capability is on, then set angular acceleration with zero angular correction 
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);
		
		// override the angular acceleration for this Jacobian to the desired acceleration
		NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	}
 }



