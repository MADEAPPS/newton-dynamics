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



// CustomSlidingContact.cpp: implementation of the CustomSlidingContact class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomSlidingContact.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


CustomSlidingContact::CustomSlidingContact (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent)
	,m_curJointAngle()
	,m_speed(0.0f)
	,m_posit(0.0f)
{
	EnableLinearLimits(false);
	EnableAngularLimits(false);
	SetLinearLimis(-1.0f, 1.0f);
	SetAngularLimis(-30.0f * 3.141592f / 180.0f, 30.0f * 3.141592f / 180.0f);

	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

CustomSlidingContact::~CustomSlidingContact()
{
}

void CustomSlidingContact::EnableLinearLimits(bool state)
{
	m_limitsLinearOn = state;
}

void CustomSlidingContact::EnableAngularLimits(bool state)
{
	m_limitsAngularOn = state;
}

void CustomSlidingContact::SetLinearLimis(dFloat minDist, dFloat maxDist)
{
	m_minLinearDist = minDist;
	m_maxLinearDist = maxDist;
}

void CustomSlidingContact::SetAngularLimis(dFloat minDist, dFloat maxDist)
{
	dAssert (minDist > -80.0f * 3.1416f / 180.0f);
	dAssert (maxDist <  80.0f * 3.1416f / 180.0f);
	m_minAngularDist = minDist;
	m_maxAngularDist = maxDist;
}

void CustomSlidingContact::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, "slidingContact");

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;


	dMatrix matrix0;
	dMatrix matrix1;
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	if (m_limitsLinearOn) {
		dFloat dist;
		dist = (matrix0.m_posit - matrix1.m_posit) % matrix0.m_front;
		info->m_minLinearDof[0] = m_minLinearDist - dist;
		info->m_maxLinearDof[0] = m_maxLinearDist - dist;
	} else {
		info->m_minLinearDof[0] = -FLT_MAX ;
		info->m_maxLinearDof[0] = FLT_MAX ;
	}

	info->m_minLinearDof[1] = 0.0f;
	info->m_maxLinearDof[1] = 0.0f;;

	info->m_minLinearDof[2] = 0.0f;
	info->m_maxLinearDof[2] = 0.0f;

	//	info->m_minAngularDof[0] = -FLT_MAX;
	//	info->m_maxAngularDof[0] =  FLT_MAX;
	if (m_limitsAngularOn) {
		dFloat angle;
		dFloat sinAngle;
		dFloat cosAngle;

		sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
		cosAngle = matrix0.m_up % matrix1.m_up;
		angle = dAtan2 (sinAngle, cosAngle);
		info->m_minAngularDof[0] = (m_minAngularDist - angle) * 180.0f / 3.141592f ;
		info->m_maxAngularDof[0] = (m_maxAngularDist - angle) * 180.0f / 3.141592f ;
	} else {
		info->m_minAngularDof[0] = -FLT_MAX ;
		info->m_maxAngularDof[0] =  FLT_MAX;
	}

	info->m_minAngularDof[1] = 0.0f;
	info->m_maxAngularDof[1] = 0.0f;

	info->m_minAngularDof[2] = 0.0f;
	info->m_maxAngularDof[2] = 0.0f;

	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix1, sizeof (dMatrix));
}


dFloat CustomSlidingContact::GetPosition() const
{
	return m_posit;
}

dFloat CustomSlidingContact::GetSpeed() const
{
	return m_speed;
}

void CustomSlidingContact::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dFloat sinAngle;
	dFloat cosAngle;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	dVector p0(matrix0.m_posit);
	dVector p1(matrix1.m_posit + matrix1.m_front.Scale((p0 - matrix1.m_posit) % matrix1.m_front));
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

	// two rows to restrict rotation around around the parent coordinate system
	CalculateYawAngle(matrix0, matrix1, sinAngle, cosAngle);
	NewtonUserJointAddAngularRow(m_joint, -dAtan2(sinAngle, cosAngle), &matrix1.m_up[0]);

	CalculateRollAngle(matrix0, matrix1, sinAngle, cosAngle);
	NewtonUserJointAddAngularRow(m_joint, -dAtan2(sinAngle, cosAngle), &matrix1.m_right[0]);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	CalculatePitchAngle(matrix0, matrix1, sinAngle, cosAngle);
	m_curJointAngle.Update(cosAngle, sinAngle);

	dVector veloc0(0.0f, 0.0f, 0.0f, 0.0f);
	dVector veloc1(0.0f, 0.0f, 0.0f, 0.0f);
	dAssert(m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}
	m_posit = (matrix0.m_posit - matrix1.m_posit) % matrix1.m_up;
	m_speed = (veloc0 - veloc1) % matrix1.m_up;
	
	// if limit are enable ...
	if (m_limitsLinearOn) {
		if (m_posit < m_minLinearDist) {
			// get a point along the up vector and set a constraint  
			dVector p (matrix1.m_posit + matrix1.m_up.Scale(m_minLinearDist));
			NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_up[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
		} else if (m_posit > m_maxLinearDist) {
			dVector p(matrix1.m_posit + matrix1.m_up.Scale(m_maxLinearDist));
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_up[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		}
	}

	if (m_limitsAngularOn) {
/*
		if (euler0.m_y < m_minAngularDist) {
			NewtonUserJointAddAngularRow(m_joint, -(euler0.m_y - m_minAngularDist), &matrix1.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (euler0.m_y > m_maxAngularDist) {
			NewtonUserJointAddAngularRow(m_joint, - (euler0.m_y - m_maxAngularDist), &matrix1.m_front[0]);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		}
*/		
	}
}

