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


// dCustomPlane.cpp: implementation of the dCustomPlane class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomPlane.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomPlane);


dCustomPlane::dCustomPlane (const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(5, child, parent)
	,m_enableControlRotation(false)
{
	dMatrix pinAndPivotFrame(dGrammSchmidt(normal));
	pinAndPivotFrame.m_posit = pivot;
	pinAndPivotFrame.m_posit.m_w = 1.0f;
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

void dCustomPlane::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	int state;
	callback(userData, &state, sizeof(state));
	m_enableControlRotation ? true : false;
}

void dCustomPlane::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);
	int state = m_enableControlRotation ? 1 : 0;
	callback(userData, &state, sizeof(state));
}


dCustomPlane::~dCustomPlane()
{
}


void dCustomPlane::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	const dVector& dir = matrix1[0];
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &dir[0]);
	
	const dFloat invTimeStep = 1.0f / timestep;
	const dFloat dist = 0.25f * dir.DotProduct3(p1 - p0);
	const dFloat accel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dist * invTimeStep * invTimeStep;
	NewtonUserJointSetRowAcceleration(m_joint, accel);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	// construct an orthogonal coordinate system with these two vectors
	if (m_enableControlRotation) {
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);
		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	}
}

