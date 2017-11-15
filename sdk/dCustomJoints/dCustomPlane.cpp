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


// dCustomPlane3DOF.cpp: implementation of the dCustomPlane3DOF class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomPlane.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomPlane3DOF);
IMPLEMENT_CUSTOM_JOINT(dCustomPlane5DOF);


dCustomPlane5DOF::dCustomPlane5DOF(const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(1, child, parent)
{
	dMatrix pinAndPivotFrame(dGrammSchmidt(normal));
	pinAndPivotFrame.m_posit = pivot;
	pinAndPivotFrame.m_posit.m_w = 1.0f;
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix(pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

void dCustomPlane5DOF::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
}

void dCustomPlane5DOF::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);
}


dCustomPlane5DOF::~dCustomPlane5DOF()
{
}

void dCustomPlane5DOF::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	dVector p0(matrix0.m_posit);
	dVector p1(matrix0.m_posit + matrix1.m_front.Scale((matrix1.m_posit - matrix0.m_posit).DotProduct3(matrix1.m_front)));
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
}


dCustomPlane3DOF::dCustomPlane3DOF (const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(3, child, parent)
{
	dMatrix pinAndPivotFrame(dGrammSchmidt(normal));
	pinAndPivotFrame.m_posit = pivot;
	pinAndPivotFrame.m_posit.m_w = 1.0f;
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

void dCustomPlane3DOF::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
}

void dCustomPlane3DOF::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize(callback, userData);
}


dCustomPlane3DOF::~dCustomPlane3DOF()
{
}


void dCustomPlane3DOF::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	dVector p0(matrix0.m_posit);
	dVector p1(matrix0.m_posit + matrix1.m_front.Scale((matrix1.m_posit - matrix0.m_posit).DotProduct3(matrix1.m_front)));
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	// construct an orthogonal coordinate system with these two vectors
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
}

