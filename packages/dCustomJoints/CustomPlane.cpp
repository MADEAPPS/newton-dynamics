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


// CustomPlane3DOF.cpp: implementation of the CustomPlane3DOF class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomPlane.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(CustomPlane3DOF);

CustomPlane3DOF::CustomPlane3DOF (const dVector& pivot, const dVector& normal, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(4, child, parent)
{
	dAssert(0);
	// calculate the two local matrix of the pivot point
//	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

CustomPlane3DOF::CustomPlane3DOF(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:CustomJoint(child, parent, callback, userData)
{
}

void CustomPlane3DOF::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	CustomJoint::Serialize(callback, userData);
}


CustomPlane3DOF::~CustomPlane3DOF()
{
}


void CustomPlane3DOF::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, GetTypeName());
	dAssert(0);
}




void CustomPlane3DOF::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dAssert(0);
/*
	dMatrix matrix0;
	dMatrix matrix1;
	dFloat sinAngle;
	dFloat cosAngle;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	dVector p0(matrix0.m_posit);
	dVector p1(matrix1.m_posit + matrix1.m_front.Scale((p0 - matrix1.m_posit).DotProduct3(matrix1.m_front)));
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

	// construct an orthogonal coordinate system with these two vectors
	dMatrix matrix1_1;
	matrix1_1.m_up = matrix1.m_up;
	matrix1_1.m_right = matrix0.m_front.CrossProduct(matrix1.m_up);
	matrix1_1.m_right = matrix1_1.m_right.Scale(1.0f / dSqrt(matrix1_1.m_right.DotProduct3(matrix1_1.m_right)));
	matrix1_1.m_front = matrix1_1.m_up.CrossProduct(matrix1_1.m_right);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_up, matrix1_1.m_up, matrix1_1.m_front), &matrix1_1.m_front[0]);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_up, matrix1_1.m_up, matrix1_1.m_right), &matrix1_1.m_right[0]);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	CalculateAngle(matrix1_1.m_front, matrix1.m_front, matrix1.m_up, sinAngle, cosAngle);
	m_curJointAngle.Update(cosAngle, sinAngle);

	dVector veloc0(0.0f);
	dVector veloc1(0.0f);
	dAssert(m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}
	m_posit = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix1.m_front);
	m_speed = (veloc0 - veloc1).DotProduct3(matrix1.m_front);
	
	// if limit are enable ...
	if (m_limitsAngularOn) {
		dFloat angle1 = m_curJointAngle.GetAngle();
		if (angle1 < m_minAngularDist) {
			dFloat relAngle = angle1 - m_minAngularDist;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness(m_joint, 1.0f);

			// allow the joint to move back freely 
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

		} else if (angle1 > m_maxAngularDist) {
			dFloat relAngle = angle1 - m_maxAngularDist;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness(m_joint, 1.0f);

			// allow the joint to move back freely
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}
	}

	if (m_setAsSpringDamper && m_limitsLinearOn) {
		if (m_posit < m_minLinearDist) {
			//dVector p(matrix1.m_posit + matrix1.m_front.Scale(m_minLinearDist));
			//NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_front[0]);
			//NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_minLinearDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
			dFloat accel = NewtonUserJointGetRowAcceleration(m_joint) + NewtonCalculateSpringDamperAcceleration(timestep, m_spring, m_posit, m_damper, m_speed);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);

		} else if (m_posit > m_maxLinearDist) {
			//dVector p(matrix1.m_posit + matrix1.m_front.Scale(m_maxLinearDist));
			//NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_front[0]);
			//NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_maxLinearDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
			dFloat accel = NewtonUserJointGetRowAcceleration(m_joint) + NewtonCalculateSpringDamperAcceleration(timestep, m_spring, m_posit, m_damper, m_speed);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		} else {
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
		}
	} else if (m_limitsLinearOn) {
		if (m_posit < m_minLinearDist) {
			dVector p(matrix1.m_posit + matrix1.m_front.Scale(m_minLinearDist));
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		} else if (m_posit > m_maxLinearDist) {
			dVector p(matrix1.m_posit + matrix1.m_front.Scale(m_maxLinearDist));
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &p[0], &matrix1.m_front[0]);
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
		}
	} else if (m_setAsSpringDamper) {
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
	}
*/
}

