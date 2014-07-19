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


// CustomPathFollow.cpp: implementation of the CustomPathFollow class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomPathFollow.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH	50.0f

//dInitRtti(CustomPathFollow);

CustomPathFollow::CustomPathFollow (const dMatrix& pinAndPivotFrame, NewtonBody* const child)
	:CustomJoint(6, child, NULL)
	,m_pathTangent (1.0f, 0.0f, 0.0f, 0.0f)
	,m_pointOnPath (0.0f, 10.0f, 0.0f, 0.0)
{
	// calculate the two local matrix of the pivot point
	dMatrix tmp;
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, tmp);
}

CustomPathFollow::~CustomPathFollow()
{
}

void CustomPathFollow::GetInfo (NewtonJointRecord* const info) const
{
/*
	strcpy (info->m_descriptionType, "slider");

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	if (m_limitsOn) {
		dFloat dist;
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);
		dist = (matrix0.m_posit - matrix1.m_posit) % matrix0.m_front;

		info->m_minLinearDof[0] = m_minDist - dist;
		info->m_maxLinearDof[0] = m_maxDist - dist;
	} else {
		info->m_minLinearDof[0] = -FLT_MAX ;
		info->m_maxLinearDof[0] =  FLT_MAX ;
	}


	info->m_minLinearDof[1] = 0.0f;
	info->m_maxLinearDof[1] = 0.0f;;

	info->m_minLinearDof[2] = 0.0f;
	info->m_maxLinearDof[2] = 0.0f;

	info->m_minAngularDof[0] = 0.0f;
	info->m_maxAngularDof[0] = 0.0f;

	info->m_minAngularDof[1] = 0.0f;
	info->m_maxAngularDof[1] = 0.0f;

	info->m_minAngularDof[2] = 0.0f;
	info->m_maxAngularDof[2] = 0.0f;

	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix1, sizeof (dMatrix));
*/
}


void CustomPathFollow::SetPathTarget (const dVector& posit, const dVector& tangent)
{
	m_pointOnPath = posit;
	m_pathTangent = tangent.Scale (1.0f / dSqrt (m_pathTangent % m_pathTangent));
}

// calculate the closest point from the spline to point point
dMatrix CustomPathFollow::EvalueCurve (const dVector& posit)
{
	dMatrix matrix;


	// calculate distance for point to list
	matrix.m_posit = m_pointOnPath + m_pathTangent.Scale ((posit - m_pointOnPath) % m_pathTangent);
	matrix.m_posit.m_w = 1.0f;


	//the tangent of the path is the line slope, passes in the first matrix dir
	matrix.m_front = m_pathTangent;

	// the normal will be such that it is horizontal to the the floor and perpendicular to the path 
	dVector normal (dVector (0.0f, 1.0f, 0.0f, 0.0f) * matrix.m_front);
	matrix.m_up = normal.Scale (1.0f / dSqrt (normal % matrix.m_front));

	// the binormal is just the cross product;
	matrix.m_right = matrix.m_front * matrix.m_up;

	return matrix;
}

void CustomPathFollow::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
		
	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);
	matrix0 = m_localMatrix0 * matrix0;

	dMatrix matrix1 (EvalueCurve (matrix0.m_posit));

	// Restrict the movement on the pivot point along all tree the normal and binormal of the path
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_right[0]);
	
	// get a point along the ping axis at some reasonable large distance from the pivot
	dVector q0 (p0 + matrix0.m_front.Scale(MIN_JOINT_PIN_LENGTH));
	dVector q1 (p1 + matrix1.m_front.Scale(MIN_JOINT_PIN_LENGTH));

	// two more constraints rows to inforce the normal and binormal 
 	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_right[0]);


//	Julio: at this point the path follow will maintaing the pivot fixed to the path and will allow it to slide freelly
//	it will also spin around the path tangent, if more control is needed then more constraion rows need to be submited 
//	for example friction along the oath tangent be added to make more realistic behavior
//  or anothet point contration along the binormal to make no spin around the path tangent

/*
	// example of ading friction
	dVector veloc0; 
	dVector omega0; 

	NewtonBodyGetVelocity(m_body0, &veloc0[0]);
	NewtonBodyGetOmega(m_body0, &omega0[0]);

	veloc0 += omega0 * (matrix0.m_posit - p0);

	dFloat relAccel; 
	relAccel = - (veloc0 % matrix0.m_front) / timestep;

	#define MaxFriction 10.0f
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p0[0], &matrix0.m_front[0]);
	NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	NewtonUserJointSetRowMinimumFriction (m_joint, -MaxFriction);
	NewtonUserJointSetRowMaximumFriction(m_joint, MaxFriction);
*/
}



