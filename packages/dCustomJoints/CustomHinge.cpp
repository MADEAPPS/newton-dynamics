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



// CustomHinge.cpp: implementation of the CustomHinge class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomHinge.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH	50.0f

//dInitRtti(CustomHinge);

CustomHinge::CustomHinge (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent)
	,m_curJointAngle()
{
	m_limitsOn = false;
	m_friction = 0.0f;
	m_jointOmega = 0.0f;
	m_minAngle = -45.0f * 3.141592f / 180.0f;
	m_maxAngle =  45.0f * 3.141592f / 180.0f;

	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}


CustomHinge::CustomHinge (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:CustomJoint(6, child, parent), m_curJointAngle()
{
	m_limitsOn = false;
	m_friction = 0.0f;
	m_jointOmega = 0.0f;
	m_minAngle = -45.0f * 3.141592f / 180.0f;
	m_maxAngle =  45.0f * 3.141592f / 180.0f;

	dMatrix	dummy;
	CalculateLocalMatrix (pinAndPivotFrameChild, m_localMatrix0, dummy);
	CalculateLocalMatrix (pinAndPivotFrameParent, dummy, m_localMatrix1);
}

CustomHinge::~CustomHinge()
{
}


void CustomHinge::EnableLimits(bool state)
{
	m_limitsOn = state;
}

void CustomHinge::SetLimis(dFloat minAngle, dFloat maxAngle)
{
	m_minAngle = minAngle;
	m_maxAngle = maxAngle;
}


dFloat CustomHinge::GetJointAngle () const
{
	return m_curJointAngle.m_angle;
}

dVector CustomHinge::GetPinAxis () const
{
	dMatrix matrix;
	NewtonBodyGetMatrix (m_body0, &matrix[0][0]);
	return matrix.RotateVector (m_localMatrix0.m_front);
}

dFloat CustomHinge::GetJointOmega () const
{
	return m_jointOmega;
}

void CustomHinge::SetFriction (dFloat frictionTorque)
{
	m_friction = frictionTorque;
}

dFloat CustomHinge::GetFriction () const
{
	return m_friction;
}


void CustomHinge::CalculatePitchAngle (const dMatrix& matrix0, const dMatrix& matrix1, dFloat& sinAngle, dFloat& cosAngle) const
{
//	sinAngle = (matrix1.m_up * matrix0.m_up) % matrix1.m_front;
//	cosAngle = matrix1.m_up % matrix0.m_up;
	sinAngle = (matrix0.m_up * matrix1.m_up) % matrix1.m_front;
	cosAngle = matrix0.m_up % matrix1.m_up;
}


void CustomHinge::ProjectError () const
{
	dMatrix body0Matrix;
	dMatrix body1Matrix;

	NewtonBodyGetMatrix(m_body0, &body0Matrix[0][0]);
	NewtonBodyGetMatrix(m_body1, &body1Matrix[0][0]);

	dMatrix matrix0 (m_localMatrix0 * body0Matrix);
	dMatrix matrix1 (m_localMatrix1 * body1Matrix);

	dFloat sinAngle;
	dFloat cosAngle;

	CalculatePitchAngle (matrix0, matrix1, sinAngle, cosAngle);
	dFloat angleMag = 1.0f / dSqrt (sinAngle * sinAngle + cosAngle * cosAngle);
	sinAngle *= angleMag;
	cosAngle *= angleMag;

	const dMatrix& identity = GetIdentityMatrix();
	dMatrix angleMatrix (identity);
	angleMatrix[1][1] = cosAngle;
	angleMatrix[1][2] = -sinAngle;
	angleMatrix[2][2] = cosAngle;
	angleMatrix[2][1] = sinAngle;

	dMatrix expectedMatrix0 (angleMatrix * matrix1);
	dMatrix errorMatrix (matrix0 * expectedMatrix0.Inverse());
	if (!errorMatrix.TestIdentity()) {
		body0Matrix = m_localMatrix0.Inverse() * expectedMatrix0; 
		NewtonBodySetMatrix(m_body0, &body0Matrix[0][0]);
	}
	dVector omega0;
	dVector omega1;
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);
	dVector deltaOmega (omega0 - omega1);
	dVector expectedOmega0 (omega1 + matrix1.m_front.Scale (deltaOmega % matrix1.m_front));
	dVector omegaError (omega0 - expectedOmega0);
	if ((omegaError % omegaError) > 1.0e-4f) {
		NewtonBodySetOmega(m_body0, &expectedOmega0[0]);
	}

	dVector com0;
	dVector com1;
	dVector veloc0;
	dVector veloc1;
	NewtonBodyGetVelocity(m_body0, &veloc0[0]);
	NewtonBodyGetVelocity(m_body1, &veloc1[0]);
	NewtonBodyGetCentreOfMass(m_body0, &com0[0]);
	NewtonBodyGetCentreOfMass(m_body1, &com1[0]);
	com0 = body0Matrix.TransformVector (com0);
	com1 = body1Matrix.TransformVector (com1);
	dVector relOmega (expectedOmega0 - omega1);
	dVector pivot0 (body0Matrix.TransformVector(m_localMatrix0.m_posit) - com0);
	dVector expectedVeloc0 (veloc1 + omega1 * (com0 - com1) - relOmega * pivot0);
	dVector velocError (expectedVeloc0 - veloc0);
	if ((velocError % velocError) > 1.0e-4f) {
		NewtonBodySetVelocity(m_body0, &expectedVeloc0[0]);
	}
}

void CustomHinge::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_right[0]);
	
	// get a point along the pin axis at some reasonable large distance from the pivot
	dVector q0 (matrix0.m_posit + matrix0.m_front.Scale(MIN_JOINT_PIN_LENGTH));
	dVector q1 (matrix1.m_posit + matrix1.m_front.Scale(MIN_JOINT_PIN_LENGTH));

	// two constraints row perpendicular to the pin vector
 	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix1.m_right[0]);

	// the joint angle can be determine by getting the angle between any two non parallel vectors
	dFloat sinAngle;
	dFloat cosAngle;
	CalculatePitchAngle (matrix0, matrix1, sinAngle, cosAngle);
	m_curJointAngle.CalculateJointAngle (cosAngle, sinAngle);

	// save the current joint Omega
	dVector omega0(0.0f, 0.0f, 0.0f, 0.0f);
	dVector omega1(0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	if (m_body1) {
		NewtonBodyGetOmega(m_body1, &omega1[0]);
	}
	m_jointOmega = (omega0 - omega1) % matrix1.m_front;

    // four possibilities
	if (m_friction != 0.0f) {
        if (m_limitsOn) {
            // friction and limits at the same time
            dAssert (0);
        } else {
            // friction but not limits
		    dFloat alpha = m_jointOmega / timestep;
		    NewtonUserJointAddAngularRow (m_joint, 0, &matrix1.m_front[0]);		
		    NewtonUserJointSetRowAcceleration (m_joint, -alpha);
		    NewtonUserJointSetRowMinimumFriction (m_joint, -m_friction);
		    NewtonUserJointSetRowMaximumFriction (m_joint,  m_friction);
		    NewtonUserJointSetRowStiffness (m_joint, 1.0f);
        }
	} else if (m_limitsOn) {
        // only limit are on 
        // the joint angle can be determine by getting the angle between any two non parallel vectors
        if (m_curJointAngle.m_angle < m_minAngle) {
            dFloat relAngle = m_curJointAngle.m_angle - m_minAngle;
            // the angle was clipped save the new clip limit
            m_curJointAngle.m_angle = m_minAngle;

            // tell joint error will minimize the exceeded angle error
            NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_front[0]);

            // need high stiffness here
            NewtonUserJointSetRowStiffness (m_joint, 1.0f);

            // allow the joint to move back freely 
            NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);
        } else if (m_curJointAngle.m_angle  > m_maxAngle) {
            dFloat relAngle = m_curJointAngle.m_angle - m_maxAngle;

            // the angle was clipped save the new clip limit
            m_curJointAngle.m_angle = m_maxAngle;

            // tell joint error will minimize the exceeded angle error
            NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_front[0]);

            // need high stiffness here
            NewtonUserJointSetRowStiffness (m_joint, 1.0f);

            // allow the joint to move back freely
            NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
        }
    }
}


void CustomHinge::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, "hinge");

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	info->m_minLinearDof[0] = 0.0f;
	info->m_maxLinearDof[0] = 0.0f;

	info->m_minLinearDof[1] = 0.0f;
	info->m_maxLinearDof[1] = 0.0f;;

	info->m_minLinearDof[2] = 0.0f;
	info->m_maxLinearDof[2] = 0.0f;

	// the joint angle can be determine by getting the angle between any two non parallel vectors
	if (m_limitsOn) {
		dFloat angle;
		dFloat sinAngle;
		dFloat cosAngle;
		dMatrix matrix0;
		dMatrix matrix1;
		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (matrix0, matrix1);
		sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
		cosAngle = matrix0.m_up % matrix1.m_up;
		angle = dAtan2 (sinAngle, cosAngle);
		info->m_minAngularDof[0] = (m_minAngle - angle) * 180.0f / 3.141592f ;
		info->m_maxAngularDof[0] = (m_maxAngle - angle) * 180.0f / 3.141592f ;
	} else {
		info->m_minAngularDof[0] = -FLT_MAX ;
		info->m_maxAngularDof[0] = FLT_MAX ;
	}

	info->m_minAngularDof[1] = 0.0f;
	info->m_maxAngularDof[1] = 0.0f;

	info->m_minAngularDof[2] = 0.0f;
	info->m_maxAngularDof[2] = 0.0f;

	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix1, sizeof (dMatrix));
}
