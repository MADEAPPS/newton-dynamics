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



// dCustomMotor.cpp: implementation of the dCustomMotor class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomMotor.h"

IMPLEMENT_CUSTOM_JOINT(dCustomMotor);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dCustomMotor::dCustomMotor(int dof, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(dof, child, parent)
	,m_motorSpeed(0.0f)
	,m_motorTorque(1.0f)
{
	SetSolverModel(2);
}


dCustomMotor::dCustomMotor(dFloat gearRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* const child, NewtonBody* const parent)
	:dCustomJoint(1, child, parent)
	,m_motorSpeed(0.0f)
	,m_motorTorque(1.0f)
{
	dMatrix dommyMatrix;
	// calculate the local matrix for body body0
	dMatrix pinAndPivot0 (dGrammSchmidt(childPin));

	CalculateLocalMatrix (pinAndPivot0, m_localMatrix0, dommyMatrix);
	m_localMatrix0.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	// calculate the local matrix for body body1  
	dMatrix pinAndPivot1 (dGrammSchmidt(parentPin));
	CalculateLocalMatrix (pinAndPivot1, dommyMatrix, m_localMatrix1);
	m_localMatrix1.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	// set as kinematic loop
	SetSolverModel(2);
}


dCustomMotor::~dCustomMotor()
{
}

void dCustomMotor::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback (userData, &m_motorSpeed, sizeof (dFloat));
	callback (userData, &m_motorTorque, sizeof (dFloat));
}

void dCustomMotor::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_motorSpeed, sizeof(dFloat));
	callback(userData, &m_motorTorque, sizeof(dFloat));
}

void dCustomMotor::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dAssert (0);
/*
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dVector omega1(0.0f);
	dFloat jacobian0[6];
	dFloat jacobian1[6];

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// calculate the angular velocity for both bodies
	dVector dir0 = matrix0.m_front.Scale (m_gearRatio);
	const dVector& dir1 = matrix1.m_front;

	jacobian0[0] = 0.0f;
	jacobian0[1] = 0.0f;
	jacobian0[2] = 0.0f;
	jacobian0[3] = dir0.m_x;
	jacobian0[4] = dir0.m_y;
	jacobian0[5] = dir0.m_z;

	jacobian1[0] = 0.0f;
	jacobian1[1] = 0.0f;
	jacobian1[2] = 0.0f;
	jacobian1[3] = dir1.m_x;
	jacobian1[4] = dir1.m_y;
	jacobian1[5] = dir1.m_z;

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetOmega(m_body1, &omega1[0]);

	dFloat w0 = omega0.DotProduct3(dir0);
	dFloat w1 = omega1.DotProduct3(dir1);
	dFloat relOmega = w0 + w1;
	dFloat invTimestep = (timestep > 0.0f) ? 1.0f / timestep : 1.0f;
	dFloat relAccel = -0.5f * relOmega * invTimestep;
	NewtonUserJointAddGeneralRow(m_joint, jacobian0, jacobian1);
	NewtonUserJointSetRowAcceleration(m_joint, relAccel);
*/
}


