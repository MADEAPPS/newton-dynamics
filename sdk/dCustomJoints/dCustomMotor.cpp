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
IMPLEMENT_CUSTOM_JOINT(dCustomMotor2);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dCustomMotor::dCustomMotor(int dof, NewtonBody* const child)
	:dCustomJoint(dof, child, NULL)
	,m_motorOmega(0.0f)
	,m_targetSpeed(0.0f)
	,m_motorTorque(1.0f)
{
	SetSolverModel(2);
}


dCustomMotor::dCustomMotor(const dVector& pin, NewtonBody* const body)
	:dCustomJoint(1, body, NULL)
	,m_motorOmega(0.0f)
	,m_targetSpeed(0.0f)
	,m_motorTorque(1.0f)
{
	dMatrix dommyMatrix;
	// calculate the local matrix for body body0
	dMatrix pinAndPivot0 (dGrammSchmidt(pin));

	CalculateLocalMatrix (pinAndPivot0, m_localMatrix0, dommyMatrix);
	m_localMatrix0.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	// calculate the local matrix for body body1  
	dMatrix pinAndPivot1 (dGrammSchmidt(pin));
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
	callback(userData, &m_motorOmega, sizeof(dFloat));
	callback (userData, &m_targetSpeed, sizeof (dFloat));
	callback (userData, &m_motorTorque, sizeof (dFloat));
}

void dCustomMotor::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);
	callback(userData, &m_motorOmega, sizeof(dFloat));
	callback(userData, &m_targetSpeed, sizeof(dFloat));
	callback(userData, &m_motorTorque, sizeof(dFloat));
}

void dCustomMotor::SetSpeed(dFloat speed)
{
	m_targetSpeed = speed;
}

void dCustomMotor::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dFloat jacobian0[6];
	dFloat jacobian1[6];

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// calculate the angular velocity for both bodies
	const dVector& dir0 = matrix0.m_front;

	jacobian0[0] = 0.0f;
	jacobian0[1] = 0.0f;
	jacobian0[2] = 0.0f;
	jacobian0[3] = dir0.m_x;
	jacobian0[4] = dir0.m_y;
	jacobian0[5] = dir0.m_z;

	jacobian1[0] = 0.0f;
	jacobian1[1] = 0.0f;
	jacobian1[2] = 0.0f;
	jacobian1[3] = -dir0.m_x;
	jacobian1[4] = -dir0.m_y;
	jacobian1[5] = -dir0.m_z;

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	m_motorOmega = omega0.DotProduct3(dir0);

dFloat32 xxx = 3000.0f;
	dFloat accel = (m_targetSpeed - m_motorOmega) / timestep;
	NewtonUserJointAddAngularRow(m_joint, 0.0f, &dir0[0]);
	NewtonUserJointSetRowAcceleration(m_joint, accel);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointSetRowMinimumFriction(m_joint, -xxx);
	NewtonUserJointSetRowMaximumFriction(m_joint, xxx);
}



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
dCustomMotor2::dCustomMotor2(const dVector& pin0, const dVector& pin1, NewtonBody* const body, NewtonBody* const referenceBody)
	:dCustomMotor(2, body)
	,m_motorOmega1(0.0f)
	,m_targetSpeed1(0.0f)
	,m_motorTorque1(1.0f)
	,m_referenceBody(referenceBody)
{
	dMatrix dommyMatrix;
	// calculate the local matrix for body body0
	dMatrix pinAndPivot0(dGrammSchmidt(pin0));

	CalculateLocalMatrix(pinAndPivot0, m_localMatrix0, dommyMatrix);
	m_localMatrix0.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);

	// calculate the local matrix for body body1  
	dMatrix pinAndPivot1(dGrammSchmidt(pin0));
	CalculateLocalMatrix(pinAndPivot1, dommyMatrix, m_localMatrix1);
	m_localMatrix1.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);

	NewtonBodyGetMatrix(m_referenceBody, &dommyMatrix[0][0]);
	m_pin1 = dommyMatrix.UnrotateVector(pin1);

/*
	dMatrix dommyMatrix;
	// calculate the local matrix for body body0
	dMatrix pinAndPivot0(dGetIdentityMatrix());
	pinAndPivot0.m_front = pin0;
	pinAndPivot0.m_up = pin1;
	pinAndPivot0.m_right = pinAndPivot0.m_front.CrossProduct(pinAndPivot0.m_up);
	pinAndPivot0.m_right = pinAndPivot0.m_right.Normalize();
	pinAndPivot0.m_up = pinAndPivot0.m_right.CrossProduct(pinAndPivot0.m_front);

	pinAndPivot0.m_front.m_w = 0.0f;
	pinAndPivot0.m_up.m_w = 0.0f;
	pinAndPivot0.m_right.m_w = 0.0f;

	CalculateLocalMatrix(pinAndPivot0, m_localMatrix0, m_localMatrix0);
*/
	// set as kinematic loop
	SetSolverModel(2);
}


dCustomMotor2::~dCustomMotor2()
{
}

void dCustomMotor2::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_motorOmega1, sizeof(dFloat));
	callback(userData, &m_targetSpeed1, sizeof(dFloat));
	callback(userData, &m_motorTorque1, sizeof(dFloat));
}

void dCustomMotor2::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomMotor::Serialize(callback, userData);
	callback(userData, &m_motorOmega, sizeof(dFloat));
	callback(userData, &m_targetSpeed, sizeof(dFloat));
	callback(userData, &m_motorTorque, sizeof(dFloat));
}

void dCustomMotor2::SetSpeed1(dFloat speed)
{
	m_targetSpeed1 = speed;
}

void dCustomMotor2::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dFloat jacobian0[6];
	dFloat jacobian1[6];

	dCustomMotor::SubmitConstraints(timestep, threadIndex);

	NewtonBodyGetMatrix(m_referenceBody, &matrix0[0][0]);
	dVector dir0 = matrix0.RotateVector(m_pin1);

	jacobian0[0] = 0.0f;
	jacobian0[1] = 0.0f;
	jacobian0[2] = 0.0f;
	jacobian0[3] = dir0.m_x;
	jacobian0[4] = dir0.m_y;
	jacobian0[5] = dir0.m_z;

	jacobian1[0] = 0.0f;
	jacobian1[1] = 0.0f;
	jacobian1[2] = 0.0f;
	jacobian1[3] = -dir0.m_x;
	jacobian1[4] = -dir0.m_y;
	jacobian1[5] = -dir0.m_z;

	NewtonBodyGetOmega(m_body0, &omega0[0]);
	m_motorOmega1 = omega0.DotProduct3(dir0);

dFloat32 xxx = 3000.0f;
	dFloat accel = (m_targetSpeed1 - m_motorOmega1) / timestep;
	NewtonUserJointAddAngularRow(m_joint, 0.0f, &dir0[0]);
	NewtonUserJointSetRowAcceleration(m_joint, accel);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	NewtonUserJointSetRowMinimumFriction(m_joint, -xxx);
	NewtonUserJointSetRowMaximumFriction(m_joint, xxx);
}

