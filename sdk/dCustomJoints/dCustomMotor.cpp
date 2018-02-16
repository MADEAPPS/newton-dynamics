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

dCustomMotor::dCustomMotor(int dof, NewtonBody* const child)
	:dCustomJoint(dof, child, NULL)
	,m_targetSpeed(0.0f)
	,m_motorOmega(0.0f)
	,m_motorTorque(1.0f)
{
//	SetSolverModel(2);
}


dCustomMotor::dCustomMotor(const dVector& pin, NewtonBody* const body)
	:dCustomJoint(1, body, NULL)
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
	dVector dir0 = matrix0.m_front;

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

	dFloat accel = (m_targetSpeed - m_motorOmega) / timestep;
	NewtonUserJointAddAngularRow(m_joint, 0.0f, &dir0[0]);
	NewtonUserJointSetRowAcceleration(m_joint, accel);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	//	NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
	//	NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
}


