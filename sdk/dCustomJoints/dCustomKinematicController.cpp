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


// dCustomKinematicController.cpp: implementation of the dCustomKinematicController class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomKinematicController.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_JOINT(dCustomKinematicController)

dCustomKinematicController::dCustomKinematicController(NewtonBody* const body, const dVector& handleInGlobalSpace)
	:dCustomJoint(6, body, NULL)
{
	dMatrix matrix;
	NewtonBodyGetMatrix (body, &matrix[0][0]);
	matrix.m_posit = handleInGlobalSpace;
	Init (body, matrix);
}

dCustomKinematicController::dCustomKinematicController (NewtonBody* const body, const dMatrix& attachmentMatrixInGlobalSpace)
	:dCustomJoint(6, body, NULL)
{
	Init (body, attachmentMatrixInGlobalSpace);
}

void dCustomKinematicController::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	dAssert (0);
}

void dCustomKinematicController::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dAssert (0);
}

dCustomKinematicController::~dCustomKinematicController()
{
	//NewtonBodySetAutoSleep (m_body0, m_autoSleepState);
	//NewtonBodySetSleepState(body, 0);
}

void dCustomKinematicController::Init (NewtonBody* const body, const dMatrix& matrix)
{
	CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);

	m_autoSleepState = NewtonBodyGetSleepState(body);
	//NewtonBodySetAutoSleep(body, 0);
	NewtonBodySetSleepState(body, 0);

	SetPickMode(1);
	SetTargetMatrix(matrix);
	SetMaxLinearFriction(1.0f);
	SetMaxAngularFriction(1.0f);

	// set as soft joint
	SetSolverModel(2);
}

void dCustomKinematicController::SetPickMode (int mode)
{
	m_pickMode = mode ? 1 : 0;
}

void dCustomKinematicController::SetMaxLinearFriction(dFloat frictionForce)
{
	m_maxLinearFriction = dAbs (frictionForce);
}

void dCustomKinematicController::SetMaxAngularFriction(dFloat frictionTorque)
{
	m_maxAngularFriction = dAbs (frictionTorque);
}


void dCustomKinematicController::SetTargetRotation(const dQuaternion& rotation)
{
	NewtonBodySetSleepState(m_body0, 0);
	m_targetRot = rotation;
}

void dCustomKinematicController::SetTargetPosit(const dVector& posit)
{
	NewtonBodySetSleepState(m_body0, 0);
	m_targetPosit = posit;
}

void dCustomKinematicController::SetTargetMatrix(const dMatrix& matrix)
{
	SetTargetRotation (matrix);
	SetTargetPosit (matrix.m_posit);
}

dMatrix dCustomKinematicController::GetTargetMatrix () const
{
	return dMatrix (m_targetRot, m_targetPosit);
}


void dCustomKinematicController::Debug(dDebugDisplay* const debugDisplay) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	matrix1 = dMatrix (m_targetRot, m_targetPosit);

	debugDisplay->DrawFrame(matrix0);
	debugDisplay->DrawFrame(matrix1);
}

void dCustomKinematicController::SubmitConstraints (dFloat timestep, int threadIndex)
{
	// check if this is an impulsive time step
	dAssert (timestep > 0.0f);

	dMatrix matrix0;
	dMatrix matrix1;
	dVector veloc(0.0f);
	dVector omega(0.0f);
	dVector com(0.0f);
	dVector pointVeloc(0.0f);

	dFloat invTimestep = 1.0f / timestep;

	CalculateGlobalMatrix(matrix0, matrix1);

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	dVector relPosit (m_targetPosit - matrix0.m_posit);
	dFloat mag2 = relPosit.DotProduct3(relPosit);
	if (mag2 > 1.0e-4f) {
		dMatrix dirMatrix (dGrammSchmidt(relPosit));

		NewtonBodyGetPointVelocity(m_body0, &m_targetPosit[0], &pointVeloc[0]);
		
		dFloat speed = pointVeloc.DotProduct3(dirMatrix.m_front);
		dFloat dist = relPosit.DotProduct3(dirMatrix.m_front) * 0.3f;
		
		dFloat relSpeed = dist * invTimestep - speed;
		dFloat relAccel = relSpeed * invTimestep;

		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &dirMatrix.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, relAccel);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxLinearFriction);

		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &dirMatrix.m_up[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxLinearFriction);

		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &dirMatrix.m_right[0]);
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxLinearFriction);

	} else {
		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &m_targetPosit[0], &matrix0.m_front[0]);
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);

		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &m_targetPosit[0], &matrix0.m_up[0]);
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);

		NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &m_targetPosit[0], &matrix0.m_right[0]);
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);
	}

	if (m_pickMode) {
		dQuaternion rotation (matrix0);
		if (m_targetRot.DotProduct (rotation) < 0.0f) {
			rotation.Scale(-1.0f);
		}

		dFloat dot = rotation.DotProduct(m_targetRot);
		if (dot < 0.9995f) {
			dMatrix rot (dGrammSchmidt(dVector (rotation.m_q1, rotation.m_q2, rotation.m_q3)));
			dFloat angle = 2.0f * dAcos(dClamp(rotation.m_q0, dFloat(-1.0f), dFloat(1.0f)));

			NewtonUserJointAddAngularRow (m_joint, -angle, &rot.m_front[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &rot.m_up[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &rot.m_right[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

		} else {
			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);
		}
	}
}



