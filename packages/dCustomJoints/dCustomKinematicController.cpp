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

//dInitRtti(dCustomKinematicController);

dCustomKinematicController::dCustomKinematicController(NewtonBody* const body, const dVector& handleInGlobalSpace)
	:dCustomJoint(6, body, NULL)
{
	dMatrix matrix;

	// get the initial position and orientation
	NewtonBodyGetMatrix (body, &matrix[0][0]);

	m_autoSlepState = NewtonBodyGetSleepState (body);
	NewtonBodySetAutoSleep (body, 0);

	m_localHandle = matrix.UntransformVector (handleInGlobalSpace);
	matrix.m_posit = handleInGlobalSpace;

	SetPickMode (1);
	SetTargetMatrix (matrix);
	SetMaxLinearFriction(1.0f); 
	SetMaxAngularFriction(1.0f); 

	// set as soft joint
	SetSolverModel(2);
}

dCustomKinematicController::~dCustomKinematicController()
{
	NewtonBodySetAutoSleep (m_body0, m_autoSlepState);
}


void dCustomKinematicController::SetPickMode (int mode)
{
	m_pickMode = mode ? 1 : 0;
}

void dCustomKinematicController::SetMaxLinearFriction(dFloat accel)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass (m_body0, &mass, &Ixx, &Iyy, &Izz);
	m_maxLinearFriction = dAbs (accel) * mass;
}

void dCustomKinematicController::SetMaxAngularFriction(dFloat alpha)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass (m_body0, &mass, &Ixx, &Iyy, &Izz);
//	if (Iyy > Ixx) {
//		Ixx = Iyy;
//	}
//	if (Izz > Ixx) {
//		Ixx = Izz;
//	}
//	m_maxAngularFriction = dAbs (alpha) * Ixx;
	m_maxAngularFriction = dAbs (alpha) * mass;
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


void dCustomKinematicController::SubmitConstraints (dFloat timestep, int threadIndex)
{
	// check if this is an impulsive time step
	if (timestep > 0.0f) {
		dMatrix matrix0;
		dVector v(0.0f);
		dVector w(0.0f);
		dVector cg(0.0f);

		dFloat invTimestep = 1.0f / timestep;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		NewtonBodyGetOmega (m_body0, &w[0]);
		NewtonBodyGetVelocity (m_body0, &v[0]);
		NewtonBodyGetCentreOfMass (m_body0, &cg[0]);
		NewtonBodyGetMatrix (m_body0, &matrix0[0][0]);

		dVector p0 (matrix0.TransformVector (m_localHandle));

		dVector pointVeloc (v + w.CrossProduct(matrix0.RotateVector (m_localHandle - cg)));
		dVector relPosit (m_targetPosit - p0);
		dVector relVeloc (relPosit.Scale (invTimestep) - pointVeloc);
		dVector relAccel (relVeloc.Scale (invTimestep * 0.3f)); 
			
		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &m_targetPosit[0], &matrix0.m_front[0]);
		NewtonUserJointSetRowAcceleration (m_joint, relAccel.DotProduct3(matrix0.m_front));
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);

		NewtonUserJointAddLinearRow (m_joint, &p0[0], &m_targetPosit[0], &matrix0.m_up[0]);
		NewtonUserJointSetRowAcceleration (m_joint, relAccel.DotProduct3(matrix0.m_up));
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);

		NewtonUserJointAddLinearRow (m_joint, &p0[0], &m_targetPosit[0], &matrix0.m_right[0]);
		NewtonUserJointSetRowAcceleration (m_joint, relAccel.DotProduct3(matrix0.m_right));
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);

		if (m_pickMode) {
			dQuaternion rotation;

			NewtonBodyGetRotation (m_body0, &rotation.m_q0);
			if (m_targetRot.DotProduct (rotation) < 0.0f) {
				rotation.m_q0 *= -1.0f; 
				rotation.m_q1 *= -1.0f; 
				rotation.m_q2 *= -1.0f; 
				rotation.m_q3 *= -1.0f; 
			}

			dVector relOmega (rotation.CalcAverageOmega (m_targetRot, invTimestep) - w);
			dFloat mag = relOmega.DotProduct3(relOmega);
			if (mag > 1.0e-6f) {
				dVector pin (relOmega.Scale (1.0f / mag));
				dMatrix basis (dGrammSchmidt (pin)); 	
				dFloat relSpeed = dSqrt (relOmega.DotProduct3(relOmega));
				dFloat relAlpha = relSpeed * invTimestep;

				NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis.m_front[0]);
				NewtonUserJointSetRowAcceleration (m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

				NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis.m_up[0]);
				NewtonUserJointSetRowAcceleration (m_joint, 0.0f);
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

				NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis.m_right[0]);
				NewtonUserJointSetRowAcceleration (m_joint, 0.0f);
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

			} else {

				dVector relAlpha (w.Scale (-invTimestep));
				NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);
				NewtonUserJointSetRowAcceleration (m_joint, relAlpha.DotProduct3(matrix0.m_front));
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

				NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_up[0]);
				NewtonUserJointSetRowAcceleration (m_joint, relAlpha.DotProduct3(matrix0.m_up));
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

				NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_right[0]);
				NewtonUserJointSetRowAcceleration (m_joint, relAlpha.DotProduct3(matrix0.m_right));
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);
			}

		} else {
			// this is the single handle pick mode, add some angular friction

			dVector relAlpha = w.Scale (-invTimestep);
			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAlpha.DotProduct3(matrix0.m_front));
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction * 0.025f);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction * 0.025f);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAlpha.DotProduct3(matrix0.m_up));
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction * 0.025f);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction * 0.025f);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAlpha.DotProduct3(matrix0.m_right));
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction * 0.025f);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction * 0.025f);
		}
	}
}



