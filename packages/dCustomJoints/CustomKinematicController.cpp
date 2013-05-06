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


// CustomKinematicController.cpp: implementation of the CustomKinematicController class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomKinematicController.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dInitRtti(CustomKinematicController);

CustomKinematicController::CustomKinematicController(NewtonBody* const body, const dVector& handleInGlobalSpace)
	:CustomJoint(6, body, NULL)
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
}

CustomKinematicController::~CustomKinematicController()
{
	NewtonBodySetAutoSleep (m_body0, m_autoSlepState);
}


void CustomKinematicController::SetPickMode (int mode)
{
	m_pickMode = mode ? 1 : 0;
}

void CustomKinematicController::SetMaxLinearFriction(dFloat accel)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMassMatrix (m_body0, &mass, &Ixx, &Iyy, &Izz);
	m_maxLinearFriction = dAbs (accel) * mass;
}

void CustomKinematicController::SetMaxAngularFriction(dFloat alpha)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMassMatrix (m_body0, &mass, &Ixx, &Iyy, &Izz);
//	if (Iyy > Ixx) {
//		Ixx = Iyy;
//	}
//	if (Izz > Ixx) {
//		Ixx = Izz;
//	}
//	m_maxAngularFriction = dAbs (alpha) * Ixx;
	m_maxAngularFriction = dAbs (alpha) * mass;
}


void CustomKinematicController::SetTargetRotation(const dQuaternion& rotation)
{
	m_targetRot = rotation;
}

void CustomKinematicController::SetTargetPosit(const dVector& posit)
{
	m_targetPosit = posit;
}

void CustomKinematicController::SetTargetMatrix(const dMatrix& matrix)
{
	SetTargetRotation (matrix);
	SetTargetPosit (matrix.m_posit);
}

dMatrix CustomKinematicController::GetTargetMatrix () const
{
	return dMatrix (m_targetRot, m_targetPosit);
}


void CustomKinematicController::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, "pickBody");
/*
	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	info->m_minLinearDof[0] = -FLT_MAX;
	info->m_maxLinearDof[0] = FLT_MAX;

	info->m_minLinearDof[1] = -FLT_MAX;
	info->m_maxLinearDof[1] = FLT_MAX;

	info->m_minLinearDof[2] = -FLT_MAX;
	info->m_maxLinearDof[2] = FLT_MAX;

	info->m_minAngularDof[0] = -FLT_MAX;
	info->m_maxAngularDof[0] = FLT_MAX;

	info->m_minAngularDof[1] = 0.0f;
	info->m_maxAngularDof[1] = 0.0f;

	info->m_minAngularDof[2] = 0.0f;
	info->m_maxAngularDof[2] = 0.0f;

	info->m_bodiesCollisionOn = 1;

	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));

	// note this is not a bug
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix0, sizeof (dMatrix));
*/
}


void CustomKinematicController::SubmitConstraints (dFloat timestep, int threadIndex)
{

	// check if this is an impulsive time step
	
	if (timestep > 0.0f) {
		dVector v;
		dVector w;
		dVector cg;
		dMatrix matrix0;

		dFloat invTimestep = 1.0f / timestep;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		NewtonBodyGetOmega (m_body0, &w[0]);
		NewtonBodyGetVelocity (m_body0, &v[0]);
		NewtonBodyGetCentreOfMass (m_body0, &cg[0]);
		NewtonBodyGetMatrix (m_body0, &matrix0[0][0]);

		dVector p0 (matrix0.TransformVector (m_localHandle));

		dVector pointVeloc (v + w * matrix0.RotateVector (m_localHandle - cg));
		dVector relPosit (m_targetPosit - p0);
		dVector relVeloc (relPosit.Scale (invTimestep) - pointVeloc);
		dVector relAccel (relVeloc.Scale (invTimestep * 0.3f)); 
			
		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &m_targetPosit[0], &matrix0.m_front[0]);
		NewtonUserJointSetRowAcceleration (m_joint, relAccel % matrix0.m_front);
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);

		NewtonUserJointAddLinearRow (m_joint, &p0[0], &m_targetPosit[0], &matrix0.m_up[0]);
		NewtonUserJointSetRowAcceleration (m_joint, relAccel % matrix0.m_up);
		NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxLinearFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxLinearFriction);

		NewtonUserJointAddLinearRow (m_joint, &p0[0], &m_targetPosit[0], &matrix0.m_right[0]);
		NewtonUserJointSetRowAcceleration (m_joint, relAccel % matrix0.m_right);
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
			dFloat mag = relOmega % relOmega;
			if (mag > 1.0e-6f) {
				dVector pin (relOmega.Scale (1.0f / mag));
				dMatrix basis (dgGrammSchmidt (pin)); 	
				dFloat relSpeed = dSqrt (relOmega % relOmega);
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
				NewtonUserJointSetRowAcceleration (m_joint, relAlpha % matrix0.m_front);
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

				NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_up[0]);
				NewtonUserJointSetRowAcceleration (m_joint, relAlpha % matrix0.m_up);
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);

				NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_right[0]);
				NewtonUserJointSetRowAcceleration (m_joint, relAlpha % matrix0.m_right);
				NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction);
			}

		} else {
			// this is the single handle pick mode, add some angular friction

			dVector relAlpha = w.Scale (-invTimestep);
			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAlpha % matrix0.m_front);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction * 0.025f);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction * 0.025f);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_up[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAlpha % matrix0.m_up);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction * 0.025f);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction * 0.025f);

			NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_right[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAlpha % matrix0.m_right);
			NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxAngularFriction * 0.025f);
			NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxAngularFriction * 0.025f);
		}
	}
}



