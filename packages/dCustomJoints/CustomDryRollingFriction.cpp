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


// CustomUpVector.cpp: implementation of the CustomUpVector class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomDryRollingFriction.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//dInitRtti(CustomDryRollingFriction);

CustomDryRollingFriction::CustomDryRollingFriction(NewtonBody* child, dFloat radius, dFloat coefficient)
	:CustomJoint(1, child, NULL)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMass (child, &mass, &Ixx, &Iyy, &Izz);

	m_frictionCoef = coefficient;
	m_frictionTorque = Ixx * radius;
}


CustomDryRollingFriction::~CustomDryRollingFriction()
{
}


// rolling friction works as follow: the idealization of the contact of a spherical object 
// with a another surface is a point that pass by the center of the sphere.
// in most cases this is enough to model the collision but in insufficient for modeling 
// the rolling friction. In reality contact with the sphere with the other surface is not 
// a point but a contact patch. A contact patch has the property the it generates a fix 
// constant rolling torque that opposes the movement of the sphere.
// we can model this torque by adding a clamped torque aligned to the instantaneously axis 
// of rotation of the ball. and with a magnitude of the stopping angular acceleration.
void CustomDryRollingFriction::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dVector omega(0.0f);

	// get the omega vector
	NewtonBodyGetOmega(m_body0, &omega[0]);

	dFloat omegaMag = dSqrt (omega.DotProduct3(omega));
	if (omegaMag > 0.1f) {
		// tell newton to used this the friction of the omega vector to apply the rolling friction
		dVector pin (omega.Scale (1.0f / omegaMag));
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &pin[0]);

		// calculate the acceleration to stop the ball in one time step
		dFloat invTimestep = (timestep > 0.0f) ? 1.0f / timestep: 1.0f;
		NewtonUserJointSetRowAcceleration (m_joint, -omegaMag * invTimestep);

		// set the friction limit proportional the sphere Inertia
		dFloat torqueFriction = m_frictionTorque * m_frictionCoef;
		NewtonUserJointSetRowMinimumFriction (m_joint, -torqueFriction);
		NewtonUserJointSetRowMaximumFriction (m_joint, torqueFriction);

	} else {
		// when omega is too low sheath a little bit and damp the omega directly
		omega = omega.Scale (0.2f);
		NewtonBodySetOmega(m_body0, &omega[0]);
	}
}


void CustomDryRollingFriction::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, GetTypeName());

	info->m_attachBody_0 = m_body0;
	info->m_attachBody_1 = m_body1;

	info->m_minLinearDof[0] = -D_CUSTOM_LARGE_VALUE;
	info->m_maxLinearDof[0] = D_CUSTOM_LARGE_VALUE;

	info->m_minLinearDof[1] = -D_CUSTOM_LARGE_VALUE;
	info->m_maxLinearDof[1] = D_CUSTOM_LARGE_VALUE;

	info->m_minLinearDof[2] = -D_CUSTOM_LARGE_VALUE;
	info->m_maxLinearDof[2] = D_CUSTOM_LARGE_VALUE;

	info->m_minAngularDof[0] = -D_CUSTOM_LARGE_VALUE;
	info->m_maxAngularDof[0] =  D_CUSTOM_LARGE_VALUE;

	info->m_minAngularDof[1] = -D_CUSTOM_LARGE_VALUE;;
	info->m_maxAngularDof[1] =  D_CUSTOM_LARGE_VALUE;

	info->m_minAngularDof[2] = -D_CUSTOM_LARGE_VALUE;;
	info->m_maxAngularDof[2] =  D_CUSTOM_LARGE_VALUE;

	dMatrix matrix (dGetIdentityMatrix());
	memcpy (info->m_attachmenMatrix_0, &matrix[0][0], sizeof (dMatrix));

	// note this is not a bug
	memcpy (info->m_attachmenMatrix_1, &matrix[0][0], sizeof (dMatrix));
}


