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



// CustomWormGear.cpp: implementation of the CustomWormGear class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomRackAndPinion.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//dInitRtti(CustomRackAndPinion);
IMPLEMENT_CUSTON_JOINT(CustomRackAndPinion);

CustomRackAndPinion::CustomRackAndPinion(dFloat gearRatio, const dVector& rotationalPin, const dVector& linearPin, NewtonBody* rotationalBody, NewtonBody* linearBody)
	:CustomJoint(1, rotationalBody, linearBody)
{
	m_gearRatio = gearRatio;

	dMatrix dommyMatrix;

	dMatrix pinAndPivot0 (dGrammSchmidt (rotationalPin));
	CalculateLocalMatrix (pinAndPivot0, m_localMatrix0, dommyMatrix);
	m_localMatrix0.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	// calculate the local matrix for body body1  
	dMatrix pinAndPivot1 (dGrammSchmidt(linearPin));
	CalculateLocalMatrix (pinAndPivot1, dommyMatrix, m_localMatrix1);
	m_localMatrix1.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
}

CustomRackAndPinion::~CustomRackAndPinion()
{
}

CustomRackAndPinion::CustomRackAndPinion (NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:CustomJoint(child, parent, callback, userData)
{
	callback (userData, &m_gearRatio, sizeof (dFloat));
}

void CustomRackAndPinion::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	CustomJoint::Serialize (callback, userData);
	callback (userData, &m_gearRatio, sizeof (dFloat));
}



void CustomRackAndPinion::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	dVector omega0(0.0f);
	dVector veloc1(0.0f);
	dFloat jacobian0[6];
	dFloat jacobian1[6];

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);
	
	// calculate the angular velocity for both bodies
	NewtonBodyGetOmega(m_body0, &omega0[0]);
	NewtonBodyGetVelocity(m_body1, &veloc1[0]);

	// get angular velocity relative to the pin vector
	dFloat w0 = omega0 % matrix0.m_front;
	dFloat w1 = veloc1 % matrix1.m_front;

	// establish the gear equation.
	dFloat relVeloc = w0 + m_gearRatio * w1;
	if (m_gearRatio > dFloat (1.0f)) {
		relVeloc = w0 / m_gearRatio +  w1;
	}

	// calculate the relative angular acceleration by dividing by the time step

	// ideally relative acceleration should be zero, but is practice there will always 
	// be a small difference in velocity that need to be compensated. 
	// using the full acceleration will make the to over show a oscillate 
	// so use only fraction of the acceleration

	// check if this is an impulsive time step
	dFloat invTimestep = (timestep > 0.0f) ? 1.0f / timestep: 1.0f;
	dFloat relAccel = - 0.3f * relVeloc * invTimestep;

	// set the linear part of Jacobian 0 to zero	
	jacobian0[0] = 0.0f;
	jacobian0[1] = 0.0f;	 
	jacobian0[2] = 0.0f;

	// set the angular part of Jacobian 0 pin vector		
	jacobian0[3] = matrix0.m_front[0];
	jacobian0[4] = matrix0.m_front[1];
	jacobian0[5] = matrix0.m_front[2];

	// set the linear part of Jacobian 1 to translational pin vector	
	jacobian1[0] = matrix1.m_front[0];
	jacobian1[1] = matrix1.m_front[1];
	jacobian1[2] = matrix1.m_front[2];

	// set the rotational part of Jacobian 1 to zero
	jacobian1[3] = 	0.0f;
	jacobian1[4] = 	0.0f;
	jacobian1[5] = 	0.0f;

	// add a angular constraint
	NewtonUserJointAddGeneralRow (m_joint, jacobian0, jacobian1);

	// set the desired angular acceleration between the two bodies
	NewtonUserJointSetRowAcceleration (m_joint, relAccel);
}


void CustomRackAndPinion::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, GetTypeName());

	info->m_extraParameters[0] = m_gearRatio;

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

	memcpy (info->m_attachmenMatrix_0, &m_localMatrix0, sizeof (dMatrix));
	memcpy (info->m_attachmenMatrix_1, &m_localMatrix1, sizeof (dMatrix));
}
