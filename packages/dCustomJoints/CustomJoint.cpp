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


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomJoint.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//dRttiRootClassSupportImplement(CustomJoint);



#ifdef _NEWTON_BUILD_DLL
	BOOL APIENTRY DllMain( HMODULE hModule,
						  DWORD  ul_reason_for_call,
						  LPVOID lpReserved
						  )
	{
		switch (ul_reason_for_call)
		{
			case DLL_PROCESS_ATTACH:
			case DLL_THREAD_ATTACH:
			case DLL_THREAD_DETACH:
			case DLL_PROCESS_DETACH:
				break;
		}
		return TRUE;
	}
#endif



CustomJoint::CustomJoint ()
	:m_maxDof(0)
	,m_autoDestroy(0)
	,m_userData(NULL)
	,m_body0(NULL)
	,m_body1(NULL)
	,m_joint(NULL)
	,m_world(NULL)
	,m_userDestructor(NULL)
	,m_userConstrationCallback(NULL)
{
}

CustomJoint::CustomJoint (int maxDOF, NewtonBody* const body0, NewtonBody* const body1)
{
	Init (maxDOF, body0, body1);
}

CustomJoint::~CustomJoint()
{
	//dAssert (m_joint);
	
	//if the joint has user data it means the application is destroying the joint

// if there is a C destructor call it form here
	CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (m_joint);  
	if (joint->m_userDestructor) {
		joint->m_userDestructor ((const NewtonUserJoint*) joint);
	}

//	if (NewtonJointGetUserData (m_joint)) {
	if (!m_autoDestroy) {
		// set the joint call to NULL to prevent infinite recursion 
		NewtonJointSetUserData (m_joint, NULL);  
		NewtonJointSetDestructor (m_joint, NULL);  

		// destroy this joint
		NewtonDestroyJoint(m_world, m_joint);
	}
}



void CustomJoint::Init (int maxDOF, NewtonBody* const body0, NewtonBody* const body1)
{
	m_autoDestroy = 0;
	m_joint = NULL;
	m_body0 = body0;
	m_body1 = body1;
	m_maxDof = maxDOF;
	m_world	= NewtonBodyGetWorld (body0);
	m_joint = NewtonConstraintCreateUserJoint (m_world, maxDOF, SubmitConstraints, GetInfo, m_body0, m_body1); 

	NewtonJointSetUserData (m_joint, this);
	NewtonJointSetDestructor (m_joint, Destructor);

	m_userData = NULL;
	m_userDestructor = NULL;
	m_userConstrationCallback = NULL;

}


NewtonBody* CustomJoint::GetBody0 () const
{
	return m_body0;
}

NewtonBody* CustomJoint::GetBody1 () const
{
	return m_body1;
}

NewtonJoint* CustomJoint::GetJoint () const
{
	return m_joint;
}

void CustomJoint::Destructor (const NewtonJoint* me)
{
	CustomJoint* joint;  

	// get the pointer to the joint class
	joint = (CustomJoint*) NewtonJointGetUserData (me);  

	joint->m_autoDestroy = 1;

	// delete the joint class
	delete joint;
}


void  CustomJoint::SubmitConstraints (const NewtonJoint* const me, dFloat timestep, int threadIndex)
{
	// get the pointer to the joint class
	CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (me);  

	// call the constraint call back
	joint->SubmitConstraints(timestep, threadIndex);

	// if there is a user define callback call it from here;
	if (joint->m_userConstrationCallback) {
		joint->m_userConstrationCallback ((const NewtonUserJoint*) joint, timestep, threadIndex);
	}

}

void CustomJoint::GetInfo (const NewtonJoint* const me, NewtonJointRecord* info)
{
	// get the pointer to the joint class
	CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (me);  
	joint->GetInfo(info);
}


void CustomJoint::CalculateLocalMatrix (const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const
{
	dMatrix matrix0;

	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);
	dMatrix matrix1 (GetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);
	}

	// create a global matrix at the pivot point with front vector aligned to the pin vector
	dAssert (pinsAndPivotFrame.SanityCheck());
	// calculate the relative matrix of the pin and pivot on each body
 	localMatrix0 = pinsAndPivotFrame * matrix0.Inverse();
	localMatrix1 = pinsAndPivotFrame * matrix1.Inverse();
}


void CustomJoint::CalculateGlobalMatrix (const dMatrix& localMatrix0, const dMatrix& localMatrix1, dMatrix& matrix0, dMatrix& matrix1) const
{
	dMatrix body0Matrix;
	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &body0Matrix[0][0]);

	dMatrix body1Matrix (GetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &body1Matrix[0][0]);
	}
	matrix0 = localMatrix0 * body0Matrix;
	matrix1 = localMatrix1 * body1Matrix;
}


void CustomJoint::GetInfo (NewtonJointRecord* const info) const
{
	dAssert (0);
}

void CustomJoint::SetBodiesCollisionState (int state)
{
	NewtonJointSetCollisionState (m_joint, state);
}

int CustomJoint::GetBodiesCollisionState () const
{
	return NewtonJointGetCollisionState (m_joint);
}

void CustomJoint::SubmitConstraints (dFloat timestep, int threadIndex)
{
}