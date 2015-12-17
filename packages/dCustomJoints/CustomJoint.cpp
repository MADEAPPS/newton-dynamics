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

CustomJoint::SerializeMetaData m_metaData("CustomJoint");

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CustomJoint::CustomJoint ()
	:m_userData(NULL)
	,m_body0(NULL)
	,m_body1(NULL)
	,m_joint(NULL)
	,m_world(NULL)
	,m_userDestructor(NULL)
	,m_userConstrationCallback(NULL)
	,m_maxDof(0)
	,m_autoDestroy(0)
{
}

CustomJoint::CustomJoint (int maxDOF, NewtonBody* const body0, NewtonBody* const body1)
{
	Init (maxDOF, body0, body1);
}

CustomJoint::CustomJoint (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData)
	:m_userData(NULL)
	,m_body0(body0)
	,m_body1(body1)
	,m_joint(NULL)
	,m_world(NULL)
	,m_userDestructor(NULL)
	,m_userConstrationCallback(NULL)
	,m_maxDof(0)
	,m_autoDestroy(0)
{
	callback (userData, &m_localMatrix0, sizeof (m_localMatrix0));
	callback (userData, &m_localMatrix1, sizeof (m_localMatrix1));
	callback (userData, &m_maxDof, sizeof (m_maxDof));
	Init (m_maxDof, body0, body1);
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

void CustomJoint::Initalize(NewtonWorld* const world)
{
	NewtonSetJointSerializationCallbacks (world, Serialize, Deserialize);
}

void CustomJoint::Init (int maxDOF, NewtonBody* const body0, NewtonBody* const body1)
{
	m_autoDestroy = 0;
	m_joint = NULL;
	m_body0 = body0;
	m_body1 = body1;
	m_maxDof = maxDOF;
	m_world	= body0 ? NewtonBodyGetWorld (body0) : NewtonBodyGetWorld (body1);
	m_joint = NewtonConstraintCreateUserJoint (m_world, maxDOF, SubmitConstraints, GetInfo, m_body0, m_body1); 

	NewtonJointSetUserData (m_joint, this);
	NewtonJointSetDestructor (m_joint, Destructor);

	m_userData = NULL;
	m_userDestructor = NULL;
	m_userConstrationCallback = NULL;
}


const dMatrix& CustomJoint::GetMatrix0() const
{
	return m_localMatrix0;
}

const dMatrix& CustomJoint::GetMatrix1() const
{
	return m_localMatrix0;
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
	// get the pointer to the joint class
	CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (me);  

	joint->m_autoDestroy = 1;

	// delete the joint class
	delete joint;
}


void  CustomJoint::SubmitConstraints (const NewtonJoint* const me, dFloat timestep, int threadIndex)
{
	// get the pointer to the joint class
	if (timestep != 0.0f) {
		CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (me);  

		// call the constraint call back
		joint->SubmitConstraints(timestep, threadIndex);

		// if there is a user define callback call it from here;
		if (joint->m_userConstrationCallback) {
			joint->m_userConstrationCallback ((const NewtonUserJoint*) joint, timestep, threadIndex);
		}
	}
}

void CustomJoint::GetInfo (const NewtonJoint* const me, NewtonJointRecord* info)
{
	// get the pointer to the joint class
	CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (me);  
	joint->GetInfo(info);
}

void CustomJoint::Serialize (const NewtonJoint* const me, NewtonSerializeCallback callback, void* const userData)
{
	CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (me);  

	dCRCTYPE key = joint->GetSerializeKey();
	callback (userData, &key, sizeof (key));

	const SerializeMetaDataDictionary& dictionary = GetDictionary();
	SerializeMetaDataDictionary::dTreeNode* const node = dictionary.Find(key); 
	if (node) {
		SerializeMetaData* const meta = node->GetInfo();
		meta->SerializeJoint(joint, callback, userData);
	}
}

void CustomJoint::Deserialize (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData)
{
	dCRCTYPE key;
	callback (userData, &key, sizeof (key));
	const SerializeMetaDataDictionary& dictionary = GetDictionary();

	SerializeMetaDataDictionary::dTreeNode* const node = dictionary.Find(key); 
	if (node) {
		SerializeMetaData* const meta = node->GetInfo();
		meta->DeserializeJoint (body0, body1, callback, userData);
	}
}

void CustomJoint::CalculateLocalMatrix (const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const
{
	dMatrix matrix0;

	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);
	dMatrix matrix1 (dGetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);
	}

	// create a global matrix at the pivot point with front vector aligned to the pin vector
	dAssert (pinsAndPivotFrame.SanityCheck());
	// calculate the relative matrix of the pin and pivot on each body
 	localMatrix0 = pinsAndPivotFrame * matrix0.Inverse();
	localMatrix1 = pinsAndPivotFrame * matrix1.Inverse();
}


void CustomJoint::CalculateGlobalMatrix (dMatrix& matrix0, dMatrix& matrix1) const
{
	dMatrix body0Matrix;
	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &body0Matrix[0][0]);

	dMatrix body1Matrix (dGetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &body1Matrix[0][0]);
	}
	matrix0 = m_localMatrix0 * body0Matrix;
	matrix1 = m_localMatrix1 * body1Matrix;
}

dFloat CustomJoint::CalculateAngle (const dVector& dir, const dVector& cosDir, const dVector& sinDir, dFloat& sinAngle, dFloat& cosAngle) const
{
	cosAngle = dir % cosDir;
	sinAngle = (dir * cosDir) % sinDir;
	return dAtan2(sinAngle, cosAngle);
}

dFloat CustomJoint::CalculateAngle (const dVector& dir, const dVector& cosDir, const dVector& sinDir) const
{
	dFloat sinAngle;
	dFloat cosAngle;
	return CalculateAngle (dir, cosDir, sinDir, sinAngle, cosAngle);
}

/*
void CustomJoint::CalculatePitchAngle(const dMatrix& matrix0, const dMatrix& matrix1, dFloat& sinAngle, dFloat& cosAngle) const
{
	dAssert (0);
	sinAngle = (matrix1.m_up * matrix0.m_up) % matrix1.m_front;
	cosAngle = matrix1.m_up % matrix0.m_up;
}

void CustomJoint::CalculateYawAngle(const dMatrix& matrix0, const dMatrix& matrix1, dFloat& sinAngle, dFloat& cosAngle) const
{
	dAssert (0);
	sinAngle = (matrix1.m_front * matrix0.m_front) % matrix1.m_up;
	cosAngle = matrix1.m_front % matrix0.m_front;
}

void CustomJoint::CalculateRollAngle(const dMatrix& matrix0, const dMatrix& matrix1, dFloat& sinAngle, dFloat& cosAngle) const
{
	dAssert (0);
	sinAngle = (matrix1.m_front * matrix0.m_front) % matrix1.m_right;
	cosAngle = matrix1.m_front % matrix0.m_front;
}
*/

void CustomJoint::GetInfo (NewtonJointRecord* const info) const
{
	strcpy (info->m_descriptionType, GetTypeName());
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


const char* CustomJoint::GetTypeName() const
{
	return "CustomJoint";
}

dCRCTYPE CustomJoint::GetSerializeKey() const
{
	return dCRC64(CustomJoint::GetTypeName());
}


CustomJoint::SerializeMetaData::SerializeMetaData(const char* const name)
{
	CustomJoint::GetDictionary().Insert(this, dCRC64(name));
}


void CustomJoint::SerializeMetaData::SerializeJoint (CustomJoint* const joint, NewtonSerializeCallback callback, void* const userData)
{
	dAssert (0);
}

CustomJoint* CustomJoint::SerializeMetaData::DeserializeJoint (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData)
{
	dAssert (0);
	return NULL;
}

CustomJoint::SerializeMetaDataDictionary& CustomJoint::GetDictionary()
{
	static SerializeMetaDataDictionary dictionary;
	return dictionary;
}


void CustomJoint::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	callback (userData, &m_localMatrix0, sizeof (m_localMatrix0));
	callback (userData, &m_localMatrix1, sizeof (m_localMatrix1));
	callback (userData, &m_maxDof, sizeof (m_maxDof));
}

