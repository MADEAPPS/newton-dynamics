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

#include "CustomJointLibraryStdAfx.h"

#include "CustomRagDoll.h"
#include "CustomBallAndSocket.h"


class CustomRagDollLimbJoint: public CustomLimitBallAndSocket
{
	public:
	dAddRtti(CustomLimitBallAndSocket);

	CustomRagDollLimbJoint (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
		:CustomLimitBallAndSocket(pinAndPivotFrame, child, parent)
	{
	}

	virtual void SubmitConstraints (dFloat timestep, int threadIndex)
	{
		CustomLimitBallAndSocket::SubmitConstraints (timestep, threadIndex);
	}
};

dInitRtti(CustomRagDoll);
dInitRtti(CustomRagDollLimbJoint);



CustomRagDoll::CustomRagDoll ()
	:CustomJoint()
{
	m_bonesCount = 0;

	m_joints[0] = NULL;
	m_bonesBodies[0] = NULL;
	m_bonesParents[0] = NULL;
}



CustomRagDoll::~CustomRagDoll(void)
{
}

int CustomRagDoll::AddBone (
	NewtonWorld* const world,
	int parentBoneIndex, 
	void* const userData, 
	const dMatrix& boneMatrix, 
	dFloat mass, 
	const dMatrix& pivotInGlobalSpace, 
	const NewtonCollision* const boneCollisionShape)
{
	dAssert (0);
	return 0;
/*
	// create the rigid body that will make this bone
	NewtonBody* const bone = NewtonCreateBody (world, boneCollisionShape, &boneMatrix[0][0]);

	// calculate the moment of inertia and the relative center of mass of the solid
	dVector origin(0.0f, 0.0f, 0.0f, 1.0f);
	dVector inertia(0.0f, 0.0f, 0.0f, 1.0f);
	NewtonConvexCollisionCalculateInertialMatrix (boneCollisionShape, &inertia[0], &origin[0]);	

	// set the body center of mass
	NewtonBodySetCentreOfMass (bone, &origin[0]);


	dFloat Ixx = mass * inertia[0];
	dFloat Iyy = mass * inertia[1];
	dFloat Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (bone, mass, Ixx, Iyy, Izz);

	// set the body matrix
//	NewtonBodySetMatrix (bone, &boneMatrix[0][0]);

	// save the user data with the bone body (usually the visual geometry)
	NewtonBodySetUserData(bone, userData);

	CustomRagDollLimbJoint* joint = NULL;
	if (m_bonesCount == 0) {
		// this is the root bone, initialize ragdoll with the root Bone
		Init (1, bone, NULL);
		NewtonBodySetTransformCallback(bone, TransformCallback);
		parentBoneIndex = 0;
	} else {
		// this is a child bone, need to be connected to its parent by a joint 
		NewtonBody* const parent = m_bonesBodies[parentBoneIndex];
		joint = new CustomRagDollLimbJoint (pivotInGlobalSpace, bone, parent);
	}

	m_joints[m_bonesCount] = joint;
	m_bonesBodies[m_bonesCount] = bone;
	m_bonesParents[m_bonesCount] = m_bonesBodies[parentBoneIndex];
	m_bonesCount ++;

	return m_bonesCount - 1;
*/
}


void CustomRagDoll::GetInfo (NewtonJointRecord* const info) const
{

}

void CustomRagDoll::SubmitConstraints (dFloat timestep, int threadIndex)
{

}


const NewtonBody* CustomRagDoll::GetBone (int bodenIndex) const
{
	return m_bonesBodies[bodenIndex];
}

const NewtonBody* CustomRagDoll::GetParentBone (int bodenIndex) const
{
    return m_bonesParents[bodenIndex];
}

const CustomJoint *CustomRagDoll::GetJoint (int bodenIndex) const
{
	return m_joints[bodenIndex];
}

int CustomRagDoll::GetBoneCount () const
{
	return m_bonesCount;
}

void CustomRagDoll::SetBoneTwistLimits (int bodenIndex, dFloat minAngle, dFloat maxAngle)
{
	if (bodenIndex > 0) {
		m_joints[bodenIndex]->SetTwistAngle (minAngle, maxAngle);
	}

}


void CustomRagDoll::SetBoneConeLimits (int bodenIndex, dFloat angle)
{
	if (bodenIndex > 0) {
		m_joints[bodenIndex]->SetConeAngle (angle);
	}
}

void CustomRagDoll::SetCollisionState (int bodenIndex, int state)
{
	NewtonBodySetJointRecursiveCollision (m_bonesBodies[bodenIndex], state);
}


void CustomRagDoll::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
	// find there ragdoll joint and call the transform call back for this joint
	for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
		CustomJoint* const customJoint = (CustomJoint*) NewtonJointGetUserData(joint);
		if (customJoint->IsType(CustomRagDoll::GetRttiType())) {
			CustomRagDoll* const ragDoll = (CustomRagDoll*)customJoint;
			dAssert (body == ragDoll->m_bonesBodies[0]);
			ragDoll->CalculateLocalMatrices ();
			break;
		}
	}
}

void CustomRagDoll::CalculateLocalMatrices () const
{
	dMatrix boneMatrix;

	void* const userData = NewtonBodyGetUserData(m_bonesBodies[0]);
	NewtonBodyGetMatrix(m_bonesBodies[0], &boneMatrix[0][0]);
	ApplyBoneMatrix (0, userData, boneMatrix);

	for (int i = 1; i < m_bonesCount; i ++) {
		dMatrix parentMatrix;

		void* const userData = NewtonBodyGetUserData(m_bonesBodies[i]);
		NewtonBodyGetMatrix(m_bonesBodies[i], &boneMatrix[0][0]);
		NewtonBodyGetMatrix(m_bonesParents[i], &parentMatrix[0][0]);
//		boneMatrix = m_joints[i]->m_boneOffetMatrix * boneMatrix * parentMatrix.Inverse();
		boneMatrix = boneMatrix * parentMatrix.Inverse();
		ApplyBoneMatrix (i, userData, boneMatrix);
	}
}



