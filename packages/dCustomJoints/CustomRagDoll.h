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


// CustomRagDoll.h: interface for the CustomRagDoll class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOM_RAGDOLL_H) 
#define AFX_CUSTOM_RAGDOLL_H

#include "CustomJoint.h"

#define MAX_BONES	128

class CustomRagDollLimbJoint;

class CustomRagDoll: public CustomJoint  
{
	public:
	dAddRtti(CustomJoint);

	CustomRagDoll();
	virtual ~CustomRagDoll(void);

	const NewtonBody* GetBone (int bodenIndex) const;
	const NewtonBody* GetParentBone (int bodenIndex) const;
	const CustomJoint* GetJoint (int bodenIndex) const;
	int AddBone (NewtonWorld* const world, int parentBoneIndex, void* const userData, const dMatrix& boneMatrix, dFloat mass, const dMatrix& pivotInGlobalSpace, const NewtonCollision* const collisionShape);

	int GetBoneCount () const;
	void SetBoneConeLimits (int bodenIndex, dFloat angle);
	void SetBoneTwistLimits (int bodenIndex, dFloat minAngle, dFloat maxAngle);

	void SetCollisionState (int bodenIndex, int state);


	protected:
	virtual void ApplyBoneMatrix (int boneIndex, void* userData, const dMatrix& matrix) const = 0;

	private:
	void CalculateLocalMatrices () const;
	static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* const info) const;

	int m_bonesCount;
	NewtonBody* m_bonesBodies[MAX_BONES];
	NewtonBody* m_bonesParents[MAX_BONES];
	CustomRagDollLimbJoint* m_joints[MAX_BONES];
};


#endif
