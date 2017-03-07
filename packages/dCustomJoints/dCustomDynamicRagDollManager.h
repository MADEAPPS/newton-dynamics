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


//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_DYNAMIC_RAGDOLL_MANAGER_H_
#define D_CUSTOM_DYNAMIC_RAGDOLL_MANAGER_H_

#include <dCustomJointLibraryStdAfx.h>
#include <dCustomBallAndSocket.h>
#include <dCustomArcticulatedTransformManager.h>


#define DYNAMIC_RAGDOLL_PLUGIN_NAME	"__dynamicRagDollManager__"

class dDynamicRagDollJoint: public dCustomBallAndSocket
{
	public:
	CUSTOM_JOINTS_API dDynamicRagDollJoint(const dMatrix& globalChildPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~dDynamicRagDollJoint();

	CUSTOM_JOINTS_API void SetConeAngle(dFloat angle);
	CUSTOM_JOINTS_API void SetTwistAngle(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetConeAngle() const;
	CUSTOM_JOINTS_API void GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	CUSTOM_JOINTS_API dDynamicRagDollJoint(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void GetInfo(NewtonJointRecord* const info) const;

	dMatrix m_rotationOffset;
	dFloat m_coneAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	dFloat m_coneAngleCos;
	dFloat m_coneAngleSin;
	dFloat m_coneAngleHalfCos;
	dFloat m_coneAngleHalfSin;
	DECLARE_CUSTOM_JOINT(dDynamicRagDollJoint, dCustomBallAndSocket)
};

class dCustomDynamicRagDollManager: public dCustomArticulaledTransformManager
{
	public:
	CUSTOM_JOINTS_API dCustomDynamicRagDollManager(NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~dCustomDynamicRagDollManager();

	CUSTOM_JOINTS_API virtual void Debug () const;

	CUSTOM_JOINTS_API virtual dCustomArticulatedTransformController* CreateTransformController (void* const userData);

	CUSTOM_JOINTS_API virtual void OnPreUpdate (dCustomArticulatedTransformController* const constroller, dFloat timestep, int threadIndex) const;
	CUSTOM_JOINTS_API virtual void OnUpdateTransform (const dCustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const;
};


#endif 

