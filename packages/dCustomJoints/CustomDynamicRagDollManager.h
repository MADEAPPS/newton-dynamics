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

#include <CustomJointLibraryStdAfx.h>
#include <CustomBallAndSocket.h>
#include <CustomArcticulatedTransformManager.h>


#define DYNAMIC_RAGDOLL_PLUGIN_NAME	"__dynamicRagDollManager__"

class DynamicRagDollJoint: public CustomBallAndSocket
{
	public:
	CUSTOM_JOINTS_API DynamicRagDollJoint(const dMatrix& globalChildPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~DynamicRagDollJoint();

	CUSTOM_JOINTS_API void SetConeAngle(dFloat angle);
	CUSTOM_JOINTS_API void SetTwistAngle(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API dFloat GetConeAngle() const;
	CUSTOM_JOINTS_API void GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	CUSTOM_JOINTS_API DynamicRagDollJoint(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
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
	DECLARE_CUSTOM_JOINT(DynamicRagDollJoint, CustomBallAndSocket)
};

class CustomDynamicRagDollManager: public CustomArticulaledTransformManager
{
	public:
	CUSTOM_JOINTS_API CustomDynamicRagDollManager(NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~CustomDynamicRagDollManager();

	CUSTOM_JOINTS_API virtual void Debug () const;

	CUSTOM_JOINTS_API virtual CustomArticulatedTransformController* CreateTransformController (void* const userData);

	CUSTOM_JOINTS_API virtual void OnPreUpdate (CustomArticulatedTransformController* const constroller, dFloat timestep, int threadIndex) const;
	CUSTOM_JOINTS_API virtual void OnUpdateTransform (const CustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const;
};


#endif 

