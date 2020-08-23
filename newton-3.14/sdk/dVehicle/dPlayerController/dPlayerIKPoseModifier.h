/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// NewtonPlayerControllerManager.h: interface for the NewtonPlayerControllerManager class.
//
//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_PLAYER_IK_POSE_MODIFIER_H_
#define D_CUSTOM_PLAYER_IK_POSE_MODIFIER_H_

#include "dStdafxVehicle.h"
#include "dPlayerIKNode.h"

class dPlayerController;


class dPlayerIKPoseModifier: public dPlayerIKNode, public dAnimationBlendTreeNode
{
	public:
	DVEHICLE_API dPlayerIKPoseModifier(dPlayerController* const controller, dAnimationBlendTreeNode* const input);
	DVEHICLE_API virtual ~dPlayerIKPoseModifier();

	DVEHICLE_API void Init (void* const userData, const dMatrix& bindMatrix, NewtonCollision* const shape);
	DVEHICLE_API void Finalize();

	DVEHICLE_API void *operator new (size_t size);
	DVEHICLE_API void operator delete (void* ptr);

	DVEHICLE_API virtual const void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	protected: 
	DVEHICLE_API virtual void Evaluate(dAnimationPose& output, dFloat timestep);

	private:
	int GetNodeArray (dPlayerIKNode** const array) const;

	protected:
	dPlayerController* m_controller; 
};

#endif 

