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


class dPlayerIKPoseModifier: public dPlayerIKNode, public dAnimationBlendTreeNode
{
	public:
	DVEHICLE_API dPlayerIKPoseModifier(dAnimationBlendTreeNode* const input);
	DVEHICLE_API virtual ~dPlayerIKPoseModifier();

	DVEHICLE_API void Init (dVehicleNode* const parent, void* const userData, const dMatrix& bindMatrix, NewtonCollision* const shape);

	DVEHICLE_API void *operator new (size_t size);
	DVEHICLE_API void operator delete (void* ptr);

	protected: 
	DVEHICLE_API virtual void Evaluate(dAnimationPose& output, dFloat timestep);


};

#endif 

