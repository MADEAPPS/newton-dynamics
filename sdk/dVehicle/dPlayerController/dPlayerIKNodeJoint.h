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

#ifndef D_CUSTOM_PLAYER_IKNODE_JOINT_H_
#define D_CUSTOM_PLAYER_IKNODE_JOINT_H_

#include "dStdafxVehicle.h"
#include "dPlayerIKNode.h"


class dPlayerIKNodeJoint: public dPlayerIKNode
{
	public:
	DVEHICLE_API dPlayerIKNodeJoint(dPlayerIKNode* const parent, void* const userData, const dMatrix& bindMatrix, NewtonCollision* const shape);
	DVEHICLE_API ~dPlayerIKNodeJoint();
};

#endif 

