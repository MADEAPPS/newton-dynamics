/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __ND_JOINT_IK_BALL_AND_SOCKET_H__
#define __ND_JOINT_IK_BALL_AND_SOCKET_H__

#include "ndNewtonStdafx.h"
#include "ndJointBallAndSocket.h"

class ndJointIkBallAndSocket: public ndJointBallAndSocket
{
	public:
	D_CLASS_REFLECTION(ndJointIkBallAndSocket);
	D_NEWTON_API ndJointIkBallAndSocket(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointIkBallAndSocket(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointIkBallAndSocket();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
};

#endif 

