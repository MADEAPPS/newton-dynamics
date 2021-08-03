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

#ifndef __D_JOINT_BALLANDSOCKET_ACTUATOR_H__
#define __D_JOINT_BALLANDSOCKET_ACTUATOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBallAndSocket.h"

class ndJointPidActuator: public ndJointBallAndSocket
{
	public:
	D_CLASS_RELECTION(ndJointPidActuator);

	D_NEWTON_API ndJointPidActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPidActuator();

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

};

#endif 

