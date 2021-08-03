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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointPidActuator.h"

ndJointPidActuator::ndJointPidActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBallAndSocket(pinAndPivotFrame, child, parent)
{
}

ndJointPidActuator::~ndJointPidActuator()
{
}

