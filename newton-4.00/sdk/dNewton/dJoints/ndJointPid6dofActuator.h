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

#ifndef __D_JOINT_PID_6DOF_ACTUATOR_H__
#define __D_JOINT_PID_6DOF_ACTUATOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointPid3dofActuator.h"

//#define D_PID_PENETRATION_RECOVERY_LINEAR_SPEED dFloat32 (0.1f) 
//#define D_PID_PENETRATION_LINEAR_LIMIT dFloat32 (10.0f * dDegreeToRad) 

class ndJointPid6dofActuator : public ndJointPid3dofActuator
{
	public:
	D_CLASS_RELECTION(ndJointPid6dofActuator);

	D_NEWTON_API ndJointPid6dofActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPid6dofActuator();

	protected:
	virtual void SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
};

#endif 

