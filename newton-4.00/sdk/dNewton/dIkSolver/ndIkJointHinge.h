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

#ifndef __ND_IK_JOINT_HINGE_H__
#define __ND_IK_JOINT_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

class ndIkJointHinge: public ndJointHinge
{
	public:
	D_CLASS_REFLECTION(ndIkJointHinge);
	D_NEWTON_API ndIkJointHinge(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndIkJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndIkJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndIkJointHinge();

	protected:
	D_NEWTON_API virtual bool IsIk() const;
	D_NEWTON_API virtual void SetIkSolver();
	D_NEWTON_API virtual void ResetIkSolver();
	D_NEWTON_API virtual void StopIkMotor(ndFloat32 timestep);
	D_NEWTON_API virtual bool SetIkMotor(ndFloat32 timestep, const ndJacobian& forceBody0, const ndJacobian& forceBody1);

	D_NEWTON_API void SetTorqueLimits(ndFloat32 minToque, ndFloat32 maxTorque);
	D_NEWTON_API void GetTorqueLimits(ndFloat32& minToque, ndFloat32& maxTorque) const;

	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndIkRowAccel m_axisAccel;
};

#endif 

