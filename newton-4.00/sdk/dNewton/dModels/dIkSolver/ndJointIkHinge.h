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

#ifndef __ND_JOINT_IK_HINGE_H__
#define __ND_JOINT_IK_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

class ndJointIkHinge: public ndJointHinge
{
	public:
	D_CLASS_REFLECTION(ndJointIkHinge);
	D_NEWTON_API ndJointIkHinge(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointIkHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointIkHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointIkHinge();

	D_NEWTON_API virtual bool IsIk() const;
	D_COLLISION_API virtual void SetIkSolver();
	D_COLLISION_API virtual void ResetIkSolver();
	D_COLLISION_API virtual void StopIkMotor(ndFloat32 timestep);
	D_COLLISION_API virtual bool SetIkMotor(ndFloat32 timestep, const ndJacobian& forceBody0, const ndJacobian& forceBody1);
	
	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	
	ndFloat32 m_minTorque;
	ndFloat32 m_maxTorque;
	ndFloat32 m_motorAccel;
	ndFloat32 m_savedMinToque;
	ndFloat32 m_savedMaxTorque;
};

#endif 

