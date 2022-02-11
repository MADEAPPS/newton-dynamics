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

	//D_NEWTON_API virtual ndFloat32 GetAngle() const;
	//D_NEWTON_API virtual ndFloat32 GetOmega() const;
	//D_NEWTON_API virtual ndFloat32 GetFriction() const;
	//
	//D_NEWTON_API virtual void SetFriction(ndFloat32 frictionTorque);
	//D_NEWTON_API virtual void EnableMotorAccel(bool state, ndFloat32 motorAccel);
	//D_NEWTON_API virtual void EnableLimits(bool state, ndFloat32 minLimit, ndFloat32 maxLimit);
	//D_NEWTON_API virtual void SetAsSpringDamper(bool state, ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	//
	//D_NEWTON_API bool IsMotor() const;
	//D_NEWTON_API ndJacobianPair GetPinJacobian() const;
	//D_NEWTON_API void GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit);
	//
	//private:
	//void SubmitConstraintLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	//void SubmitConstraintLimitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	
	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	
	//ndFloat32 m_angle;
	//ndFloat32 m_omega;
	//ndFloat32 m_springK;
	//ndFloat32 m_damperC;
	//ndFloat32 m_minLimit;
	//ndFloat32 m_maxLimit;
	//ndFloat32 m_friction;
	//ndFloat32 m_motorAccel;
	//ndFloat32 m_springDamperRegularizer;
	//
	//bool m_isMotor;
	//bool m_hasLimits;
	//bool m_isSpringDamper;
};

#endif 

