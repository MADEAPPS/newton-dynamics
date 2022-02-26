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

#ifndef __ND_JOINT_HINGE_H__
#define __ND_JOINT_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_MAX_HINGE_RECOVERY_SPEED	ndFloat32 (0.25f)
#define D_MAX_HINGE_PENETRATION		(ndFloat32 (4.0f) * ndDegreeToRad)

class ndJointHinge: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointHinge);
	D_NEWTON_API ndJointHinge(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointHinge();

	D_NEWTON_API ndFloat32 GetAngle() const;
	D_NEWTON_API ndFloat32 GetOmega() const;
	D_NEWTON_API ndJacobianPair GetPinJacobian() const;

	D_NEWTON_API void SetLimits(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit);

	D_NEWTON_API ndFloat32 GetOffsetAngle() const;
	D_NEWTON_API void SetOffsetAngle(ndFloat32 angle);
	D_NEWTON_API void SetAsSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);

	protected:
	D_NEWTON_API ndFloat32 PenetrationOmega(ndFloat32 penetartion) const;
	D_NEWTON_API void SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API bool SubmitConstraintLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	ndFloat32 m_angle;
	ndFloat32 m_omega;
	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	ndFloat32 m_offsetAngle;
	ndFloat32 m_springDamperRegularizer;
};

#endif 

