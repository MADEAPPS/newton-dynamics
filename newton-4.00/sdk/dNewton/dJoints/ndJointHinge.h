/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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

D_MSV_NEWTON_ALIGN_32
class ndJointHinge: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointHinge, ndJointBilateralConstraint)

	D_NEWTON_API ndJointHinge();
	D_NEWTON_API ndJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointHinge();

	D_NEWTON_API ndFloat32 GetAngle() const;
	D_NEWTON_API ndFloat32 GetOmega() const;
	D_NEWTON_API ndFloat32 GetTargetAngle() const;
	D_NEWTON_API void SetTargetAngle(ndFloat32 angle);
	D_NEWTON_API bool GetLimitState() const;
	D_NEWTON_API void SetLimitState(bool state);
	D_NEWTON_API void SetLimits(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API void SetAsSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	protected:
	D_NEWTON_API void ClearMemory();
	D_NEWTON_API ndFloat32 PenetrationOmega(ndFloat32 penetartion) const;
	D_NEWTON_API void SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API ndInt32 GetKinematicState(ndKinematicState* const state) const;
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	ndFloat32 m_angle;
	ndFloat32 m_omega;
	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	ndFloat32 m_targetAngle;
	ndFloat32 m_springDamperRegularizer;
	ndInt8 m_limitState;
	D_MEMORY_ALIGN_FIXUP
} D_GCC_NEWTON_ALIGN_32;

#endif 

