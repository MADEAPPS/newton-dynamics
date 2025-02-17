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

#ifndef __ND_JOINT_DOUBLE_HINGE_H__
#define __ND_JOINT_DOUBLE_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_MAX_DOUBLE_HINGE_RECOVERY_SPEED	ndFloat32 (0.25f)
#define D_MAX_DOUBLE_HINGE_PENETRATION		(ndFloat32 (4.0f) * ndDegreeToRad)

D_MSV_NEWTON_ALIGN_32
class ndJointDoubleHinge: public ndJointBilateralConstraint
{
	public:
	class ndAxisParam
	{
		public:
		ndAxisParam();
		ndFloat32 m_angle;
		ndFloat32 m_omega;
		ndFloat32 m_springK;
		ndFloat32 m_damperC;
		ndFloat32 m_minLimit;
		ndFloat32 m_maxLimit;
		ndFloat32 m_offsetAngle;
		ndFloat32 m_springDamperRegularizer;
	};

	D_CLASS_REFLECTION(ndJointDoubleHinge, ndJointBilateralConstraint)

	D_NEWTON_API ndJointDoubleHinge();
	D_NEWTON_API ndJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointDoubleHinge();

	D_NEWTON_API ndFloat32 GetAngle0() const;
	D_NEWTON_API ndFloat32 GetOmega0() const;
	D_NEWTON_API ndFloat32 GetOffsetAngle0() const;
	D_NEWTON_API void SetOffsetAngle0(ndFloat32 angle);
	D_NEWTON_API void SetLimits0(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits0(ndFloat32& minLimit, ndFloat32& maxLimit);
	D_NEWTON_API void SetAsSpringDamper0(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamper0(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	D_NEWTON_API ndFloat32 GetAngle1() const;
	D_NEWTON_API ndFloat32 GetOmega1() const;
	D_NEWTON_API ndFloat32 GetOffsetAngle1() const;
	D_NEWTON_API void SetOffsetAngle1(ndFloat32 angle);
	D_NEWTON_API void SetLimits1(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits1(ndFloat32& minLimit, ndFloat32& maxLimit);
	D_NEWTON_API void SetAsSpringDamper1(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamper1(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	protected:
	D_NEWTON_API void ClearMemory();
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API ndFloat32 PenetrationOmega(ndFloat32 penetartion) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API void SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamper0(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamper1(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	ndAxisParam m_axis0;
	ndAxisParam m_axis1;
	D_MEMORY_ALIGN_FIXUP
} D_GCC_NEWTON_ALIGN_32;


#endif 

