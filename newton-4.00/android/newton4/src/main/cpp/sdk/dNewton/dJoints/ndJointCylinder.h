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

#ifndef __ND_JOINT_CYLINDER_H__
#define __ND_JOINT_CYLINDER_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointCylinder: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointCylinder);
	D_NEWTON_API ndJointCylinder(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointCylinder(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointCylinder(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointCylinder();

	D_NEWTON_API ndFloat32 GetAngle() const;
	D_NEWTON_API ndFloat32 GetOmega() const;
	D_NEWTON_API ndFloat32 GetOffsetAngle() const;
	D_NEWTON_API void SetOffsetAngle(ndFloat32 angle);
	D_NEWTON_API bool GetLimitStateAngle() const;
	D_NEWTON_API void SetLimitStateAngle(bool state);
	D_NEWTON_API void SetLimitsAngle(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimitsAngle(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API void SetAsSpringDamperAngle(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamperAngle(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	D_NEWTON_API ndFloat32 GetPosit() const;
	D_NEWTON_API ndFloat32 GetOffsetPosit() const;
	D_NEWTON_API void SetOffsetPosit(ndFloat32 offset);
	D_NEWTON_API bool GetLimitStatePosit() const;
	D_NEWTON_API void SetLimitStatePosit(bool state);
	D_NEWTON_API void SetLimitsPosit(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimitsPosit(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API void SetAsSpringDamperPosit(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamperPosit(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	protected:
	D_NEWTON_API ndFloat32 PenetrationOmega(ndFloat32 penetartion) const;
	D_NEWTON_API void SubmitLimitsAngle(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamperAngle(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API ndFloat32 PenetrationSpeed(ndFloat32 penetration) const;
	D_NEWTON_API void SubmitLimitsPosit(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamperPosit(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	ndFloat32 m_angle;
	ndFloat32 m_omega;
	ndFloat32 m_springKAngle;
	ndFloat32 m_damperCAngle;
	ndFloat32 m_minLimitAngle;
	ndFloat32 m_maxLimitAngle;
	ndFloat32 m_offsetAngle;
	ndFloat32 m_springDamperRegularizerAngle;

	ndFloat32 m_posit;
	ndFloat32 m_speed;
	ndFloat32 m_springKPosit;
	ndFloat32 m_damperCPosit;
	ndFloat32 m_minLimitPosit;
	ndFloat32 m_maxLimitPosit;
	ndFloat32 m_offsetPosit;
	ndFloat32 m_springDamperRegularizerPosit;

	ndInt8 m_limitStatePosit;
	ndInt8 m_limitStateAngle;
};

#endif 

