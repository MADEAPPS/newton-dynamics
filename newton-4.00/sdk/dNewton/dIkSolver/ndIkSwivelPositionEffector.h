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

#ifndef __ND_IK_SWIVEL_POSITION_EFFECTOR_H__
#define __ND_IK_SWIVEL_POSITION_EFFECTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndIkSwivelPositionEffector: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndIkSwivelPositionEffector, ndJointBilateralConstraint)

	D_NEWTON_API ndIkSwivelPositionEffector();
	D_NEWTON_API ndIkSwivelPositionEffector(
		const ndMatrix& pinAndPivotParentInGlobalSpace, ndBodyKinematic* const parent,
		const ndVector& childPivotInGlobalSpace, ndBodyKinematic* const child);
	D_NEWTON_API virtual ~ndIkSwivelPositionEffector();

	D_NEWTON_API ndVector GetLocalTargetPosition() const;
	D_NEWTON_API void SetLocalTargetPosition(const ndVector& posit);
	
	D_NEWTON_API ndVector GetEffectorPosit() const;
	//D_NEWTON_API ndVector GetGlobalPosition() const;
	
	D_NEWTON_API ndVector GetRestPosit() const;
	D_NEWTON_API void SetRestPosit(const ndVector& posit);
	
	D_NEWTON_API ndFloat32 GetSwivelAngle() const;
	D_NEWTON_API void SetSwivelAngle(ndFloat32 angle);
	D_NEWTON_API ndFloat32 CalculateLookAtSwivelAngle(const ndVector& lookAtDir) const;
	
	D_NEWTON_API void SetLinearSpringDamper(ndFloat32 regularizer, ndFloat32 springConst, ndFloat32 damperConst);
	D_NEWTON_API void GetLinearSpringDamper(ndFloat32& regularizer, ndFloat32& springConst, ndFloat32& damperConst) const;
	
	D_NEWTON_API void SetAngularSpringDamper(ndFloat32 regularizer, ndFloat32 springConst, ndFloat32 damperConst);
	D_NEWTON_API void GetAngularSpringDamper(ndFloat32& regularizer, ndFloat32& springConst, ndFloat32& damperConst) const;
	
	D_NEWTON_API void SetWorkSpaceConstraints(ndFloat32 minRadio, ndFloat32 maxRadio);
	D_NEWTON_API void GetWorkSpaceConstraints(ndFloat32& minRadio, ndFloat32& maxRadio) const;
	
	D_NEWTON_API ndFloat32 GetMaxForce() const;
	D_NEWTON_API void SetMaxForce(ndFloat32 force);

	D_NEWTON_API ndFloat32 GetMaxTorque() const;
	D_NEWTON_API void SetMaxTorque(ndFloat32 torque);

	D_NEWTON_API bool GetSwivelMode() const;
	D_NEWTON_API void SetSwivelMode(bool active);

	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
	D_NEWTON_API ndInt32 GetKinematicState(ndKinematicState* const state) const;
	
	D_NEWTON_API void ClearMemory();

	protected:
	ndMatrix CalculateSwivelFrame(const ndMatrix& matrix1) const;
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	void SubmitLinearAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	void SubmitAngularAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	
	ndVector m_restPosition;
	ndVector m_localTargetPosit;
	ndFloat32 m_swivelAngle;

	ndFloat32 m_angularSpring;
	ndFloat32 m_angularDamper;
	ndFloat32 m_angularMaxTorque;
	ndFloat32 m_angularRegularizer;
	
	ndFloat32 m_linearSpring;
	ndFloat32 m_linearDamper;
	ndFloat32 m_linearMaxForce;
	ndFloat32 m_linearRegularizer;

	ndFloat32 m_minWorkSpaceRadio;
	ndFloat32 m_maxWorkSpaceRadio;
	bool m_enableSwivelControl;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 

