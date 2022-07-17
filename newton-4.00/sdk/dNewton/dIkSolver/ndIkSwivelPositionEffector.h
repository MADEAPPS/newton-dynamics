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

#ifndef __ND_IK_SWIVEL_POSITION_EFFECTOR_H__
#define __ND_IK_SWIVEL_POSITION_EFFECTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndIkSwivelPositionEffector: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndIkSwivelPositionEffector);
	D_NEWTON_API ndIkSwivelPositionEffector(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndIkSwivelPositionEffector(const ndMatrix& pinAndPivotChild, const ndMatrix& pinAndPivotParent, const ndMatrix& swivelAngleParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndIkSwivelPositionEffector();

	D_NEWTON_API ndVector GetPosition() const;
	D_NEWTON_API void SetPosition(const ndVector& posit);

	D_NEWTON_API ndFloat32 GetSwivelAngle() const;
	D_NEWTON_API void SetSwivelAngle(const ndFloat32 angle);
	
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
	D_NEWTON_API void CalculateSwivelMatrices(ndMatrix& swivelMatrix0, ndMatrix& swivelMatrix1) const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	D_NEWTON_API void SubmitLinearAxis(ndConstraintDescritor& desc);
	D_NEWTON_API void SubmitAngularAxis(ndConstraintDescritor& desc);
	
	ndMatrix m_targetFrame;
	ndMatrix m_localSwivelMatrix0;
	ndMatrix m_localSwivelMatrix1;
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
};

#endif 

