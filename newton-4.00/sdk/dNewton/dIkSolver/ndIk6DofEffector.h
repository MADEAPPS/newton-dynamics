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

#ifndef __ND_IK_6DOF_EFFECTOR_H__
#define __ND_IK_6DOF_EFFECTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndIk6DofEffector: public ndJointBilateralConstraint
{
	public:
	enum ndRotationType
	{
		m_disabled,
		m_fixAxis,
		m_shortestPath,
	};

	D_CLASS_REFLECTION(ndIk6DofEffector, ndJointBilateralConstraint)

	D_NEWTON_API ndIk6DofEffector();
	D_NEWTON_API ndIk6DofEffector(const ndMatrix& pinAndPivotChild, const ndMatrix& pinAndPivotParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndIk6DofEffector();

	D_NEWTON_API void EnableAxisX(bool state);
	D_NEWTON_API void EnableAxisY(bool state);
	D_NEWTON_API void EnableAxisZ(bool state);
	D_NEWTON_API void EnableRotationAxis(ndRotationType type);

	D_NEWTON_API bool GetAxisX() const;
	D_NEWTON_API bool GetAxisY() const;
	D_NEWTON_API bool GetAxisZ() const;
	D_NEWTON_API ndRotationType GetRotationAxis() const;

	D_NEWTON_API ndMatrix GetOffsetMatrix() const;
	D_NEWTON_API ndMatrix GetEffectorMatrix() const;
	D_NEWTON_API void SetOffsetMatrix(const ndMatrix& matrix);
	
	D_NEWTON_API void SetLinearSpringDamper(ndFloat32 regularizer, ndFloat32 springConst, ndFloat32 damperConst);
	D_NEWTON_API void GetLinearSpringDamper(ndFloat32& regularizer, ndFloat32& springConst, ndFloat32& damperConst) const;
	
	D_NEWTON_API void SetAngularSpringDamper(ndFloat32 regularizer, ndFloat32 springConst, ndFloat32 damperConst);
	D_NEWTON_API void GetAngularSpringDamper(ndFloat32& regularizer, ndFloat32& springConst, ndFloat32& damperConst) const;

	D_NEWTON_API ndFloat32 GetMaxForce() const;
	D_NEWTON_API void SetMaxForce(ndFloat32 force);
	
	D_NEWTON_API ndFloat32 GetMaxTorque() const;
	D_NEWTON_API void SetMaxTorque(ndFloat32 torque);
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	D_NEWTON_API ndMatrix CalculateGlobalMatrix1() const;
	D_NEWTON_API ndMatrix CalculateGlobalBaseMatrix1() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void SubmitLinearAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	D_NEWTON_API void SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	D_NEWTON_API void SubmitShortestPathAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	
	ndMatrix m_targetFrame;
	ndFloat32 m_angularSpring;
	ndFloat32 m_angularDamper;
	ndFloat32 m_angularMaxTorque;
	ndFloat32 m_angularRegularizer;
	
	ndFloat32 m_linearSpring;
	ndFloat32 m_linearDamper;
	ndFloat32 m_linearMaxForce;
	ndFloat32 m_linearRegularizer;

	ndRotationType m_rotationType;
	union
	{
		ndUnsigned8 m_controlDofOptions;
		struct
		{
			ndUnsigned8 m_axisX	: 1;
			ndUnsigned8 m_axisY	: 1;
			ndUnsigned8 m_axisZ	: 1;
		};
	};
};

#endif 

