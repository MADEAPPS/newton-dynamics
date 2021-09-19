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

#ifndef __D_JOINT_TWO_BODY_IK_H__
#define __D_JOINT_TWO_BODY_IK_H__

#include "ndNewtonStdafx.h"
#include "ndJointInverseDynamicsBase.h"

class ndJointTwoBodyIK: public ndJointInverseDynamicsBase
{
	public:
	D_CLASS_REFLECTION(ndJointTwoBodyIK);
	D_NEWTON_API ndJointTwoBodyIK(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointTwoBodyIK(const dMatrix& basisInGlobalSpace, const dVector& pivotInGlobalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointTwoBodyIK();

	//D_NEWTON_API void SetTwistLimits(dFloat32 minAngle, dFloat32 maxAngle);
	//D_NEWTON_API void GetTwistLimits(dFloat32& minAngle, dFloat32& maxAngle) const;
	//
	//D_NEWTON_API void GetAngularSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	//D_NEWTON_API void SetAngularSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32(5.0e-3f));
	//
	//D_NEWTON_API void GetLinearSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const;
	//D_NEWTON_API void SetLinearSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer = dFloat32(5.0e-3f));

	//D_NEWTON_API dVector GetTargetPosition() const;
	//D_NEWTON_API void SetTargetPosition(const dVector& posit);

	//D_NEWTON_API dMatrix GetTargetMatrix() const;
	//D_NEWTON_API void SetTargetMatrix(const dMatrix& posit);
	//const dMatrix& GetReferenceMatrix() const;
	//dMatrix GetTargetRotation() const;
	//void SetTargetRotation(const dMatrix& rotation);

	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	virtual void SetTargetLocalMatrix(const dMatrix& localMatrix);
	virtual void SetTargetGlobalMatrix(const dMatrix& globalMatrix);

	void SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc);

	dMatrix m_coneRotation;
	dMatrix m_rotationBasis;
	dVector m_pivot;
	dVector m_refPosit;
	dVector m_targetPosit;

	dFloat32 m_angle;
	dFloat32 m_minAngle;
	dFloat32 m_maxAngle;
	dFloat32 m_angularSpring;
	dFloat32 m_angularDamper;
	dFloat32 m_angularRegularizer;

	dFloat32 m_maxDist;
	dFloat32 m_linearSpring;
	dFloat32 m_linearDamper;
	dFloat32 m_linearRegularizer;
};

//inline const dMatrix& ndJointTwoBodyIK::GetReferenceMatrix() const
//{
//	return m_pivotFrame;
//}


#endif 

