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

#ifndef __ND_JOINT_PD_BALL_AND_SOCKET_H__
#define __ND_JOINT_PD_BALL_AND_SOCKET_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointPdBallAndSocket : public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointPdBallAndSocket);
	D_NEWTON_API ndJointPdBallAndSocket(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointPdBallAndSocket(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointPdBallAndSocket();

	D_NEWTON_API void SetTwistLimits(ndFloat32 minAngle, ndFloat32 maxAngle);
	D_NEWTON_API void GetTwistLimits(ndFloat32& minAngle, ndFloat32& maxAngle) const;
	D_NEWTON_API void GetTwistAngleSpringDamperRegularizer(ndFloat32& spring, ndFloat32& damper, ndFloat32& regularizer) const;
	D_NEWTON_API void SetTwistAngleSpringDamperRegularizer(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer = ndFloat32(5.0e-3f));

	D_NEWTON_API ndFloat32 GetMaxConeAngle() const;
	D_NEWTON_API void SetConeLimit(ndFloat32 maxConeAngle);
	D_NEWTON_API void GetConeAngleSpringDamperRegularizer(ndFloat32& spring, ndFloat32& damper, ndFloat32& regularizer) const;
	D_NEWTON_API void SetConeAngleSpringDamperRegularizer(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer = ndFloat32(5.0e-3f));

	D_NEWTON_API void GetLinearSpringDamperRegularizer(ndFloat32& spring, ndFloat32& damper, ndFloat32& regularizer) const;
	D_NEWTON_API void SetLinearSpringDamperRegularizer(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer = ndFloat32(5.0e-3f));

	D_NEWTON_API ndVector GetTargetPosition() const;
	D_NEWTON_API void SetTargetPosition(const ndVector& posit);

	D_NEWTON_API ndMatrix GetTargetMatrix() const;
	D_NEWTON_API void SetTargetMatrix(const ndMatrix& posit);

	const ndMatrix& GetReferenceMatrix() const;

	ndMatrix GetTargetRotation() const;
	void SetTargetRotation(const ndMatrix& rotation);

	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;


	//void SubmitTwistLimits(const dVector& pin, ndFloat32 angle, ndConstraintDescritor& desc);
	//void SubmitPdRotation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	void SubmitLinearLimits(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	void SubmitConeAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitTwistAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	ndMatrix m_pivotFrame;
	
	ndFloat32 m_minTwistAngle;
	ndFloat32 m_maxTwistAngle;
	ndFloat32 m_twistAngleSpring;
	ndFloat32 m_twistAngleDamper;
	ndFloat32 m_twistAngleRegularizer;

	ndFloat32 m_maxConeAngle;
	ndFloat32 m_coneAngleSpring;
	ndFloat32 m_coneAngleDamper;
	ndFloat32 m_coneAngleRegularizer;

	ndFloat32 m_linearSpring;
	ndFloat32 m_linearDamper;
	ndFloat32 m_linearRegularizer;
};

inline ndMatrix ndJointPdBallAndSocket::GetTargetRotation() const
{
	ndMatrix tmp(m_localMatrix1);
	tmp.m_posit = m_pivotFrame.m_posit;
	return tmp;
}

inline void ndJointPdBallAndSocket::SetTargetRotation(const ndMatrix& matrix)
{
	ndMatrix tmp(matrix);
	tmp.m_posit = m_localMatrix1.m_posit;
	m_localMatrix1 = tmp;
}

inline const ndMatrix& ndJointPdBallAndSocket::GetReferenceMatrix() const
{
	return m_pivotFrame;
}

#endif 

