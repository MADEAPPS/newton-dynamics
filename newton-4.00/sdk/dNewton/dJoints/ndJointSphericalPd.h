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

#ifndef __ND_JOINT_SPHERICAL_PD_H__
#define __ND_JOINT_SPHERICAL_PD_H__

#include "ndNewtonStdafx.h"
#include "ndJointSpherical.h"

class ndJointSphericalPd : public ndJointSpherical
{
	public:
	D_CLASS_REFLECTION(ndJointSphericalPd);
	D_NEWTON_API ndJointSphericalPd(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointSphericalPd(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointSphericalPd();

	//D_NEWTON_API void SetTwistLimits(ndFloat32 minAngle, ndFloat32 maxAngle);
	//D_NEWTON_API void GetTwistLimits(ndFloat32& minAngle, ndFloat32& maxAngle) const;
	D_NEWTON_API void SetTwistSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetTwistSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;
	
	//D_NEWTON_API ndFloat32 GetMaxConeAngle() const;
	//D_NEWTON_API void SetConeLimit(ndFloat32 maxConeAngle);
	D_NEWTON_API void SetConeSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetConeSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;
	

	//D_NEWTON_API ndVector GetTargetPosition() const;
	//D_NEWTON_API void SetTargetPosition(const ndVector& posit);
	//
	//D_NEWTON_API ndMatrix GetTargetMatrix() const;
	//D_NEWTON_API void SetTargetMatrix(const ndMatrix& posit);
	//
	//const ndMatrix& GetReferenceMatrix() const;
	//
	//ndMatrix GetTargetRotation() const;
	//void SetTargetRotation(const ndMatrix& rotation);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	void SubmitTwistAngle(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc);
	void SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	
	//void SubmitConeAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	//void SubmitTwistAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	//ndMatrix m_pivotFrame;
	//ndFloat32 m_minTwistAngle;
	//ndFloat32 m_maxTwistAngle;
	ndFloat32 m_twistAngleSpring;
	ndFloat32 m_twistAngleDamper;
	ndFloat32 m_twistAngleRegularizer;
	
	//ndFloat32 m_maxConeAngle;
	ndFloat32 m_coneAngleSpring;
	ndFloat32 m_coneAngleDamper;
	ndFloat32 m_coneAngleRegularizer;
};

//inline ndMatrix ndJointSphericalPd::GetTargetRotation() const
//{
//	ndMatrix tmp(m_localMatrix1);
//	tmp.m_posit = m_pivotFrame.m_posit;
//	return tmp;
//}
//
//inline void ndJointSphericalPd::SetTargetRotation(const ndMatrix& matrix)
//{
//	ndMatrix tmp(matrix);
//	tmp.m_posit = m_localMatrix1.m_posit;
//	m_localMatrix1 = tmp;
//}
//
//inline const ndMatrix& ndJointSphericalPd::GetReferenceMatrix() const
//{
//	return m_pivotFrame;
//}

#endif 

