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

	D_NEWTON_API ndMatrix GetTargetMatrix() const;
	D_NEWTON_API void SetTargetMatrix(const ndMatrix& matrix);
	D_NEWTON_API void SetSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	void SubmitTwistAngle(const ndVector& pin, ndFloat32 angle, ndConstraintDescritor& desc);
	void SubmitAngularAxis(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	void SubmitAngularAxisCartesianApproximation(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	
	//void SubmitConeAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);
	//void SubmitTwistAngleOnlyRows(const ndMatrix& matrix0, const ndMatrix& matrix1, ndConstraintDescritor& desc);

	ndMatrix m_pivotFrame;
	ndFloat32 m_springConst;
};


#endif 

