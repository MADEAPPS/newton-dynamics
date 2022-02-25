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

#ifndef __ND_JOINT_HINGE_PD_H__
#define __ND_JOINT_HINGE_PD_H__

#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

class ndJointHingePd: public ndJointHinge
{
	public:
	D_CLASS_REFLECTION(ndJointHingePd);
	D_NEWTON_API ndJointHingePd(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointHingePd(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointHingePd();

	D_NEWTON_API virtual ndFloat32 GetTarget() const;
	D_NEWTON_API virtual void SetTarget(ndFloat32 angle);
	
	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	
	ndFloat32 m_targetAngle;
};

#endif
