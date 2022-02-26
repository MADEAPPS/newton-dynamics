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

#ifndef __ND_JOINT_DOUBLE_HINGE_H__
#define __ND_JOINT_DOUBLE_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndJointDoubleHinge: public ndJointBilateralConstraint
{
	public:
	class ndAxisParam
	{
		public:
		ndAxisParam();
		void Load(const nd::TiXmlNode* const xmlNode);
		void Save(const nd::TiXmlNode* const xmlNode) const;

		ndFloat32 m_angle;
		ndFloat32 m_omega;
		ndFloat32 m_springK;
		ndFloat32 m_damperC;
		ndFloat32 m_minLimit;
		ndFloat32 m_maxLimit;
		ndFloat32 m_offsetAngle;
		ndFloat32 m_springDamperRegularizer;
	};

	D_CLASS_REFLECTION(ndJointDoubleHinge);
	D_NEWTON_API ndJointDoubleHinge(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointDoubleHinge();

	D_NEWTON_API ndFloat32 GetAngle0() const;
	D_NEWTON_API ndFloat32 GetOmega0() const;

	D_NEWTON_API void SetLimits0(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits0(ndFloat32& minLimit, ndFloat32& maxLimit);

	D_NEWTON_API ndFloat32 GetOffsetAngle0() const;
	D_NEWTON_API void SetOffsetAngle0(ndFloat32 angle);
	D_NEWTON_API void SetAsSpringDamper0(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	ndAxisParam m_axis0;
	ndAxisParam m_axis1;
};

#endif 

