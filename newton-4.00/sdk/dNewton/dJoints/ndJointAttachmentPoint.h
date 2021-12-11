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

#ifndef __ND_JOINT_ATTACHEMENT_POINT_H_
#define __ND_JOINT_ATTACHEMENT_POINT_H_

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

// this joint is usefully to simulate the rolling friction of a rolling ball over 
// a flat surface.
// normally this is not important for non spherical objects, but for games like 
// poll, pinball, bolling, golf or any other where the movement of balls is the main objective
// the rolling friction is a real big problem.
class ndJointAttachmentPoint: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointAttachmentPoint);
	D_NEWTON_API ndJointAttachmentPoint(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API ndJointAttachmentPoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	D_NEWTON_API virtual ~ndJointAttachmentPoint();

	void SetDimnetionX(bool lock);
	void SetDimnetionY(bool lock);
	void SetDimnetionZ(bool lock);

	bool GetDimnetionX() const;
	bool GetDimnetionY() const;
	bool GetDimnetionZ() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	union ndLockedDOF
	{
		dInt32 m_lockDof;
		struct
		{
			dInt32 m_lock_x : 1;
			dInt32 m_lock_y : 1;
			dInt32 m_lock_z : 1;
		};
	};
	
	ndLockedDOF m_lockedDimnetions;
};

inline void ndJointAttachmentPoint::SetDimnetionX(bool lock)
{
	m_lockedDimnetions.m_lock_x = lock ? 1 : 0;
}

inline void ndJointAttachmentPoint::SetDimnetionY(bool lock)
{
	m_lockedDimnetions.m_lock_y = lock ? 1 : 0;
}

inline void ndJointAttachmentPoint::SetDimnetionZ(bool lock)
{
	m_lockedDimnetions.m_lock_z = lock ? 1 : 0;
}

inline bool ndJointAttachmentPoint::GetDimnetionX() const
{
	return m_lockedDimnetions.m_lock_x ? true : false;
}

inline bool ndJointAttachmentPoint::GetDimnetionY() const
{
	return m_lockedDimnetions.m_lock_y ? true : false;
}

inline bool ndJointAttachmentPoint::GetDimnetionZ() const
{
	return m_lockedDimnetions.m_lock_z ? true : false;
}



#endif 

