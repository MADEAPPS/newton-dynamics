/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_BODY_H_
#define __ND_BODY_H_

#include "ndCollisionStdafx.h"
#include "ndShapeInstance.h"

class ndContact;
class ndBodyNotify;
class ndBodyDynamic;
class ndBodySentinel;
class ndBodySphFluid;
class ndBodyKinematic;
class ndRayCastNotify;
class ndBodyParticleSet;
class ndBodyTriggerVolume;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndBody : public ndContainersFreeListAlloc<ndBody>
{
	public:
	D_CLASS_REFLECTION(ndBody);
	D_COLLISION_API ndBody();
	D_COLLISION_API ndBody(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_COLLISION_API virtual ~ndBody();

	virtual ndBody* GetAsBody() { return this;}
	virtual ndBodyDynamic* GetAsBodyDynamic() { return nullptr; }
	virtual ndBodySentinel* GetAsBodySentinel() { return nullptr; }
	virtual ndBodySphFluid* GetAsBodySphFluid() { return nullptr; }
	virtual ndBodyKinematic* GetAsBodyKinematic() { return nullptr; }
	virtual ndBodyKinematic* GetAsBodyPlayerCapsule() { return nullptr; }
	virtual ndBodyParticleSet* GetAsBodyParticleSet() { return nullptr; }
	virtual ndBodyTriggerVolume* GetAsBodyTriggerVolume() { return nullptr; }

	ndUnsigned32 GetId() const;
	void GetAABB(ndVector& p0, ndVector& p1) const;

	virtual ndFloat32 GetInvMass() const;
	virtual bool RayCast(ndRayCastNotify& callback, const ndFastRay& ray, const ndFloat32 maxT) const = 0;

	const ndVector& GetCentreOfMass() const;
	D_COLLISION_API virtual void SetCentreOfMass(const ndVector& com);

	ndBodyNotify* GetNotifyCallback() const;

	
	ndVector GetOmega() const;
	ndMatrix GetMatrix() const;
	ndVector GetVelocity() const;
	ndVector GetPosition() const;
	ndQuaternion GetRotation() const;
	ndVector GetGlobalGetCentreOfMass() const;

	D_COLLISION_API virtual void SetNotifyCallback(ndBodyNotify* const notify);
	D_COLLISION_API virtual void SetOmega(const ndVector& veloc);
	D_COLLISION_API virtual void SetVelocity(const ndVector& veloc);
	D_COLLISION_API virtual void SetMatrix(const ndMatrix& matrix);
	
	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	D_COLLISION_API ndVector GetVelocityAtPoint(const ndVector& point) const;

	void SetMatrixAndCentreOfMass(const ndQuaternion& rotation, const ndVector& globalcom);

	protected:
	D_COLLISION_API static const nd::TiXmlNode* FindNode(const nd::TiXmlNode* const rootNode, const char* const name);

	virtual void AttachContact(ndContact* const) {}
	virtual void DetachContact(ndContact* const) {}
	virtual ndContact* FindContact(const ndBody* const) const { return nullptr; }

	ndMatrix m_matrix;
	ndVector m_veloc;
	ndVector m_omega;
	ndVector m_localCentreOfMass;
	ndVector m_globalCentreOfMass;
	ndVector m_minAabb;
	ndVector m_maxAabb;
	ndQuaternion m_rotation;
	ndBodyNotify* m_notifyCallback;

	ndUnsigned32 m_uniqueId;
	union
	{
		ndUnsigned32 m_flags;
		struct
		{
			ndUnsigned32 m_isDynamics : 1;
			ndUnsigned32 m_skeletonMark : 1;
			ndUnsigned32 m_skeletonMark0 : 1;
			ndUnsigned32 m_skeletonMark1 : 1;
			ndUnsigned32 m_contactTestOnly : 1;
			ndUnsigned32 m_transformIsDirty : 1;
			ndUnsigned32 m_equilibriumOverride : 1;
		};
	};

	ndUnsigned8 m_isStatic;
	ndUnsigned8 m_autoSleep;
	ndUnsigned8 m_islandSleep;
	ndUnsigned8 m_equilibrium;
	ndUnsigned8 m_equilibrium0;
	ndUnsigned8 m_isJointFence0;
	ndUnsigned8 m_isJointFence1;
	ndUnsigned8 m_bodyIsConstrained;
	D_COLLISION_API static ndUnsigned32 m_uniqueIdCount;

	friend class ndWorld;
	friend class ndScene;
	friend class ndConstraint;
	friend class ndBodyPlayerCapsuleImpulseSolver;
} D_GCC_NEWTON_ALIGN_32;

inline ndUnsigned32 ndBody::GetId() const
{
	return m_uniqueId;
}

inline ndBodyNotify* ndBody::GetNotifyCallback() const
{
	return m_notifyCallback;
}

inline ndMatrix ndBody::GetMatrix() const
{
	return m_matrix;
}

inline ndVector ndBody::GetPosition() const
{
	return m_matrix.m_posit;
}

inline ndQuaternion ndBody::GetRotation() const
{
	return m_rotation;
}

inline ndVector ndBody::GetGlobalGetCentreOfMass() const
{
	return m_globalCentreOfMass;
}

inline ndVector ndBody::GetVelocity() const
{
	return m_veloc;
}

inline ndVector ndBody::GetOmega() const
{
	return m_omega;
}

inline void ndBody::GetAABB(ndVector& p0, ndVector& p1) const
{
	p0 = m_minAabb;
	p1 = m_maxAabb;
}

inline const ndVector& ndBody::GetCentreOfMass() const
{
	return m_localCentreOfMass;
}

inline ndVector ndBody::GetVelocityAtPoint(const ndVector& point) const
{
	return m_veloc + m_omega.CrossProduct(point - m_globalCentreOfMass);
}

inline ndFloat32 ndBody::GetInvMass() const 
{ 
	return ndFloat32(0.0f); 
}

inline void ndBody::SetMatrixAndCentreOfMass(const ndQuaternion& rotation, const ndVector& globalcom)
{
	m_rotation = rotation;
	m_globalCentreOfMass = globalcom;
	m_matrix = ndMatrix(rotation, m_matrix.m_posit);
	m_matrix.m_posit = m_globalCentreOfMass - m_matrix.RotateVector(m_localCentreOfMass);
}

#endif 

