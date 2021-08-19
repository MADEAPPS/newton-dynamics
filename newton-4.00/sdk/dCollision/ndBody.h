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

#ifndef _D_BODY_H_
#define _D_BODY_H_

#include "ndCollisionStdafx.h"
#include "ndSetData.h"
#include "ndShapeInstance.h"

class ndContact;
class ndBodyNotify;
class ndBodyDynamic;
class ndBodyKinematic;
class ndRayCastNotify;
class ndBodySphFluid;
class ndBodyParticleSet;
class ndBodyTriggerVolume;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndBody: public dClassAlloc
{
	public:
	D_CLASS_REFLECTION(ndBody);
	D_COLLISION_API ndBody();
	D_COLLISION_API ndBody(const dClassLoaderBase::dDesc& desc);
	D_COLLISION_API virtual ~ndBody();

	virtual ndBody* GetAsBody() { return this;}
	virtual ndBodyDynamic* GetAsBodyDynamic() { return nullptr; }
	virtual ndBodyKinematic* GetAsBodyKinematic() { return nullptr; }
	virtual ndBodyKinematic* GetAsBodyPlayerCapsule() { return nullptr; }
	virtual ndBodySphFluid* GetAsBodySphFluid() { return nullptr; }
	virtual ndBodyParticleSet* GetAsBodyParticleSet() { return nullptr; }
	virtual ndBodyTriggerVolume* GetAsBodyTriggerVolume() { return nullptr; }

	dUnsigned32 GetId() const;
	void GetAABB(dVector& p0, dVector& p1) const;

	virtual dFloat32 GetInvMass() const;
	virtual bool RayCast(ndRayCastNotify& callback, const dFastRay& ray, const dFloat32 maxT) const = 0;

	const dVector& GetCentreOfMass() const;
	D_COLLISION_API void SetCentreOfMass(const dVector& com);

	ndBodyNotify* GetNotifyCallback() const;
	D_COLLISION_API void SetNotifyCallback(ndBodyNotify* const notify);
	D_COLLISION_API dVector GetOmega() const;
	D_COLLISION_API void SetOmega(const dVector& veloc);
	D_COLLISION_API dVector GetVelocity() const;
	D_COLLISION_API void SetVelocity(const dVector& veloc);
	D_COLLISION_API dMatrix GetMatrix() const;
	D_COLLISION_API void SetMatrix(const dMatrix& matrix);
	D_COLLISION_API dQuaternion GetRotation() const;
	D_COLLISION_API virtual void Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 shapeHash, dInt32 nodeHash) const;

	D_COLLISION_API dVector GetVelocityAtPoint(const dVector& point) const;

	protected:
	D_COLLISION_API static const nd::TiXmlNode* FindNode(const nd::TiXmlNode* const rootNode, const char* const name);

	virtual void AttachContact(ndContact* const) {}
	virtual void DetachContact(ndContact* const) {}
	virtual ndContact* FindContact(const ndBody* const) const { return nullptr; }

	dMatrix m_matrix;
	dVector m_veloc;
	dVector m_omega;
	dVector m_localCentreOfMass;
	dVector m_globalCentreOfMass;
	dVector m_minAabb;
	dVector m_maxAabb;
	dQuaternion m_rotation;
	ndBodyNotify* m_notifyCallback;

	union
	{
		dUnsigned32 m_flags;
		struct
		{
			dUnsigned32 m_resting : 1;
			dUnsigned32 m_autoSleep : 1;
			dUnsigned32 m_equilibrium : 1;
			dUnsigned32 m_islandSleep : 1;
			dUnsigned32 m_solverSleep0 : 1;
			dUnsigned32 m_solverSleep1 : 1;
			dUnsigned32 m_skeletonMark : 1;
			dUnsigned32 m_skeletonMark0 : 1;
			dUnsigned32 m_skeletonMark1 : 1;
			dUnsigned32 m_contactTestOnly : 1;
			dUnsigned32 m_transformIsDirty : 1;
			dUnsigned32 m_bodyIsConstrained : 1;
			dUnsigned32 m_equilibriumOverride : 1;
		};
	};

	dUnsigned32 m_uniqueId;
	static dUnsigned32 m_uniqueIdCount;

	friend class ndScene;
	friend class ndConstraint;
	friend class ndBodyPlayerCapsuleImpulseSolver;
} D_GCC_NEWTON_ALIGN_32;


inline dUnsigned32 ndBody::GetId() const
{
	return m_uniqueId;
}

inline ndBodyNotify* ndBody::GetNotifyCallback() const
{
	return m_notifyCallback;
}

inline dVector ndBody::GetVelocity() const
{
	return m_veloc;
}

inline dVector ndBody::GetOmega() const
{
	return m_omega;
}

inline void ndBody::GetAABB(dVector& p0, dVector& p1) const
{
	p0 = m_minAabb;
	p1 = m_maxAabb;
}

inline const dVector& ndBody::GetCentreOfMass() const
{
	return m_localCentreOfMass;
}

inline dVector ndBody::GetVelocityAtPoint(const dVector& point) const
{
	return m_veloc + m_omega.CrossProduct(point - m_globalCentreOfMass);
}

inline dFloat32 ndBody::GetInvMass() const 
{ 
	return dFloat32(0.0f); 
}

#endif 

