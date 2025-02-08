/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
#include "ndBodyListView.h"

class ndContact;
class ndBodyNotify;
class ndBodyDynamic;
class ndBodySentinel;
class ndBodySphFluid;
class ndBodyKinematic;
class ndRayCastNotify;
class ndBodyParticleSet;
class ndBodyTriggerVolume;
class ndBodyPlayerCapsule;
class ndBodyKinematicBase;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndBody : public ndContainersFreeListAlloc<ndBody>
{
	public:
	D_BASE_CLASS_REFLECTION(ndBody)

	D_COLLISION_API ndBody();
	D_COLLISION_API ndBody(const ndBody& src);
	D_COLLISION_API virtual ~ndBody();

	virtual ndBody* GetAsBody() { return this;}
	virtual ndBodyDynamic* GetAsBodyDynamic() { return nullptr; }
	virtual ndBodySentinel* GetAsBodySentinel() { return nullptr; }
	virtual ndBodySphFluid* GetAsBodySphFluid() { return nullptr; }
	virtual ndBodyKinematic* GetAsBodyKinematic() { return nullptr; }
	virtual ndBodyParticleSet* GetAsBodyParticleSet() { return nullptr; }
	virtual ndBodyPlayerCapsule* GetAsBodyPlayerCapsule() { return nullptr; }
	virtual ndBodyTriggerVolume* GetAsBodyTriggerVolume() { return nullptr; }
	virtual ndBodyKinematicBase* GetAsBodyKinematicSpecial() { return nullptr; }

	D_COLLISION_API ndUnsigned32 GetId() const;
	D_COLLISION_API void GetAABB(ndVector& p0, ndVector& p1) const;

	D_COLLISION_API bool GetSeletonSelfCollision() const;
	D_COLLISION_API void SetSeletonSelfCollision(bool state);

	D_COLLISION_API virtual ndFloat32 GetInvMass() const;
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const ndFastRay& ray, const ndFloat32 maxT) const = 0;

	D_COLLISION_API const ndVector& GetCentreOfMass() const;
	D_COLLISION_API virtual void SetCentreOfMass(const ndVector& com);
	
	D_COLLISION_API ndVector GetOmega() const;
	D_COLLISION_API ndMatrix GetMatrix() const;
	D_COLLISION_API ndVector GetVelocity() const;
	D_COLLISION_API ndVector GetPosition() const;
	D_COLLISION_API ndQuaternion GetRotation() const;
	D_COLLISION_API ndVector GetGlobalGetCentreOfMass() const;

	D_COLLISION_API ndBodyNotify* GetNotifyCallback() const;
	D_COLLISION_API virtual void SetNotifyCallback(ndBodyNotify* const notify);
	D_COLLISION_API virtual void SetOmega(const ndVector& veloc);
	D_COLLISION_API virtual void SetVelocity(const ndVector& veloc);
	D_COLLISION_API virtual void SetMatrix(const ndMatrix& matrix);
	D_COLLISION_API ndVector GetVelocityAtPoint(const ndVector& point) const;

	D_COLLISION_API void SetOmegaNoSleep(const ndVector& veloc);
	D_COLLISION_API void SetVelocityNoSleep(const ndVector& veloc);
	D_COLLISION_API void SetMatrixNoSleep(const ndMatrix& matrix);
	D_COLLISION_API void SetMatrixAndCentreOfMass(const ndQuaternion& rotation, const ndVector& globalcom);

	protected:
	virtual void AttachContact(ndContact* const) {}
	virtual void DetachContact(ndContact* const) {}
	virtual ndContact* FindContact(const ndBody* const) const { return nullptr; }

	ndBodyNotify* m_notifyCallback;
	ndSpecialList<ndBody>::ndNode* m_deletedNode;

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
	ndUnsigned8 m_equilibrium;
	ndUnsigned8 m_equilibrium0;
	ndUnsigned8 m_isJointFence0;
	ndUnsigned8 m_isJointFence1;
	ndUnsigned8 m_isConstrained;
	ndUnsigned8 m_sceneForceUpdate;
	ndUnsigned8 m_sceneEquilibrium;
	ndUnsigned8 m_skeletonSelfCollision;
	
	ndMatrix m_matrix;
	ndQuaternion m_rotation;
	ndVector m_veloc;
	ndVector m_omega;
	ndVector m_localCentreOfMass;
	ndVector m_globalCentreOfMass;
	ndVector m_minAabb;
	ndVector m_maxAabb;
	D_MEMORY_ALIGN_FIXUP

	D_COLLISION_API static ndUnsigned32 m_uniqueIdCount;

	friend class ndWorld;
	friend class ndScene;
	friend class ndConstraint;
	friend class ndBodyPlayerCapsuleImpulseSolver;
} D_GCC_NEWTON_ALIGN_32;

#endif 

