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

#ifndef __ND_BODY_KINEMATIC_H__
#define __ND_BODY_KINEMATIC_H__

#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndBodyList.h"
#include "ndJointList.h"
#include "ndConstraint.h"

class ndScene;
class ndSceneBodyNode;
class ndSkeletonContainer;
class ndJointBilateralConstraint;

D_MSV_NEWTON_ALIGN_32
class ndBodyKinematic: public ndBody
{
	class ndContactkey
	{
		public:
		ndContactkey(ndUnsigned32 tag0, ndUnsigned32 tag1)
			:m_tagLow(dMin(tag0, tag1))
			,m_tagHigh(dMax(tag0, tag1))
		{
			dAssert(m_tagLow < m_tagHigh);
		}

		bool operator== (const ndContactkey& key) const
		{
			return m_tag == key.m_tag;
		}

		bool operator< (const ndContactkey& key) const
		{
			return m_tag < key.m_tag;
		}

		bool operator> (const ndContactkey& key) const
		{
			return m_tag > key.m_tag;
		}

		private:
		union
		{
			ndUnsigned64 m_tag;
			struct
			{
				ndUnsigned32 m_tagLow;
				ndUnsigned32 m_tagHigh;
			};
		};
	};
	public:
	class ndContactMap: public ndTree<ndContact*, ndContactkey, ndContainersFreeListAlloc<ndContact*>>
	{
		public:
		ndContactMap();
		D_COLLISION_API ndContact* FindContact(const ndBody* const body0, const ndBody* const body1) const;

		private:
		void AttachContact(ndContact* const contact);
		void DetachContact(ndContact* const contact);
		friend class ndBodyKinematic;
	};

	D_CLASS_REFLECTION(ndBodyKinematic);
	D_COLLISION_API ndBodyKinematic();
	D_COLLISION_API ndBodyKinematic(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_COLLISION_API virtual ~ndBodyKinematic();

	ndScene* GetScene() const;

	ndUnsigned32 GetIndex() const;
	ndFloat32 GetInvMass() const;
	const ndVector GetInvInertia() const;
	const ndVector& GetMassMatrix() const;
	const ndMatrix& GetInvInertiaMatrix() const;

	void SetMassMatrix(const ndVector& massMatrix);

	ndVector GetGyroAlpha() const;
	ndVector GetGyroTorque() const;

	bool GetSleepState() const;
	void RestoreSleepState(bool state);
	D_COLLISION_API void SetSleepState(bool state);

	bool GetAutoSleep() const;
	void SetAutoSleep(bool state);
	void SetDebugMaxAngularIntegrationSteepAndLinearSpeed(ndFloat32 angleInRadian, ndFloat32 speedInMitersPerSeconds);

	virtual ndFloat32 GetLinearDamping() const;
	virtual void SetLinearDamping(ndFloat32 linearDamp);

	virtual ndVector GetAngularDamping() const;
	virtual void SetAngularDamping(const ndVector& angularDamp);

	D_COLLISION_API ndShapeInstance& GetCollisionShape();
	D_COLLISION_API const ndShapeInstance& GetCollisionShape() const;
	D_COLLISION_API virtual void SetCollisionShape(const ndShapeInstance& shapeInstance);
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const ndFastRay& ray, const ndFloat32 maxT) const;

	D_COLLISION_API ndVector CalculateLinearMomentum() const;
	D_COLLISION_API ndVector CalculateAngularMomentum() const;
	D_COLLISION_API ndFloat32 TotalEnergy() const;

	D_COLLISION_API ndMatrix CalculateInertiaMatrix() const;
	D_COLLISION_API virtual ndMatrix CalculateInvInertiaMatrix() const;
	
	D_COLLISION_API virtual void IntegrateVelocity(ndFloat32 timestep);
	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	void UpdateInvInertiaMatrix();
	void SetMassMatrix(ndFloat32 mass, const ndShapeInstance& shapeInstance);
	void SetMassMatrix(ndFloat32 Ixx, ndFloat32 Iyy, ndFloat32 Izz, ndFloat32 mass);
	void GetMassMatrix(ndFloat32& Ixx, ndFloat32& Iyy, ndFloat32& Izz, ndFloat32& mass);

	D_COLLISION_API virtual ndContact* FindContact(const ndBody* const otherBody) const;

	virtual ndBodyKinematic* GetAsBodyKinematic();

	ndSkeletonContainer* GetSkeleton() const;
	void SetSkeleton(ndSkeletonContainer* const skeleton);

	virtual ndVector GetForce() const;
	virtual ndVector GetTorque() const;

	virtual void SetForce(const ndVector& force);
	virtual void SetTorque(const ndVector& torque);

	virtual void AddImpulse(const ndVector& pointVeloc, const ndVector& pointPosit, ndFloat32 timestep);
	virtual void ApplyImpulsePair(const ndVector& linearImpulse, const ndVector& angularImpulse, ndFloat32 timestep);
	virtual void ApplyImpulsesAtPoint(ndInt32 count, const ndVector* const impulseArray, const ndVector* const pointArray, ndFloat32 timestep);

	ndVector GetAccel() const;
	ndVector GetAlpha() const;
	void SetAccel(const ndVector& accel);
	void SetAlpha(const ndVector& alpha);

	ndContactMap& GetContactMap();
	const ndJointList& GetJointList() const;
	const ndContactMap& GetContactMap() const;

	protected:
	D_COLLISION_API virtual void AttachContact(ndContact* const contact);
	D_COLLISION_API virtual void DetachContact(ndContact* const contact);
	D_COLLISION_API virtual void SetMassMatrix(ndFloat32 mass, const ndMatrix& inertia);

	D_COLLISION_API virtual ndJointList::ndNode* AttachJoint(ndJointBilateralConstraint* const joint);
	D_COLLISION_API virtual void DetachJoint(ndJointList::ndNode* const node);
	D_COLLISION_API virtual void IntegrateExternalForce(ndFloat32 timestep);

	void SetAccel(const ndJacobian& accel);
	virtual void IntegrateGyroSubstep(const ndVector& timestep);
	virtual ndJacobian IntegrateForceAndToque(const ndVector& force, const ndVector& torque, const ndVector& timestep) const;

	void UpdateCollisionMatrix();
	void PrepareStep(ndInt32 index);
	void SetSceneNodes(ndScene* const scene, ndBodyList::ndNode* const node);

	ndSceneBodyNode* GetSceneBodyNode() const;
	void SetSceneBodyNode(ndSceneBodyNode* const node);
	virtual void AddDampingAcceleration(ndFloat32 timestep);
	
	ndMatrix m_invWorldInertiaMatrix;
	ndShapeInstance m_shapeInstance;
	ndVector m_mass;
	ndVector m_invMass;
	ndVector m_accel;
	ndVector m_alpha;
	ndVector m_gyroAlpha;
	ndVector m_gyroTorque;
	ndQuaternion m_gyroRotation;
	ndJointList m_jointList;
	ndContactMap m_contactList;
	mutable ndSpinLock m_lock;
	ndScene* m_scene;
	ndBodyKinematic* m_islandParent;
	ndBodyList::ndNode* m_sceneNode;
	ndSceneBodyNode* m_sceneBodyBodyNode;
	ndSkeletonContainer* m_skeletonContainer;

	ndFloat32 m_maxAngleStep;
	ndFloat32 m_maxLinearSpeed;
	ndFloat32 m_weigh;
	ndInt32 m_rank;
	ndInt32 m_index;
	ndInt32 m_sleepingCounter;

	friend class ndWorld;
	friend class ndScene;
	friend class ndContact;
	friend class ndSceneBodyNode;
	friend class ndDynamicsUpdate;
	friend class ndSkeletonContainer;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateOpencl;
	friend class ndJointBilateralConstraint;
} D_GCC_NEWTON_ALIGN_32;

inline ndUnsigned32 ndBodyKinematic::GetIndex() const
{
	return m_index;
}

inline ndFloat32 ndBodyKinematic::GetInvMass() const
{
	return m_invMass.m_w;
}

inline const ndVector ndBodyKinematic::GetInvInertia() const
{
	return m_invMass & ndVector::m_triplexMask;
}

inline const ndVector& ndBodyKinematic::GetMassMatrix() const
{
	return m_mass;
}

inline const ndMatrix& ndBodyKinematic::GetInvInertiaMatrix() const
{
	return m_invWorldInertiaMatrix;
}

inline ndVector ndBodyKinematic::GetGyroAlpha() const
{
	return m_gyroAlpha;
}

inline ndVector ndBodyKinematic::GetGyroTorque() const
{
	return m_gyroTorque;
}

inline void ndBodyKinematic::GetMassMatrix(ndFloat32& Ixx, ndFloat32& Iyy, ndFloat32& Izz, ndFloat32& mass)
{
	Ixx = m_mass.m_x;
	Iyy = m_mass.m_y;
	Izz = m_mass.m_z;
	mass = m_mass.m_w;
}

inline void ndBodyKinematic::SetMassMatrix(const ndVector& massMatrix)
{
	ndMatrix inertia(dGetZeroMatrix());
	inertia[0][0] = massMatrix.m_x;
	inertia[1][1] = massMatrix.m_y;
	inertia[2][2] = massMatrix.m_z;
	SetMassMatrix(massMatrix.m_w, inertia);
}

inline void ndBodyKinematic::SetMassMatrix(ndFloat32 Ixx, ndFloat32 Iyy, ndFloat32 Izz, ndFloat32 mass)
{
	SetMassMatrix(ndVector(Ixx, Iyy, Izz, mass));
}

inline void ndBodyKinematic::SetMassMatrix(ndFloat32 mass, const ndShapeInstance& shapeInstance)
{
	ndMatrix inertia(shapeInstance.CalculateInertia());

	ndVector origin(inertia.m_posit);
	for (ndInt32 i = 0; i < 3; i++) 
	{
		inertia[i] = inertia[i].Scale(mass);
		//inertia[i][i] = (inertia[i][i] + origin[i] * origin[i]) * mass;
		//for (ndInt32 j = i + 1; j < 3; j ++) {
		//	ndFloat32 crossIJ = origin[i] * origin[j];
		//	inertia[i][j] = (inertia[i][j] + crossIJ) * mass;
		//	inertia[j][i] = (inertia[j][i] + crossIJ) * mass;
		//}
	}

	// although the engine fully supports asymmetric inertia, I will ignore cross inertia for now
	SetCentreOfMass(origin);
	SetMassMatrix(mass, inertia);
}

inline ndBodyKinematic* ndBodyKinematic::GetAsBodyKinematic() 
{ 
	return this; 
}

inline ndScene* ndBodyKinematic::GetScene() const
{
	return m_scene;
}

inline void ndBodyKinematic::SetSceneNodes(ndScene* const scene, ndBodyList::ndNode* const node)
{
	m_scene = scene;
	m_sceneNode = node;
}

inline ndSceneBodyNode* ndBodyKinematic::GetSceneBodyNode() const
{
	return m_sceneBodyBodyNode;
}

inline void ndBodyKinematic::SetSceneBodyNode(ndSceneBodyNode* const node)
{
	m_sceneBodyBodyNode = node;
}

inline ndVector ndBodyKinematic::GetForce() const
{
	return ndVector::m_zero;
}

inline ndVector ndBodyKinematic::GetTorque() const
{
	return ndVector::m_zero;
}

inline void ndBodyKinematic::SetForce(const ndVector&)
{
}

inline void ndBodyKinematic::SetTorque(const ndVector&)
{
}

inline ndVector ndBodyKinematic::GetAccel() const
{
	return m_accel;
}

inline void ndBodyKinematic::SetAccel(const ndVector& accel)
{
	m_accel = accel;
}

inline ndVector ndBodyKinematic::GetAlpha() const
{
	return m_alpha;
}

inline void ndBodyKinematic::SetAlpha(const ndVector& alpha)
{
	m_alpha = alpha;
}

inline void ndBodyKinematic::AddDampingAcceleration(ndFloat32)
{
}

inline void ndBodyKinematic::SetAccel(const ndJacobian& accel)
{
	SetAccel(accel.m_linear);
	SetAlpha(accel.m_angular);
}


inline void ndBodyKinematic::PrepareStep(ndInt32 index)
{
	m_rank = 0;
	m_index = index;
	m_isJointFence0 = 1;
	m_isJointFence1 = 1;
	m_islandParent = this;
	m_bodyIsConstrained = 0;
	m_weigh = ndFloat32(1.0f);
	m_isStatic = (m_invMass.m_w == ndFloat32(0.0f));
	m_equilibrium = m_isStatic | m_equilibrium;
	m_islandSleep = m_equilibrium;
	m_equilibrium0 = m_equilibrium;

	m_resting = 1;
}

inline ndBodyKinematic::ndContactMap& ndBodyKinematic::GetContactMap()
{
	return m_contactList;
}

inline const ndBodyKinematic::ndContactMap& ndBodyKinematic::GetContactMap() const
{
	return m_contactList;
}

inline const ndJointList& ndBodyKinematic::GetJointList() const
{
	return m_jointList;
}

inline ndShapeInstance& ndBodyKinematic::GetCollisionShape()
{
	return (ndShapeInstance&)m_shapeInstance;
}

inline const ndShapeInstance& ndBodyKinematic::GetCollisionShape() const
{
	return m_shapeInstance;
}

inline bool ndBodyKinematic::GetAutoSleep() const
{
	return m_autoSleep ? true : false;
}

inline bool ndBodyKinematic::GetSleepState() const
{
	return m_equilibrium ? true : false;
}

inline void ndBodyKinematic::RestoreSleepState(bool state)
{
	m_equilibrium = state ? 1 : 0;
}

inline void ndBodyKinematic::SetAutoSleep(bool state)
{
	m_autoSleep = state ? 1 : 0;
	SetSleepState(false);
}

inline ndSkeletonContainer* ndBodyKinematic::GetSkeleton() const
{ 
	return m_skeletonContainer;
}

inline void ndBodyKinematic::SetSkeleton(ndSkeletonContainer* const skeleton)
{
	m_skeletonContainer = skeleton;
}

inline void ndBodyKinematic::SetDebugMaxAngularIntegrationSteepAndLinearSpeed(ndFloat32 angleInRadian, ndFloat32 speedInMitersPerSeconds)
{
	m_maxAngleStep = dMax(dAbs(angleInRadian), ndFloat32(90.0f) * ndDegreeToRad);
	m_maxLinearSpeed = dMax(speedInMitersPerSeconds, ndFloat32 (100.0f));
}

inline void ndBodyKinematic::SetLinearDamping(ndFloat32)
{
}

inline ndFloat32 ndBodyKinematic::GetLinearDamping() const
{
	return ndFloat32(0.0f);
}

inline void ndBodyKinematic::SetAngularDamping(const ndVector&)
{
}

inline ndVector ndBodyKinematic::GetAngularDamping() const
{
	return ndVector::m_zero;
}

inline void ndBodyKinematic::UpdateInvInertiaMatrix()
{
	dAssert(m_invWorldInertiaMatrix[0][3] == ndFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[1][3] == ndFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[2][3] == ndFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[3][3] == ndFloat32(1.0f));

	m_invWorldInertiaMatrix = CalculateInvInertiaMatrix();

	dAssert(m_invWorldInertiaMatrix[0][3] == ndFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[1][3] == ndFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[2][3] == ndFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[3][3] == ndFloat32(1.0f));
}

inline void ndBodyKinematic::IntegrateGyroSubstep(const ndVector&)
{
}

inline ndJacobian ndBodyKinematic::IntegrateForceAndToque(const ndVector&, const ndVector&, const ndVector&) const
{
	ndJacobian step;
	step.m_linear = ndVector::m_zero;
	step.m_angular = ndVector::m_zero;
	return step;
}

inline void ndBodyKinematic::AddImpulse(const ndVector&, const ndVector&, ndFloat32)
{
}

inline void ndBodyKinematic::ApplyImpulsePair(const ndVector&, const ndVector&, ndFloat32)
{
}

inline void ndBodyKinematic::ApplyImpulsesAtPoint(ndInt32, const ndVector* const, const ndVector* const, ndFloat32)
{
}

#endif 

