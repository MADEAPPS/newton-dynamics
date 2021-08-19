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

#ifndef __D_BODY_KINEMATIC_H__
#define __D_BODY_KINEMATIC_H__

#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndBodyList.h"
#include "ndJointList.h"

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
		ndContactkey(dUnsigned32 tag0, dUnsigned32 tag1)
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
			dUnsigned64 m_tag;
			struct
			{
				dUnsigned32 m_tagLow;
				dUnsigned32 m_tagHigh;
			};
		};
	};
	public:
	class ndContactMap: public dTree<ndContact*, ndContactkey, dContainersFreeListAlloc<ndContact*>>
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
	D_COLLISION_API ndBodyKinematic(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache);
	D_COLLISION_API virtual ~ndBodyKinematic();

	ndScene* GetScene() const;

	dUnsigned32 GetIndex() const;
	dFloat32 GetInvMass() const;
	const dVector GetInvInertia() const;
	const dVector& GetMassMatrix() const;
	const dMatrix& GetInvInertiaMatrix() const;

	void SetMassMatrix(const dVector& massMatrix);

	dVector GetGyroAlpha() const;

	bool GetSleepState() const;
	void RestoreSleepState(bool state);
	D_COLLISION_API void SetSleepState(bool state);

	bool GetAutoSleep() const;
	void SetAutoSleep(bool state);
	void SetDebugMaxAngularIntegrationSteepAndLinearSpeed(dFloat32 angleInRadian, dFloat32 speedInMitersPerSeconds);

	virtual dFloat32 GetLinearDamping() const;
	virtual void SetLinearDamping(dFloat32 linearDamp);

	virtual dVector GetAngularDamping() const;
	virtual void SetAngularDamping(const dVector& angularDamp);

	D_COLLISION_API ndShapeInstance& GetCollisionShape();
	D_COLLISION_API const ndShapeInstance& GetCollisionShape() const;
	D_COLLISION_API virtual void SetCollisionShape(const ndShapeInstance& shapeInstance);
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const dFastRay& ray, const dFloat32 maxT) const;

	D_COLLISION_API dVector CalculateLinearMomentum() const;
	D_COLLISION_API dVector CalculateAngularMomentum() const;
	D_COLLISION_API dFloat32 TotalEnergy() const;

	D_COLLISION_API dMatrix CalculateInertiaMatrix() const;
	D_COLLISION_API virtual dMatrix CalculateInvInertiaMatrix() const;

	D_COLLISION_API void UpdateInvInertiaMatrix();
	D_COLLISION_API virtual void IntegrateVelocity(dFloat32 timestep);

	D_COLLISION_API virtual void Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 shapeHash, dInt32 nodeHash) const;

	void SetMassMatrix(dFloat32 mass, const ndShapeInstance& shapeInstance);
	void SetMassMatrix(dFloat32 Ixx, dFloat32 Iyy, dFloat32 Izz, dFloat32 mass);
	void GetMassMatrix(dFloat32& Ixx, dFloat32& Iyy, dFloat32& Izz, dFloat32& mass);

	D_COLLISION_API virtual ndContact* FindContact(const ndBody* const otherBody) const;

	virtual ndBodyKinematic* GetAsBodyKinematic();

	ndSkeletonContainer* GetSkeleton() const;
	void SetSkeleton(ndSkeletonContainer* const skeleton);

	virtual dVector GetForce() const;
	virtual dVector GetTorque() const;
	virtual void SetForce(const dVector& force);
	virtual void SetTorque(const dVector& torque);

	virtual void SetAccel(const dVector& accel);
	virtual void SetAlpha(const dVector& alpha);

	ndContactMap& GetContactMap();
	const ndJointList& GetJointList() const;
	const ndContactMap& GetContactMap() const;

	protected:
	D_COLLISION_API static void ReleaseMemory();
	D_COLLISION_API virtual void AttachContact(ndContact* const contact);
	D_COLLISION_API virtual void DetachContact(ndContact* const contact);
	D_COLLISION_API virtual void SetMassMatrix(dFloat32 mass, const dMatrix& inertia);

	D_COLLISION_API virtual ndJointList::dNode* AttachJoint(ndJointBilateralConstraint* const joint);
	D_COLLISION_API virtual void DetachJoint(ndJointList::dNode* const node);
	D_COLLISION_API virtual void IntegrateExternalForce(dFloat32 timestep);

	void UpdateCollisionMatrix();
	void PrepareStep(dInt32 index);
	void SetSceneNodes(ndScene* const scene, ndBodyList::dNode* const node);

	ndSceneBodyNode* GetSceneBodyNode() const;
	void SetSceneBodyNode(ndSceneBodyNode* const node);
	virtual void AddDampingAcceleration(dFloat32 timestep);
	
	dMatrix m_invWorldInertiaMatrix;
	ndShapeInstance m_shapeInstance;
	dVector m_mass;
	dVector m_invMass;
	dVector m_residualVeloc;
	dVector m_residualOmega;
	dVector m_gyroAlpha;
	dVector m_gyroTorque;
	dQuaternion m_gyroRotation;

	ndJointList m_jointList;
	ndContactMap m_contactList;
	mutable dSpinLock m_lock;
	ndScene* m_scene;
	ndBodyKinematic* m_islandParent;
	ndBodyList::dNode* m_sceneNode;
	ndSceneBodyNode* m_sceneBodyBodyNode;
	ndSkeletonContainer* m_skeletonContainer;

	dFloat32 m_maxAngleStep;
	dFloat32 m_maxLinearSpeed;
	dFloat32 m_weigh;
	dInt32 m_rank;
	dInt32 m_index;
	dInt32 m_sleepingCounter;

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

inline dUnsigned32 ndBodyKinematic::GetIndex() const
{
	return m_index;
}

inline dFloat32 ndBodyKinematic::GetInvMass() const
{
	return m_invMass.m_w;
}

inline const dVector ndBodyKinematic::GetInvInertia() const
{
	return m_invMass & dVector::m_triplexMask;
}

inline const dVector& ndBodyKinematic::GetMassMatrix() const
{
	return m_mass;
}

inline const dMatrix& ndBodyKinematic::GetInvInertiaMatrix() const
{
	return m_invWorldInertiaMatrix;
}

inline dVector ndBodyKinematic::GetGyroAlpha() const
{
	return m_gyroAlpha;
}

inline void ndBodyKinematic::GetMassMatrix(dFloat32& Ixx, dFloat32& Iyy, dFloat32& Izz, dFloat32& mass)
{
	Ixx = m_mass.m_x;
	Iyy = m_mass.m_y;
	Izz = m_mass.m_z;
	mass = m_mass.m_w;
}

inline void ndBodyKinematic::SetMassMatrix(const dVector& massMatrix)
{
	dMatrix inertia(dGetZeroMatrix());
	inertia[0][0] = massMatrix.m_x;
	inertia[1][1] = massMatrix.m_y;
	inertia[2][2] = massMatrix.m_z;
	SetMassMatrix(massMatrix.m_w, inertia);
}

inline void ndBodyKinematic::SetMassMatrix(dFloat32 Ixx, dFloat32 Iyy, dFloat32 Izz, dFloat32 mass)
{
	SetMassMatrix(dVector(Ixx, Iyy, Izz, mass));
}

inline void ndBodyKinematic::SetMassMatrix(dFloat32 mass, const ndShapeInstance& shapeInstance)
{
	dMatrix inertia(shapeInstance.CalculateInertia());

	dVector origin(inertia.m_posit);
	for (dInt32 i = 0; i < 3; i++) 
	{
		inertia[i] = inertia[i].Scale(mass);
		//inertia[i][i] = (inertia[i][i] + origin[i] * origin[i]) * mass;
		//for (dInt32 j = i + 1; j < 3; j ++) {
		//	dFloat32 crossIJ = origin[i] * origin[j];
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

inline void ndBodyKinematic::SetSceneNodes(ndScene* const scene, ndBodyList::dNode* const node)
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

inline dVector ndBodyKinematic::GetForce() const
{
	return dVector::m_zero;
}

inline dVector ndBodyKinematic::GetTorque() const
{
	return dVector::m_zero;
}

inline void ndBodyKinematic::SetForce(const dVector&)
{
}

inline void ndBodyKinematic::SetTorque(const dVector&)
{
}

inline void ndBodyKinematic::SetAccel(const dVector&)
{
}

inline void ndBodyKinematic::SetAlpha(const dVector&)
{
}

inline void ndBodyKinematic::AddDampingAcceleration(dFloat32)
{
}

inline void ndBodyKinematic::PrepareStep(dInt32 index)
{
	m_rank = 0;
	m_index = index;
	m_resting = 1;
	m_solverSleep0 = 1;
	m_solverSleep1 = 1;
	m_islandSleep = m_equilibrium;
	m_weigh = dFloat32(0.0f);
	m_islandParent = this;
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
	return m_autoSleep;
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

inline void ndBodyKinematic::SetDebugMaxAngularIntegrationSteepAndLinearSpeed(dFloat32 angleInRadian, dFloat32 speedInMitersPerSeconds)
{
	m_maxAngleStep = dMax(dAbs(angleInRadian), dFloat32(90.0f) * dDegreeToRad);
	m_maxLinearSpeed = dMax(speedInMitersPerSeconds, dFloat32 (100.0f));
}

inline void ndBodyKinematic::SetLinearDamping(dFloat32)
{
}

inline dFloat32 ndBodyKinematic::GetLinearDamping() const
{
	return dFloat32(0.0f);
}

inline void ndBodyKinematic::SetAngularDamping(const dVector&)
{
}

inline dVector ndBodyKinematic::GetAngularDamping() const
{
	return dVector::m_zero;
}

#endif 

