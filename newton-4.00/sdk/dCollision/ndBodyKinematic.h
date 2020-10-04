/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

class ndSceneBodyNode;
class ndSceneAggregate;
class ndSkeletonContainer;

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

	D_COLLISION_API ndBodyKinematic();
	D_COLLISION_API virtual ~ndBodyKinematic();

	ndScene* GetBroadPhase() const;

	const dUnsigned32 GetIndex() const;
	const dFloat32 GetInvMass() const;
	dVector GetMassMatrix() const;
	void SetMassMatrix(const dVector& massMatrix);

	dVector GetGyroAlpha() const;

	D_COLLISION_API bool GetSleepState() const;
	D_COLLISION_API void SetSleepState(bool state);

	D_COLLISION_API bool GetAutoSleep() const;
	D_COLLISION_API void SetAutoSleep(bool state);

	D_COLLISION_API ndShapeInstance& GetCollisionShape();
	D_COLLISION_API const ndShapeInstance& GetCollisionShape() const;
	D_COLLISION_API void SetCollisionShape(const ndShapeInstance& shapeInstance);

	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dFastRayTest& ray, const dFloat32 maxT) const;

	D_COLLISION_API dVector CalculateLinearMomentum() const;
	D_COLLISION_API dVector CalculateAngularMomentum() const;

	D_COLLISION_API dMatrix CalculateInvInertiaMatrix() const;
	D_COLLISION_API void UpdateInvInertiaMatrix();

	D_COLLISION_API virtual void IntegrateVelocity(dFloat32 timestep);

	void SetMassMatrix(dFloat32 mass, const ndShapeInstance& shapeInstance);
	void SetMassMatrix(dFloat32 Ixx, dFloat32 Iyy, dFloat32 Izz, dFloat32 mass);
	void GetMassMatrix(dFloat32& Ixx, dFloat32& Iyy, dFloat32& Izz, dFloat32& mass);

	D_COLLISION_API virtual ndContact* FindContact(const ndBody* const otherBody) const;

	virtual ndBodyKinematic* GetAsBodyKinematic() { return this; }
	virtual ndSkeletonContainer* GetSkeleton() const { return nullptr; }

	virtual dVector GetForce() const;
	virtual dVector GetTorque() const;
	virtual void SetAccel(const dVector& accel);
	virtual void SetAlpha(const dVector& alpha);

	const ndContactMap& GetContactMap() const;

	private:
	D_COLLISION_API void SetMassMatrix(dFloat32 mass, const dMatrix& inertia);

	protected:
	D_COLLISION_API static void ReleaseMemory();
	D_COLLISION_API virtual void AttachContact(ndContact* const contact);
	D_COLLISION_API virtual void DetachContact(ndContact* const contact);
	D_COLLISION_API void UpdateGyroData();

	void UpdateCollisionMatrix();
	void PrepareStep(dInt32 index);
	void SetBroadPhase(ndScene* const broadPhase, ndBodyList::dListNode* const node);

	ndSceneBodyNode* GetBroadPhaseBodyNode() const;
	void SetBroadPhaseBodyNode(ndSceneBodyNode* const node);

	ndSceneAggregate* GetBroadPhaseAggregate() const;
	void SetBroadPhaseAggregate(ndSceneAggregate* const node);

	virtual void AddDampingAcceleration(dFloat32 timestep);
	D_COLLISION_API void IntegrateExternalForce(dFloat32 timestep);

	dMatrix m_invWorldInertiaMatrix;
	ndShapeInstance m_shapeInstance;
	dVector m_mass;
	dVector m_invMass;
	dVector m_gyroAlpha;
	dVector m_gyroTorque;
	dQuaternion m_gyroRotation;

	ndContactMap m_contactList;
	dSpinLock m_lock;
	ndScene* m_scene;
	ndBodyKinematic* m_islandParent;
	ndBodyList::dListNode* m_sceneNode;
	ndSceneBodyNode* m_sceneBodyBodyNode;
	ndSceneAggregate* m_sceneAggregateNode;

	dFloat32 m_weight;
	dInt32 m_rank;
	dInt32 m_index;

	friend class ndWorld;
	friend class ndScene;
	friend class ndContact;
	friend class ndSceneMixed;
	friend class ndSceneBodyNode;
	friend class ndDynamicsUpdate;
	friend class ndJointBilateralConstraint;
} D_GCC_NEWTON_ALIGN_32;

inline const dUnsigned32 ndBodyKinematic::GetIndex() const
{
	return m_index;
}

inline const dFloat32 ndBodyKinematic::GetInvMass() const
{
	return m_invMass.m_w;
}

inline dVector ndBodyKinematic::GetMassMatrix() const
{
	return m_mass;
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
	for (dInt32 i = 0; i < 3; i++) {
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

inline ndScene* ndBodyKinematic::GetBroadPhase() const
{
	return m_scene;
}

inline void ndBodyKinematic::SetBroadPhase(ndScene* const broadPhase, ndBodyList::dListNode* const node)
{
	m_sceneNode = node;
	m_scene = broadPhase;
}

inline ndSceneBodyNode* ndBodyKinematic::GetBroadPhaseBodyNode() const
{
	return m_sceneBodyBodyNode;
}

inline void ndBodyKinematic::SetBroadPhaseBodyNode(ndSceneBodyNode* const node)
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

inline void ndBodyKinematic::SetAccel(const dVector& accel)
{
}

inline void ndBodyKinematic::SetAlpha(const dVector& alpha)
{
}

inline void ndBodyKinematic::AddDampingAcceleration(dFloat32 timestep)
{
}

inline void ndBodyKinematic::PrepareStep(dInt32 index)
{
	m_rank = 0;
	m_index = index;
	m_resting = 1;
	m_islandSleep = m_equilibrium;
	m_weight = dFloat32(0.0f);
	m_islandParent = this;
	//m_tmpBodyArray[index] = body;
}

inline const ndBodyKinematic::ndContactMap& ndBodyKinematic::GetContactMap() const
{
	return m_contactList;
}

#endif 

