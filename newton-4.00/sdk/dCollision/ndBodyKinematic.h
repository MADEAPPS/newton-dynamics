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

#ifndef __ND_BODY_KINEMATIC_H__
#define __ND_BODY_KINEMATIC_H__

#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndConstraint.h"
#include "ndBodyListView.h"

class ndScene;
class ndModel;
class ndSkeletonContainer;
class ndJointBilateralConstraint;

#define	D_FREEZZING_VELOCITY_DRAG	ndFloat32 (0.9f)
#define	D_SOLVER_MAX_ACCEL_ERROR	(D_FREEZE_MAG * ndFloat32 (0.5f))

D_MSV_NEWTON_ALIGN_32
class ndBodyKinematic : public ndBody
{
	class ndContactkey
	{
		public:
		ndContactkey(ndUnsigned32 tag0, ndUnsigned32 tag1);

		bool operator> (const ndContactkey& key) const;
		bool operator< (const ndContactkey& key) const;
		bool operator== (const ndContactkey& key) const;
		private:
		union
		{
			ndUnsigned64 m_tag;
			class
			{
				public:
				ndUnsigned32 m_tagLow;
				ndUnsigned32 m_tagHigh;
			};
		};
	};

	public:
	class ndJointList : public ndList<ndJointBilateralConstraint*, ndContainersFreeListAlloc<ndJointBilateralConstraint*>>
	{
		public:
		ndJointList()
			:ndList<ndJointBilateralConstraint*, ndContainersFreeListAlloc<ndJointBilateralConstraint*>>()
		{
		}
	};

	class ndContactMap: public ndTree<ndContact*, ndContactkey, ndContainersFreeListAlloc<ndContact*>>
	{
		public:
		D_COLLISION_API ndContact* FindContact(const ndBody* const body0, const ndBody* const body1) const;

		private:
		ndContactMap();
		~ndContactMap();
		void AttachContact(ndContact* const contact);
		void DetachContact(ndContact* const contact);
		friend class ndBodyKinematic;
	};

	D_CLASS_REFLECTION(ndBodyKinematic,ndBody)

	D_COLLISION_API ndBodyKinematic();
	D_COLLISION_API ndBodyKinematic(const ndBodyKinematic& src);
	D_COLLISION_API virtual ~ndBodyKinematic();

	ndScene* GetScene() const;

	ndUnsigned32 GetIndex() const;
	ndFloat32 GetInvMass() const;
	const ndVector GetInvInertia() const;
	const ndVector& GetMassMatrix() const;
	const ndMatrix& GetInvInertiaMatrix() const;

	ndVector GetGyroAlpha() const;
	ndVector GetGyroTorque() const;

	bool GetSleepState() const;
	void RestoreSleepState(bool state);
	D_COLLISION_API void SetSleepState(bool state);

	bool GetAutoSleep() const;
	void SetAutoSleep(bool state);
	ndFloat32 GetMaxLinearStep() const;
	ndFloat32 GetMaxAngularStep() const;
	void SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32 angleInRadian, ndFloat32 stepInUnitPerSeconds);

	virtual ndFloat32 GetLinearDamping() const;
	virtual void SetLinearDamping(ndFloat32 linearDamp);

	virtual ndVector GetCachedDamping() const;
	virtual ndVector GetAngularDamping() const;
	virtual void SetAngularDamping(const ndVector& angularDamp);

	D_COLLISION_API ndShapeInstance& GetCollisionShape();
	D_COLLISION_API const ndShapeInstance& GetCollisionShape() const;
	D_COLLISION_API virtual void SetCollisionShape(const ndShapeInstance& shapeInstance);
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const ndFastRay& ray, const ndFloat32 maxT) const;

	D_COLLISION_API ndVector CalculateLinearMomentum() const;
	D_COLLISION_API virtual ndVector CalculateAngularMomentum() const;
	D_COLLISION_API ndFloat32 TotalEnergy() const;

	D_COLLISION_API void ClearMemory();
	D_COLLISION_API virtual void IntegrateVelocity(ndFloat32 timestep);
	D_COLLISION_API void SetMatrixUpdateScene(const ndMatrix& matrix);
	D_COLLISION_API virtual ndContact* FindContact(const ndBody* const otherBody) const;

	D_COLLISION_API ndJacobian CalculateNetForce() const;
	D_COLLISION_API ndMatrix CalculateInertiaMatrix() const;
	D_COLLISION_API virtual ndMatrix CalculateInvInertiaMatrix() const;
	D_COLLISION_API virtual void SetMassMatrix(ndFloat32 mass, const ndMatrix& inertia);
	D_COLLISION_API void SetMassMatrix(ndFloat32 mass, const ndShapeInstance& shapeInstance, bool fullInertia = false);
	D_COLLISION_API void SetIntrinsicMassMatrix(ndFloat32 mass, const ndShapeInstance& shapeInstance, bool fullInertia = false);

	D_COLLISION_API virtual void InitSurrogateBody(ndBodyKinematic* const surrogate) const;
	
	void UpdateInvInertiaMatrix();
	void SetMassMatrix(const ndVector& massMatrix);
	void SetMassMatrix(ndFloat32 Ixx, ndFloat32 Iyy, ndFloat32 Izz, ndFloat32 mass);

	ndMatrix GetPrincipalAxis() const;
	void GetMassMatrix(ndFloat32& Ixx, ndFloat32& Iyy, ndFloat32& Izz, ndFloat32& mass);

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

	ndJointList& GetJointList();
	ndContactMap& GetContactMap();
	const ndJointList& GetJointList() const;
	const ndContactMap& GetContactMap() const;

	protected:
	D_COLLISION_API virtual void AttachContact(ndContact* const contact);
	D_COLLISION_API virtual void DetachContact(ndContact* const contact);

	D_COLLISION_API virtual void DetachJoint(ndJointList::ndNode* const node);
	D_COLLISION_API virtual ndJointList::ndNode* AttachJoint(ndJointBilateralConstraint* const joint);

	D_COLLISION_API virtual void IntegrateExternalForce(ndFloat32 timestep);

	void SetAccel(const ndJacobian& accel);
	virtual void SpecialUpdate(ndFloat32 timestep);
	virtual void IntegrateGyroSubstep(const ndVector& timestep);
	virtual void ApplyExternalForces(ndInt32 threadIndex, ndFloat32 timestep);
	virtual ndJacobian IntegrateForceAndToque(const ndVector& force, const ndVector& torque, const ndVector& timestep) const;

	void UpdateCollisionMatrix();
	void PrepareStep(ndInt32 index);
	void SetSceneNodes(ndScene* const scene, ndBodyListView::ndNode* const node);

	virtual void AddDampingAcceleration(ndFloat32 timestep);
	
	D_COLLISION_API virtual void EvaluateSleepState(ndFloat32 freezeSpeed2, ndFloat32 freezeAccel2);

	virtual void SetAcceleration(const ndVector& accel, const ndVector& alpha);

	ndMatrix m_inertiaPrincipalAxis;
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
	ndBodyListView::ndNode* m_sceneNode;
	ndSkeletonContainer* m_skeletonContainer;
	ndSpecialList<ndBodyKinematic>::ndNode* m_spetialUpdateNode;

	ndFloat32 m_maxAngleStep;
	ndFloat32 m_maxLinearStep;
	ndFloat32 m_weigh;
	ndInt32 m_index;
	ndInt32 m_bodyNodeIndex;
	ndInt32 m_buildSkelIndex;
	ndInt32 m_sceneNodeIndex;
	ndInt32 m_buildBodyNodeIndex;
	ndInt32 m_buildSceneNodeIndex;

	D_COLLISION_API static ndVector m_velocTol;

	friend class ndWorld;
	friend class ndScene;
	friend class ndContact;
	friend class ndIkSolver;
	friend class ndBvhLeafNode;
	friend class ndDynamicsUpdate;
	friend class ndWorldSceneSycl;
	friend class ndWorldSceneCuda;
	friend class ndBvhSceneManager;
	friend class ndSkeletonContainer;
	friend class ndModelArticulation;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateSycl;
	friend class ndDynamicsUpdateCuda;
	friend class ndJointBilateralConstraint;
} D_GCC_NEWTON_ALIGN_32;


class ndBodySentinel : public ndBodyKinematic
{
	ndBodySentinel* GetAsBodySentinel() { return this; }
};

inline ndUnsigned32 ndBodyKinematic::GetIndex() const
{
	return ndUnsigned32(m_index);
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
	ndMatrix inertia(ndGetIdentityMatrix());
	inertia[0][0] = massMatrix.m_x;
	inertia[1][1] = massMatrix.m_y;
	inertia[2][2] = massMatrix.m_z;
	SetMassMatrix(massMatrix.m_w, inertia);
}

inline void ndBodyKinematic::SetMassMatrix(ndFloat32 Ixx, ndFloat32 Iyy, ndFloat32 Izz, ndFloat32 mass)
{
	SetMassMatrix(ndVector(Ixx, Iyy, Izz, mass));
}

inline ndMatrix ndBodyKinematic::GetPrincipalAxis() const
{
	return m_inertiaPrincipalAxis;
}

inline ndBodyKinematic* ndBodyKinematic::GetAsBodyKinematic() 
{ 
	return this; 
}

inline ndScene* ndBodyKinematic::GetScene() const
{
	return m_scene;
}

inline void ndBodyKinematic::SetSceneNodes(ndScene* const scene, ndBodyListView::ndNode* const node)
{
	m_scene = scene;
	m_sceneNode = node;
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
	m_index = index;
	m_isJointFence0 = 1;
	m_isJointFence1 = 1;
	m_isConstrained = 0;
	m_buildSkelIndex = 0;
	m_islandParent = this;
	m_weigh = ndFloat32(0.0f);
	m_isStatic = ndUnsigned8(m_invMass.m_w == ndFloat32(0.0f));
	m_equilibrium = ndUnsigned8 (m_isStatic | m_equilibrium);
	m_equilibrium0 = m_equilibrium;
}

inline ndBodyKinematic::ndContactMap& ndBodyKinematic::GetContactMap()
{
	return m_contactList;
}

inline const ndBodyKinematic::ndContactMap& ndBodyKinematic::GetContactMap() const
{
	return m_contactList;
}

inline ndBodyKinematic::ndJointList& ndBodyKinematic::GetJointList()
{
	return m_jointList;
}

inline const ndBodyKinematic::ndJointList& ndBodyKinematic::GetJointList() const
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
	m_equilibrium = ndUnsigned8 (state ? 1 : 0);
}

inline void ndBodyKinematic::SetAutoSleep(bool state)
{
	m_autoSleep = ndUnsigned8 (state ? 1 : 0);
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

inline ndFloat32 ndBodyKinematic::GetMaxLinearStep() const
{
	return m_maxLinearStep;
}

inline ndFloat32 ndBodyKinematic::GetMaxAngularStep() const
{
	return m_maxAngleStep;
}

inline void ndBodyKinematic::SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32 angleInRadian, ndFloat32 stepInUnitPerSeconds)
{
	m_maxLinearStep = ndMax(ndAbs(stepInUnitPerSeconds), ndFloat32(1.0f));
	m_maxAngleStep = ndMax(ndAbs(angleInRadian), ndFloat32(90.0f) * ndDegreeToRad);
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

inline ndVector ndBodyKinematic::GetCachedDamping() const
{
	return ndVector::m_one;
}

inline void ndBodyKinematic::UpdateInvInertiaMatrix()
{
	ndAssert(m_invWorldInertiaMatrix[0][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[1][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[2][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[3][3] == ndFloat32(1.0f));

	m_invWorldInertiaMatrix = CalculateInvInertiaMatrix();

	ndAssert(m_invWorldInertiaMatrix[0][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[1][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[2][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[3][3] == ndFloat32(1.0f));
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

inline void ndBodyKinematic::SpecialUpdate(ndFloat32)
{
	ndAssert(0);
}

inline void ndBodyKinematic::ApplyExternalForces(ndInt32, ndFloat32)
{
}

inline void ndBodyKinematic::SetAcceleration(const ndVector&, const ndVector&)
{
	m_accel = ndVector::m_zero;
	m_alpha = ndVector::m_zero;
}

#endif 

