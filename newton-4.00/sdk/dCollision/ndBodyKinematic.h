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
class ndSkeletonContainer;
class ndJointBilateralConstraint;

#define	D_FREEZZING_VELOCITY_DRAG	ndFloat32 (0.9f)
#define	D_SOLVER_MAX_ACCEL_ERROR	(D_FREEZE_MAG * ndFloat32 (0.5f))

D_MSV_NEWTON_CLASS_ALIGN_32
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

	D_COLLISION_API ndScene* GetScene() const;

	D_COLLISION_API ndUnsigned32 GetIndex() const;
	D_COLLISION_API ndFloat32 GetInvMass() const;
	D_COLLISION_API const ndVector GetInvInertia() const;
	D_COLLISION_API const ndVector& GetMassMatrix() const;
	D_COLLISION_API const ndMatrix& GetInvInertiaMatrix() const;

	D_COLLISION_API ndVector GetGyroAlpha() const;
	D_COLLISION_API ndVector GetGyroTorque() const;

	D_COLLISION_API bool GetSleepState() const;
	D_COLLISION_API void RestoreSleepState(bool state);
	D_COLLISION_API void SetSleepState(bool state);

	D_COLLISION_API bool GetAutoSleep() const;
	D_COLLISION_API void SetAutoSleep(bool state);
	D_COLLISION_API ndFloat32 GetMaxLinearStep() const;
	D_COLLISION_API ndFloat32 GetMaxAngularStep() const;
	D_COLLISION_API void SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32 angleInRadian, ndFloat32 stepInUnitPerSeconds);

	D_COLLISION_API virtual ndFloat32 GetLinearDamping() const;
	D_COLLISION_API virtual void SetLinearDamping(ndFloat32 linearDamp);

	D_COLLISION_API virtual ndVector GetCachedDamping() const;
	D_COLLISION_API virtual ndVector GetAngularDamping() const;
	D_COLLISION_API virtual void SetAngularDamping(const ndVector& angularDamp);

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
	
	D_COLLISION_API void UpdateInvInertiaMatrix();
	D_COLLISION_API void SetMassMatrix(const ndVector& massMatrix);
	D_COLLISION_API void SetMassMatrix(ndFloat32 Ixx, ndFloat32 Iyy, ndFloat32 Izz, ndFloat32 mass);

	D_COLLISION_API ndMatrix GetPrincipalAxis() const;
	D_COLLISION_API void GetMassMatrix(ndFloat32& Ixx, ndFloat32& Iyy, ndFloat32& Izz, ndFloat32& mass);

	D_COLLISION_API virtual ndBodyKinematic* GetAsBodyKinematic();

	D_COLLISION_API ndSkeletonContainer* GetSkeleton() const;
	D_COLLISION_API void SetSkeleton(ndSkeletonContainer* const skeleton);

	D_COLLISION_API virtual ndVector GetForce() const;
	D_COLLISION_API virtual ndVector GetTorque() const;

	D_COLLISION_API virtual void SetForce(const ndVector& force);
	D_COLLISION_API virtual void SetTorque(const ndVector& torque);

	D_COLLISION_API virtual void AddImpulse(const ndVector& pointVeloc, const ndVector& pointPosit, ndFloat32 timestep);
	D_COLLISION_API virtual void ApplyImpulsePair(const ndVector& linearImpulse, const ndVector& angularImpulse, ndFloat32 timestep);
	D_COLLISION_API virtual void ApplyImpulsesAtPoint(ndInt32 count, const ndVector* const impulseArray, const ndVector* const pointArray, ndFloat32 timestep);

	D_COLLISION_API ndVector GetAccel() const;
	D_COLLISION_API ndVector GetAlpha() const;
	D_COLLISION_API void SetAccel(const ndVector& accel);
	D_COLLISION_API void SetAlpha(const ndVector& alpha);

	D_COLLISION_API ndJointList& GetJointList();
	D_COLLISION_API ndContactMap& GetContactMap();
	D_COLLISION_API const ndJointList& GetJointList() const;
	D_COLLISION_API const ndContactMap& GetContactMap() const;

	protected:
	D_COLLISION_API virtual void AttachContact(ndContact* const contact);
	D_COLLISION_API virtual void DetachContact(ndContact* const contact);

	D_COLLISION_API virtual void DetachJoint(ndJointList::ndNode* const node);
	D_COLLISION_API virtual ndJointList::ndNode* AttachJoint(ndJointBilateralConstraint* const joint);

	D_COLLISION_API virtual void IntegrateExternalForce(ndFloat32 timestep);

	D_COLLISION_API void SetAccel(const ndJacobian& accel);
	D_COLLISION_API virtual void SpecialUpdate(ndFloat32 timestep);
	D_COLLISION_API virtual void IntegrateGyroSubstep(const ndVector& timestep);
	D_COLLISION_API virtual void ApplyExternalForces(ndInt32 threadIndex, ndFloat32 timestep);
	D_COLLISION_API virtual ndJacobian IntegrateForceAndToque(const ndVector& force, const ndVector& torque, const ndVector& timestep) const;

	D_COLLISION_API void UpdateCollisionMatrix();
	D_COLLISION_API void PrepareStep(ndInt32 index);
	D_COLLISION_API void SetSceneNodes(ndScene* const scene, ndBodyListView::ndNode* const node);

	D_COLLISION_API virtual void AddDampingAcceleration(ndFloat32 timestep);
	D_COLLISION_API virtual void SetAcceleration(const ndVector& accel, const ndVector& alpha);
	D_COLLISION_API virtual void EvaluateSleepState(ndFloat32 freezeSpeed2, ndFloat32 freezeAccel2);

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
} D_GCC_NEWTON_CLASS_ALIGN_32;


class ndBodySentinel : public ndBodyKinematic
{
	ndBodySentinel* GetAsBodySentinel() { return this; }
};


#endif 

