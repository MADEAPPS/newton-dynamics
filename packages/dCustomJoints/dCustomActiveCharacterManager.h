/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_DYNAMIC_RAGDOLL_MANAGER_H_
#define D_CUSTOM_DYNAMIC_RAGDOLL_MANAGER_H_

#include "dCustomJointLibraryStdAfx.h"
#include "dCustomControllerManager.h"
#include "dCustomBallAndSocket.h"

#define DYNAMIC_RAGDOLL_PLUGIN_NAME	"__dynamicRagDollManager__"


// a Skeleton Transform controller is use to calculate local transform on contractions of rigid bodies and joint that form part of a hierarchical Skeleton
class dCustomActiveCharacterController: public dCustomControllerBase
{
	public:
/*
	class dSelfCollisionBitmask
	{
		public:
		dSelfCollisionBitmask()
		{
			memset(m_mask, 0xff, sizeof(m_mask));
		}

		void GetAddress(int id, int& index, int& shift) const
		{
			int bits = 8 * sizeof(dLong);
			shift = id & (bits - 1);
			index = id / (bits * sizeof(m_mask) / sizeof(m_mask[0]));
		}


		void SetBit(int id)
		{
			int index;
			int shift;
			GetAddress(id, index, shift);
			dLong bit = 1;
			m_mask[index] |= (bit << shift);
		}

		void ResetBit(int id)
		{
			int index;
			int shift;
			GetAddress(id, index, shift);
			dLong bit = 1;
			m_mask[index] &= ~(bit << shift);
		}

		bool TestMask(int id) const
		{
			int index;
			int shift;
			GetAddress(id, index, shift);
			dLong bit = 1;
			return (m_mask[index] & (bit << shift)) ? true : false;
		}

		dLong m_mask[((D_HIERACHICAL_CONTROLLER_MAX_BONES - 1) / (8 * sizeof(dLong))) + 1];
	};

	class dSkeletonBone
	{
		public:
		dMatrix m_bindMatrix;
		NewtonBody* m_body;
		dSkeletonBone* m_parent;
		dCustomArticulatedTransformController* m_myController;
		dSelfCollisionBitmask m_bitField;
	};
*/

/*
	CUSTOM_JOINTS_API void DisableAllSelfCollision();
	CUSTOM_JOINTS_API void SetDefaultSelfCollisionMask();
	CUSTOM_JOINTS_API void SetSelfCollisionMask(dSkeletonBone* const bone0, dSkeletonBone* const bone1, bool mode);

	CUSTOM_JOINTS_API bool SelfCollisionTest(const dSkeletonBone* const bone0, const dSkeletonBone* const bone1) const;
	CUSTOM_JOINTS_API dSkeletonBone* AddBone(NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBone = NULL);

	CUSTOM_JOINTS_API int GetBoneCount() const;
	CUSTOM_JOINTS_API dSkeletonBone* GetBone(int index);
	CUSTOM_JOINTS_API const dSkeletonBone* GetBone(int index) const;

	CUSTOM_JOINTS_API NewtonBody* GetBoneBody(int index) const;
	CUSTOM_JOINTS_API NewtonBody* GetBoneBody(const dSkeletonBone* const bone) const;

	CUSTOM_JOINTS_API const dSkeletonBone* GetParent(const dSkeletonBone* const bone) const;

	CUSTOM_JOINTS_API void LinkCycleBones(dCustomJoint*) const;

	void SetCalculateLocalTransforms(bool val) { m_calculateLocalTransform = val; }
	bool GetCalculateLocalTransforms() const { return m_calculateLocalTransform; }

	protected:
	


	private:
	dSkeletonBone m_bones[D_HIERACHICAL_CONTROLLER_MAX_BONES];
	void* m_collisionAggregate;
	int m_boneCount;
	bool m_calculateLocalTransform;
	
*/

	CUSTOM_JOINTS_API dCustomActiveCharacterController();
	CUSTOM_JOINTS_API ~dCustomActiveCharacterController();

	CUSTOM_JOINTS_API void* GetRoot() const;
	CUSTOM_JOINTS_API void* AddRoot(NewtonBody* const root);
	CUSTOM_JOINTS_API void* AddBone(dCustomJoint* const childJoint, void* const parentBone);

	CUSTOM_JOINTS_API NewtonBody* GetBody(void* const node) const;
	CUSTOM_JOINTS_API dCustomJoint* const childJoint(void* const node) const;

	CUSTOM_JOINTS_API virtual void Finalize ();
	
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

	protected:
	NewtonInverseDynamics* m_kinemativSolver;
	friend class dCustomActiveCharacterManager;
};


class dCustomActiveCharacterManager: public dCustomControllerManager<dCustomActiveCharacterController> 
{
	public:
	CUSTOM_JOINTS_API dCustomActiveCharacterManager(NewtonWorld* const world, const char* const name = DYNAMIC_RAGDOLL_PLUGIN_NAME);
	CUSTOM_JOINTS_API virtual ~dCustomActiveCharacterManager();

	CUSTOM_JOINTS_API virtual void Debug () const;

	CUSTOM_JOINTS_API virtual dCustomActiveCharacterController* CreateTransformController ();
	
//	CUSTOM_JOINTS_API virtual void OnUpdateTransform (const dCustomActiveCharacterController::dSkeletonBone* const bone, const dMatrix& localMatrix) const;
//	CUSTOM_JOINTS_API virtual dCustomArticulatedTransformController* CreateTransformController(void* const userData);
//	CUSTOM_JOINTS_API virtual void DisableAllSelfCollision(dCustomArticulatedTransformController* const controller);
//	CUSTOM_JOINTS_API virtual void SetDefaultSelfCollisionMask(dCustomArticulatedTransformController* const controller);
//	CUSTOM_JOINTS_API virtual void SetCollisionMask(dCustomArticulatedTransformController::dSkeletonBone* const bone0, dCustomArticulatedTransformController::dSkeletonBone* const bone1, bool mode);
//	CUSTOM_JOINTS_API virtual bool SelfCollisionTest(const dCustomArticulatedTransformController::dSkeletonBone* const bone0, const dCustomArticulatedTransformController::dSkeletonBone* const bone1) const;
//	CUSTOM_JOINTS_API virtual void OnUpdateTransform(const dCustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const = 0;
};


#endif 

