/* Copyright (c) <2009> <Newton Game Dynamics>
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

#ifndef D_CUSTOM_SKELETON_TRANSFORM_MANAGER_H_
#define D_CUSTOM_SKELETON_TRANSFORM_MANAGER_H_

#include "CustomJointLibraryStdAfx.h"
#include "CustomControllerManager.h"

#define D_SKELETON_CONTROLLER_MAX_BONES	64

#define SKELETON_TRANSFORM_PLUGIN_NAME	"skeletonTransformManager"

// a Skeleton Transform controller is use to calculate local transform on contractions of rigid bodies and joint that form part of a hierarchical Skeleton
class CustomSkeletonTransformController: public CustomControllerBase
{
	public:
	class dSelfCollisionBitmask
	{
		public: 
		dSelfCollisionBitmask()
		{
			memset (m_mask, 0xff, sizeof (m_mask));
		}

		void GetAddress (int id, int& index, int& shift) const
		{
			int bits = 8 * sizeof (dLong);
			shift = id & (bits - 1);
			index = id / (bits * sizeof (m_mask) / sizeof (m_mask[0]));
		}


		void SetBit (int id)
		{
			int index;
			int shift;
			GetAddress (id, index, shift);
			dLong bit = 1;
			m_mask[index] |= (bit << shift);
		}

		void ResetBit (int id)
		{
			int index;
			int shift;
			GetAddress (id, index, shift);
			dLong bit = 1;
			m_mask[index] &= ~(bit << shift);
		}

		bool TestMask (int id) const
		{
			int index;
			int shift;
			GetAddress (id, index, shift);
			dLong bit = 1;
			return (m_mask[index] & (bit << shift)) ? true : false;
		}

		dLong m_mask [((D_SKELETON_CONTROLLER_MAX_BONES - 1) / (8 * sizeof (dLong))) + 1];
	};

	class dSkeletonBone
	{
		public: 
		dMatrix m_bindMatrix;
		NewtonBody* m_body;
		dSkeletonBone* m_parent;
		CustomSkeletonTransformController* m_myController;
		dSelfCollisionBitmask m_bitField;
	};

	NEWTON_API CustomSkeletonTransformController();
	NEWTON_API ~CustomSkeletonTransformController();

	NEWTON_API void DisableAllSelfCollision ();
	NEWTON_API void SetDefaultSelfCollisionMask ();
	NEWTON_API void SetSelfCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1, bool mode);

	NEWTON_API bool SelfCollisionTest (const dSkeletonBone* const bone0, const dSkeletonBone* const bone1) const;
	NEWTON_API dSkeletonBone* AddBone (NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBodne = NULL);
	
	protected:
	NEWTON_API void Init (void* const userData);

	NEWTON_API virtual void PreUpdate(dFloat timestep, int threadIndex) {};
	NEWTON_API virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	int m_boneCount;
	dSkeletonBone m_bones[D_SKELETON_CONTROLLER_MAX_BONES];
	friend class CustomSkeletonTransformManager;
};

class CustomSkeletonTransformManager: public CustomControllerManager<CustomSkeletonTransformController> 
{
	public:
	NEWTON_API CustomSkeletonTransformManager(NewtonWorld* const world);
	NEWTON_API virtual ~CustomSkeletonTransformManager();

	virtual void Debug () const 
	{
	}

	virtual void PreUpdate(dFloat timestep)
	{
		// bypass the entire Post Update call by not calling the base class
	}
	NEWTON_API virtual void PostUpdate(dFloat timestep);

	
	NEWTON_API virtual void DisableAllSelfCollision (CustomSkeletonTransformController* const controller);
	NEWTON_API virtual void SetDefaultSelfCollisionMask (CustomSkeletonTransformController* const controller);
	
	NEWTON_API virtual void SetCollisionMask (CustomSkeletonTransformController::dSkeletonBone* const bone0, CustomSkeletonTransformController::dSkeletonBone* const bone1, bool mode);
	NEWTON_API virtual bool SelfCollisionTest (const CustomSkeletonTransformController::dSkeletonBone* const bone0, const CustomSkeletonTransformController::dSkeletonBone* const bone1) const;
	
	NEWTON_API virtual void UpdateTransform (const CustomSkeletonTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const = 0;
	NEWTON_API virtual CustomSkeletonTransformController* CreateTransformController (void* const userData);
};


#endif 

