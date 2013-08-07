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

#ifndef D_CUSTOM_HIERACHICAL_TRANSFORM_MANAGER_H_
#define D_CUSTOM_HIERACHICAL_TRANSFORM_MANAGER_H_

#include "CustomJointLibraryStdAfx.h"
#include "CustomControllerManager.h"

#define D_HIERACHICAL_CONTROLLER_MAX_BONES	64

#define HIERACHICAL_TRANSFORM_PLUGIN_NAME	"__hierachicalTransformManager__"

// a Skeleton Transform controller is use to calculate local transform on contractions of rigid bodies and joint that form part of a hierarchical Skeleton
class CustomHierarchicalTransformController: public CustomControllerBase
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

		dLong m_mask [((D_HIERACHICAL_CONTROLLER_MAX_BONES - 1) / (8 * sizeof (dLong))) + 1];
	};

	class dSkeletonBone
	{
		public: 
		dMatrix m_bindMatrix;
		NewtonBody* m_body;
		dSkeletonBone* m_parent;
		CustomHierarchicalTransformController* m_myController;
		dSelfCollisionBitmask m_bitField;
	};

	NEWTON_API CustomHierarchicalTransformController();
	NEWTON_API ~CustomHierarchicalTransformController();

	NEWTON_API void SetErrorProjectionMode (bool mode);
	NEWTON_API bool GetErrorProjectionMode () const;

	NEWTON_API void DisableAllSelfCollision ();
	NEWTON_API void SetDefaultSelfCollisionMask ();
	NEWTON_API void SetSelfCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1, bool mode);

	NEWTON_API bool SelfCollisionTest (const dSkeletonBone* const bone0, const dSkeletonBone* const bone1) const;
	NEWTON_API dSkeletonBone* AddBone (NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBodne = NULL);
	
	protected:
	NEWTON_API void Init (void* const userData, bool errorCorrectionMode);

	NEWTON_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	NEWTON_API virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	dSkeletonBone m_bones[D_HIERACHICAL_CONTROLLER_MAX_BONES];
	int m_boneCount;
	bool m_errorProjectionMode;
	friend class CustomHierarchicalTransformManager;
};

class CustomHierarchicalTransformManager: public CustomControllerManager<CustomHierarchicalTransformController> 
{
	public:
	NEWTON_API CustomHierarchicalTransformManager(NewtonWorld* const world);
	NEWTON_API virtual ~CustomHierarchicalTransformManager();

	virtual void Debug () const 
	{
	}

	NEWTON_API virtual CustomHierarchicalTransformController* CreateTransformController (void* const userData, bool errorCorrectionMode);
	
	NEWTON_API virtual void DisableAllSelfCollision (CustomHierarchicalTransformController* const controller);
	NEWTON_API virtual void SetDefaultSelfCollisionMask (CustomHierarchicalTransformController* const controller);
	
	NEWTON_API virtual void SetCollisionMask (CustomHierarchicalTransformController::dSkeletonBone* const bone0, CustomHierarchicalTransformController::dSkeletonBone* const bone1, bool mode);
	NEWTON_API virtual bool SelfCollisionTest (const CustomHierarchicalTransformController::dSkeletonBone* const bone0, const CustomHierarchicalTransformController::dSkeletonBone* const bone1) const;

	NEWTON_API virtual void OnPreUpdate (CustomHierarchicalTransformController* const constroller, dFloat timestep, int threadIndex) const = 0;
	NEWTON_API virtual void OnUpdateTransform (const CustomHierarchicalTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const = 0;
};


#endif 

