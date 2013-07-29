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
class CustomSkeletonTransformController: public CustomControllerBase_
{
	public:
	class dBitFieldMask
	{
		public: 
		dBitFieldMask()
		{
			memset (m_mask, 0xff, sizeof (m_mask));
		}

		void GetAddress (int id, int& index, int& shift) const
		{
			int bits = 8 * sizeof (long long);
			shift = id & (bits - 1);
			index = id / (bits * sizeof (m_mask) / sizeof (m_mask[0]));
		}


		void SetBit (int id)
		{
			int index;
			int shift;
			GetAddress (id, index, shift);
			long long bit = 1;
			m_mask[index] |= (bit << shift);
		}

		void ResetBit (int id)
		{
			int index;
			int shift;
			GetAddress (id, index, shift);
			long long bit = 1;
			m_mask[index] &= ~(bit << shift);
		}

		bool TestMask (int id) const
		{
			int index;
			int shift;
			GetAddress (id, index, shift);
			long long bit = 1;
			return (m_mask[index] & (bit << shift)) ? true : false;
		}

		long long m_mask [D_SKELETON_CONTROLLER_MAX_BONES / (8 * sizeof (long long))];
	};

	class dSkeletonBone
	{
		public: 
		dMatrix m_bindMatrix;
		NewtonBody* m_body;
		dSkeletonBone* m_parent;
		CustomSkeletonTransformController* m_myController;
		dBitFieldMask m_bitField;
	};


	NEWTON_API void SetUserData(void* const userData);
	NEWTON_API const void* GetUserData() const;

	NEWTON_API dSkeletonBone* AddBone (NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBodne = NULL);
	NEWTON_API void SetDefaultBitFieldMask ();

	NEWTON_API bool TestCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1) const;
	
	protected:
	NEWTON_API void Init (void* const userData);

	NEWTON_API virtual void PreUpdate(dFloat timestep, int threadIndex) {};
	NEWTON_API virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	void* m_usertData;
	int m_boneCount;
	dSkeletonBone m_bones[D_SKELETON_CONTROLLER_MAX_BONES];
	friend class CustomSkeletonTransformManager;
};

class CustomSkeletonTransformManager: public CustomControllerManager_<CustomSkeletonTransformController> 
{
	public:
	NEWTON_API CustomSkeletonTransformManager(NewtonWorld* const world);
	NEWTON_API virtual ~CustomSkeletonTransformManager();

	virtual void PreUpdate(dFloat timestep)
	{
		// bypass the entire Post Update call by not calling the base class
	}
	NEWTON_API virtual void PostUpdate(dFloat timestep);

	NEWTON_API virtual void UpdateTransform (const CustomSkeletonTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const = 0;

	virtual void Debug () const {};
	NEWTON_API virtual CustomSkeletonTransformController* CreateTransformController (void* const userData);
};

inline void CustomSkeletonTransformController::SetUserData(void* const userData)
{
	m_usertData = userData;
}

inline const void* CustomSkeletonTransformController::GetUserData() const
{
	return m_usertData;
}


#endif 

