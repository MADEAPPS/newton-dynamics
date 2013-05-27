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
class CustomSkeletonTransformController
{
	CUSTOM_CONTROLLER_GLUE(CustomSkeletonTransformController);

	class dBitFieldMask
	{
		public: 
		dBitFieldMask()
		{
			memset (m_mask, 0xff, sizeof (m_mask));
			//memset (m_mask, 0, sizeof (m_mask));
		}
		long long m_mask [D_SKELETON_CONTROLLER_MAX_BONES / (8 * sizeof (long long))];
	};

	class dSkeletonBone
	{
		public: 
		NewtonBody* m_body;
		int m_boneIndex;
		int m_parentIndex;
		dBitFieldMask m_bitField;
	};

	public:
	void SetUserData(void* const userData);
	const void* GetUserData() const;

	int AddBone (NewtonBody* const bone, int parentIndex = -1);
	void SetDefaultBitFieldMask ();

	bool TestCollisionMask () const;
	
	protected:
	void Init (void* const userData);

	virtual void PreUpdate(dFloat timestep, int threadIndex) {};
	virtual void PostUpdate(dFloat timestep, int threadIndex);
	
	private:
	void* m_usertData;
	int m_boneCount;
	dSkeletonBone m_bones[D_SKELETON_CONTROLLER_MAX_BONES];
	friend class CustomSkeletonTransformManager;
};

class CustomSkeletonTransformManager: public CustomControllerManager<CustomSkeletonTransformController> 
{
	public:
	CustomSkeletonTransformManager(NewtonWorld* const world);
	virtual ~CustomSkeletonTransformManager();

	virtual void PreUpdate(dFloat timestep)
	{
		// bypass the entire Post Update call by not calling the base class
	}
	virtual void PostUpdate(dFloat timestep);

	virtual void Debug () const {};
	virtual CustomSkeletonTransformController* CreateTransformController (void* const userData);
//	virtual void ApplyBoneTransform (const CustomSkeletonTransformController* const me, TriggerEventType eventType, NewtonBody* const visitor) const = 0;
//	unsigned m_lru;
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

