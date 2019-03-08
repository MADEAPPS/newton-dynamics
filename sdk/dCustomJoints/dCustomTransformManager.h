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

#ifndef D_CUSTOM_ARTICULATED_TRANSFORM_MANAGER_H_
#define D_CUSTOM_ARTICULATED_TRANSFORM_MANAGER_H_

#include "dCustomJointLibraryStdAfx.h"
//#include "dCustomControllerManager.h"
#include "dCustomListener.h"


#define HIERACHICAL_ARTICULATED_PLUGIN_NAME	"__articulatedTransformManager__"
/*
// a Skeleton Transform controller is use to calculate local transform on contractions of rigid bodies and joint that form part of a hierarchical Skeleton
class dCustomTransformController: public dCustomControllerBase
{
	public:
	class dSkeletonBone: public dList<dSkeletonBone>
	{
		public: 
		dMatrix m_bindMatrix;
		NewtonBody* m_body;
		dSkeletonBone* m_parent;
		dCustomTransformController* m_controller;

		CUSTOM_JOINTS_API dCustomJoint* FindJoint() const; 
	};

	CUSTOM_JOINTS_API dCustomTransformController();
	CUSTOM_JOINTS_API ~dCustomTransformController();

	CUSTOM_JOINTS_API dSkeletonBone* GetRoot () const;
	CUSTOM_JOINTS_API dSkeletonBone* AddRoot (NewtonBody* const bone, const dMatrix& bindMatrix);
	CUSTOM_JOINTS_API dSkeletonBone* AddBone (NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBone);

	CUSTOM_JOINTS_API void SetSelfCollision(bool selfCollsion);

	void SetCalculateLocalTransforms (bool val) {m_calculateLocalTransform = val;}
	bool GetCalculateLocalTransforms () const {return m_calculateLocalTransform;}
	
	protected:
	CUSTOM_JOINTS_API void Init ();

	
	
	private:
//	dList<dSkeletonBone> m_bones;
	dSkeletonBone m_root;
	void* m_collisionAggregate;
	bool m_calculateLocalTransform;
	friend class dCustomTransformManager;
};
*/


class dSkeletonBone: public dList<dSkeletonBone>
{
	public:
	dSkeletonBone()
		:dList<dSkeletonBone>()
	{
		dAssert(0);
	}

	~dSkeletonBone()
	{
		dAssert(0);
	}

	dMatrix m_bindMatrix;
	NewtonBody* m_body;
	dSkeletonBone* m_parent;
//	dCustomTransformController* m_controller;
	CUSTOM_JOINTS_API dCustomJoint* FindJoint() const;
};

class dCustomTransformController: public dSkeletonBone
{
	public:
	dCustomTransformController()
	{
		dAssert (0);
	}

	~dCustomTransformController()
	{
		dAssert (0);
	}
};

//class dCustomTransformManager: public dCustomControllerManager<dCustomTransformController> 
class dCustomTransformManager: public dCustomListener
{
	public:
	CUSTOM_JOINTS_API dCustomTransformManager(NewtonWorld* const world, const char* const name = HIERACHICAL_ARTICULATED_PLUGIN_NAME);
	CUSTOM_JOINTS_API virtual ~dCustomTransformManager();

	CUSTOM_JOINTS_API virtual dCustomTransformController* CreateTransformController (NewtonBody* const bone, const dMatrix& bindMatrix);

	CUSTOM_JOINTS_API virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext) = 0;
	CUSTOM_JOINTS_API virtual void OnPreUpdate (dCustomTransformController* const controller, dFloat timestep, int threadIndex) const = 0;
	CUSTOM_JOINTS_API virtual void OnUpdateTransform (const dCustomTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const = 0;

	protected:
	CUSTOM_JOINTS_API virtual void OnDestroy();
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep);

	private: 
//	friend class dCustomTransformController;
	dList<dCustomTransformController> m_controllersList;
};


#endif 

