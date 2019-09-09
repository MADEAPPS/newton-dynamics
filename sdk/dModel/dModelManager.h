/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __D_MODEL_MANAGER_H__
#define __D_MODEL_MANAGER_H__

#include "dModelStdAfx.h"
#include "dModelRootNode.h"

#define D_MODEL_MANAGER	"__dModelManager__"


class dModelManager: public dCustomParallelListener
{
	public:
	dModelManager(NewtonWorld* const world, const char* const name = D_MODEL_MANAGER);
	virtual ~dModelManager();

	void AddRoot(dModelRootNode* const root);


	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const = 0;

	//dAnimationJointRoot* CreateModel(NewtonBody* const bone, const dMatrix& bindMatrix);
//	void AddModel(dAnimationJointRoot* const model);
//	void RemoveModel(dAnimationJointRoot* const model);

//	dAnimationJoint* GetFirstJoint(const dAnimationJointRoot* const model) const;
//	dAnimationJoint* GetNextJoint(const dAnimationJoint* const joint) const;

	//virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext) = 0;
	

//	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext);
//	virtual void OnPreUpdate(dCustomTransformController* const controller, dFloat timestep, int threadIndex) const;
//	virtual void OnUpdateTransform(const dCustomTransformController::dModelNode* const bone, const dMatrix& localMatrix) const;

	protected:
//	virtual void OnDestroy();
//	virtual void OnPostUpdate(dAnimationJointRoot* const model, dFloat timestep) {}

	void PostUpdate(dFloat timestep, int threadID);
//	void PreUpdate(dFloat timestep, int threadID);
//	dAnimationJoint* GetFirstJoint(const dAnimationJoint* const joint) const;
	private:

	private:
	dList<dPointer<dModelRootNode>> m_controllerList;
	dFloat m_timestep;
};


#endif 

