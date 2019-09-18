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

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext) {}
	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const = 0;

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const {};
	virtual void OnPostUpdate(dModelRootNode* const model, dFloat timestep) const {};

	protected:
	void PreUpdate(dFloat timestep, int threadID);
	void PostUpdate(dFloat timestep, int threadID);

	private:
	void UpdateLocalTranforms(dModelRootNode* const model) const;
	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext);
	

	dList<dPointer<dModelRootNode>> m_controllerList;
	dFloat m_timestep;
};


#endif 

