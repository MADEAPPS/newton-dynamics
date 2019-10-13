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

#define D_MODEL_MANAGER	"__dModelManager__"

class dModelNode;
class dModelRootNode;

class dModelManager: public dCustomParallelListener
{
	public:
	dModelManager(NewtonWorld* const world, const char* const name = D_MODEL_MANAGER);
	virtual ~dModelManager();

	void AddRoot(dModelRootNode* const root);
	void RemoveRoot(dModelRootNode* const root);
	void RemoveAndDeleteRoot(dModelRootNode* const root);

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const {};
	virtual void OnPostUpdate(dModelRootNode* const model, dFloat timestep) const {};
	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const {}
	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext) {}

	protected:
	void PostStep(dFloat timestep, int threadID);
	void PreUpdate(dFloat timestep, int threadID);
	void PostUpdate(dFloat timestep, int threadID);

	private:
	void UpdateLocalTranforms(dModelRootNode* const model) const;
	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext);
	
	dList<dModelRootNode*> m_modelList;
};


#endif 

