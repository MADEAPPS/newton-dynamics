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


#ifndef __D_MODEL_NODE_H__
#define __D_MODEL_NODE_H__

#include "dModelStdAfx.h"

class dModelNode;
class dModelChildrenList: public dList<dPointer<dModelNode>>
{
	public:
	dModelChildrenList()
		:dList<dPointer<dModelNode>>()
	{
	}

	~dModelChildrenList()
	{
	}
};

class dModelNode: public dCustomAlloc
{
	public:
	dModelNode(NewtonBody* const modelBody, const dMatrix& bindMatrix, dModelNode* const parent);
	virtual ~dModelNode();

	NewtonBody* GetBody() const { return m_body; }
	const dMatrix& GetBindMatrix() const { return m_bindMatrix; }

	void* GetUserData() const { return m_userData; }
	void SetUserData(void* const data) { m_userData = data; }

	const dModelNode* GetRoot() const;
	dCustomJoint* GetJoint() const { return m_joint; }
	const dModelNode* GetParent() const {return m_parent;}
	dModelChildrenList& GetChildren() {return m_children;}
	const dModelChildrenList& GetChildren() const {return m_children;}

	protected:
	dMatrix m_bindMatrix;
	void* m_userData;
	NewtonBody* m_body;
	dModelNode* m_parent;
	dCustomJoint* m_joint;
	dModelChildrenList m_children;

	friend class dModelManager;
	friend class dModelRootNode;
};



#endif 

