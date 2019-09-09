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

class dModelNode
{
	public:
	dModelNode()
		:m_bindMatrix(dGetIdentityMatrix())
		,m_body(NULL)
		,m_parent(NULL)
		,m_children()
	{
	}

	virtual ~dModelNode()
	{
	}

	NewtonBody* GetBody() const { return m_body; }
	const dMatrix& GetBindMatrix() const { return m_bindMatrix; }

	void* GetUserData() const { return m_userData; }
	void SetUserData(void* const data) { m_userData = data; }

	dCustomJoint* GetParentJoint() const;

	protected:
	dMatrix m_bindMatrix;
	NewtonBody* m_body;
	dModelNode* m_parent;
	void* m_userData;
	dList<dPointer<dModelNode> > m_children;

	friend class dCustomTransformController;
};



#endif 

