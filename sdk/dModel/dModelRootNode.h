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


#ifndef __D_MODEL_ROOT_NODE_H__
#define __D_MODEL_ROOT_NODE_H__

#include "dModelStdAfx.h"
#include "dModelNode.h"

class dModelManager;

class dModelRootNode: public dModelNode
{
	public:
	dModelRootNode(NewtonBody* const rootBody, const dMatrix& bindMatrix);
	virtual ~dModelRootNode();

	void SetTranformMode(bool localTransform) {m_localTransformMode = localTransform;}

	protected:
	bool m_localTransformMode;
	friend class dModelManager;
};


#endif 

