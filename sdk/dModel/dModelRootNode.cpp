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

#include "dModelStdAfx.h"
#include "dModelManager.h"
#include "dModelRootNode.h"

dModelRootNode::dModelRootNode(NewtonBody* const rootBody, const dMatrix& bindMatrix)
	:dModelNode(rootBody, bindMatrix, NULL)
	,m_localTransformMode(false)
{
}

dModelRootNode::~dModelRootNode()
{
}

