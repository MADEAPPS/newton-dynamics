/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndDemoCameraNode.h"
#include "ndDemoEntityManager.h"

ndDemoCameraNode::ndDemoCameraNode(ndRender* const owner)
	:ndRenderSceneNode(ndGetIdentityMatrix())
{
	m_owner = owner;
	ndSharedPtr<ndRenderSceneNode> camera(new ndRenderSceneCamera(owner));
	AddChild(camera);
}

void ndDemoCameraNode::TickUpdate(ndFloat32 timestep)
{
	ndAssert(0);
}
