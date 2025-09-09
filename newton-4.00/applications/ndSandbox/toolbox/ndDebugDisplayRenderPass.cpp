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
#include "ndDemoEntityManager.h"
#include "ndDebugDisplayRenderPass.h"

ndDebugDisplayRenderPass::ndDebugDisplayRenderPass(ndDemoEntityManager* const owner)
	:ndRenderPass(*owner->GetRenderer())
	,m_manager(owner)
	,m_collisionDisplayMode(0)
{
	m_active = false;
}

ndDebugDisplayRenderPass::~ndDebugDisplayRenderPass()
{
}

void ndDebugDisplayRenderPass::SetDisplayMode(ndInt32 mode)
{
	m_collisionDisplayMode = mode;
	if (!mode)
	{
		m_active = false;
		return;
	}

	m_active = true;
}

void ndDebugDisplayRenderPass::RenderScene(ndFloat32)
{
	//ndUserCallback

	ndPhysicsWorld* const world = m_manager->GetWorld();
	const ndBodyListView& bodyList = world->GetBodyList();

	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		const ndShapeInstance& shapeInstance = body->GetCollisionShape();

		ndShape* const key = (ndShape*)shapeInstance.GetShape();
		if (!(key->GetAsShapeNull() || key->GetAsShapeStaticProceduralMesh()))
		{
			ndAssert(0);
			//const ndMatrix matrix(shapeInstance.GetScaledTransform(body->GetMatrix()));
		}

		if (m_collisionDisplayMode == 3)
		{
			ndAssert(0);
		}
	}
}
