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
	:ndRenderPassDebug(*owner->GetRenderer(), owner->GetWorld())
	,m_awakeColor(ndFloat32(1.0f))
	,m_sleepColor(ndFloat32(0.42f), ndFloat32(0.73f), ndFloat32(0.98f), ndFloat32(1.0f))
	,m_manager(owner)
	,m_meshCache()
	,m_showCollisionMeshMode(0)
{
}

ndDebugDisplayRenderPass::~ndDebugDisplayRenderPass()
{
}

void ndDebugDisplayRenderPass::SetDebugDisplayOptions()
{
	ndDebugLineOptions options;
	options.m_showBodyAABB = m_manager->m_showAABB;
	options.m_showBroadPhase = m_manager->m_showScene;
	options.m_showBodyFrame = m_manager->m_showBodyFrame;
	options.m_showContacts = m_manager->m_showContactPoints;
	options.m_showCentreOfMass = m_manager->m_showCenterOfMass;
	options.m_showJointDebugInfo = m_manager->m_showJointDebugInfo;
	options.m_showModelsDebugInfo = m_manager->m_showModelsDebugInfo;
	ndRenderPassDebug::SetDebugDisplayOptions(options);

	m_showCollisionMeshMode = m_manager->m_showCollisionMeshMode;
}

void ndDebugDisplayRenderPass::ResetScene()
{
	m_meshCache.RemoveAll();
}

ndDebugDisplayRenderPass::ndDebugMesh* ndDebugDisplayRenderPass::CreateRenderPrimitive(const ndShapeInstance& shapeInstance) const
{
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(shapeInstance));
	shape->SetLocalMatrix(ndGetIdentityMatrix());

	ndRender* const render = m_owner;
	ndDebugMesh* const debugMesh = new ndDebugMesh;

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugFlatShaded;
	debugMesh->m_flatShaded = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugWireFrame;
	debugMesh->m_wireFrameShareEdge = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugHiddenLines;
	debugMesh->m_zBuffer = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	return debugMesh;
}

void ndDebugDisplayRenderPass::RenderCollisionShape()
{
	ndPhysicsWorld* const world = m_manager->GetWorld();
	const ndBodyListView& bodyList = world->GetBodyList();

	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		const ndShapeInstance& shapeInstance = body->GetCollisionShape();

		ndShape* const key = (ndShape*)shapeInstance.GetShape();
		if (key->GetAsShapeNull() || key->GetAsShapeStaticProceduralMesh())
		{
			continue;
		}

		ndTree<ndSharedPtr<ndDebugMesh>, ndShape*>::ndNode* node = m_meshCache.Find(key);
		if (!node)
		{
			ndSharedPtr<ndDebugMesh> debugMesh(CreateRenderPrimitive(shapeInstance));
			node = m_meshCache.Insert(debugMesh, key);
		}

		const ndMatrix matrix(shapeInstance.GetLocalMatrix() * body->GetMatrix());
		ndSharedPtr<ndDebugMesh>& debugMesh = node->GetInfo();
		const ndVector color((body->GetSleepState() == 1) ? m_sleepColor : m_awakeColor);

		switch (m_showCollisionMeshMode)
		{
			case 1:
			{
				// render solid color collision mesh
				ndRenderPrimitive* const mesh = *debugMesh->m_flatShaded;
				ndRenderPrimitiveSegment& segment = mesh->m_segments.GetFirst()->GetInfo();
				ndRenderPrimitiveMaterial* const material = &segment.m_material;
				material->m_diffuse = color;
				debugMesh->m_flatShaded->Render(m_owner, matrix, m_debugDisplaySolidMesh);
				break;
			}

			case 2:
			{
				// render solid color collsion mesh
				ndRenderPrimitive* const mesh = *debugMesh->m_wireFrameShareEdge;
				ndRenderPrimitiveSegment& segment = mesh->m_segments.GetFirst()->GetInfo();
				ndRenderPrimitiveMaterial* const material = &segment.m_material;
				material->m_diffuse = color;
				debugMesh->m_wireFrameShareEdge->Render(m_owner, matrix, m_debugDisplayWireFrameMesh);
				break;
			}

			case 3:
			default:
			{
				debugMesh->m_zBuffer->Render(m_owner, matrix, m_debugDisplaySetZbuffer);
				break;
			}
		}
	}

	if (m_showCollisionMeshMode == 3)
	{
		for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
		{
			ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
			const ndShapeInstance& shapeInstance = body->GetCollisionShape();
			ndShape* const key = (ndShape*)shapeInstance.GetShape();
			if (key->GetAsShapeNull() || key->GetAsShapeStaticProceduralMesh())
			{
				continue;
			}
			ndTree<ndSharedPtr<ndDebugMesh>, ndShape*>::ndNode* const node = m_meshCache.Find(key);
			ndAssert(node);
			const ndMatrix matrix(shapeInstance.GetLocalMatrix() * body->GetMatrix());
			ndSharedPtr<ndDebugMesh>& debugMesh = node->GetInfo();
			const ndVector color((body->GetSleepState() == 1) ? m_sleepColor : m_awakeColor);

			ndRenderPrimitive* const mesh = *debugMesh->m_wireFrameShareEdge;
			ndRenderPrimitiveSegment& segment = mesh->m_segments.GetFirst()->GetInfo();
			ndRenderPrimitiveMaterial* const material = &segment.m_material;
			material->m_diffuse = color;
			debugMesh->m_wireFrameShareEdge->Render(m_owner, matrix, m_debugDisplayWireFrameMesh);
		}
	}
}

void ndDebugDisplayRenderPass::RenderScene()
{
	m_world = m_manager->GetWorld();
	ndRenderPassDebug::RenderScene();

	if (m_showCollisionMeshMode)
	{
		RenderCollisionShape();
	}
}
