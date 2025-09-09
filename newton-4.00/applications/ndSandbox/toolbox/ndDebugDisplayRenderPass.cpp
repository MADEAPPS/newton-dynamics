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
	,m_awakeColor(ndFloat32 (1.0f))
	,m_sleepColor(ndFloat32(0.42f), ndFloat32(0.73f), ndFloat32(0.98f), ndFloat32(1.0f))
	,m_manager(owner)
	,m_meshCache()
	,m_collisionDisplayMode(0)
{
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

ndDebugDisplayRenderPass::ndDebugMesh* ndDebugDisplayRenderPass::CreateRenderPrimitive(const ndShapeInstance& shapeInstance) const
{
	ndShapeInstance shape(shapeInstance);
	shape.SetScale(ndVector(1.0f));
	shape.SetLocalMatrix(ndGetIdentityMatrix());

	ndRender* const render = m_owner;
	ndDebugMesh* const debugMesh = new ndDebugMesh;

	debugMesh->m_flatShaded = ndSharedPtr<ndRenderPrimitive> (ndRenderPrimitiveMesh::CreateFromCollisionShape(render, &shape));

	//ndDebugMesh debugMesh;
	//debugMesh.m_flatShaded = new ndFlatShadedDebugMesh(scene->GetShaderCache(), &shape);
	//debugMesh.m_zBufferShaded = new ndZbufferDebugMesh(scene->GetShaderCache(), &shape);
	//debugMesh.m_wireFrameShareEdge = new ndWireFrameDebugMesh(scene->GetShaderCache(), &shape);
	//if (shape.GetShape()->GetAsShapeStaticBVH())
	//{
	//	debugMesh.m_wireFrameOpenEdge = new ndWireFrameDebugMesh(scene->GetShaderCache(), &shape, ndShapeDebugNotify::ndEdgeType::m_open);
	//	debugMesh.m_wireFrameOpenEdge->SetColor(ndVector(1.0f, 0.0f, 1.0f, 1.0f));
	//}
	//shapeNode = m_meshCache.Insert(debugMesh, key);

	return debugMesh;
}

void ndDebugDisplayRenderPass::RenderScene(ndFloat32)
{
	ndPhysicsWorld* const world = m_manager->GetWorld();
	const ndBodyListView& bodyList = world->GetBodyList();

	//ndRenderPrimitiveMeshImplement* const meshReander = m_im
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

		if (m_collisionDisplayMode == 1)
		{
			// render solid color collsion mesh
			const ndMatrix matrix(shapeInstance.GetScaledTransform(body->GetMatrix()));
			ndSharedPtr<ndDebugMesh>& debugMesh = node->GetInfo();
			const ndVector color((body->GetSleepState() == 1) ? m_sleepColor : m_awakeColor);

			//ndRenderPrimitiveMesh* const mesh = (ndRenderPrimitiveMesh*)*debugMesh->m_flatShaded;
			//ndRenderPrimitiveMeshImplement* const implement = *mesh->m_implementation;
			//ndRenderPrimitiveMeshSegment& segment = implement->m_segments.GetFirst()->GetInfo();
			//const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;

			debugMesh->m_flatShaded->Render(m_owner, matrix, m_debugDisplaySolidMesh);
		}
	}
}
