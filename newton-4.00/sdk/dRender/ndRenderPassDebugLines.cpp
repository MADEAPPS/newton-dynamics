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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderTexture.h"
#include "ndRenderSceneNode.h"
#include "ndRenderPrimitive.h"
#include "ndRenderPassDebugLines.h"

#if 0
ndRenderPassDebugLines::ndRenderPassDebugLines(ndDemoEntityManager* const owner)
	:ndRenderPass(*owner->GetRenderer())
	,m_awakeColor(ndFloat32(1.0f))
	,m_sleepColor(ndFloat32(0.42f), ndFloat32(0.73f), ndFloat32(0.98f), ndFloat32(1.0f))
	,m_manager(owner)
	,m_meshCache()
	,m_showCollisionMeshMode(0)
	,m_showCenterOfMass(false)
{
	// make the render mesh
	ndRender* const render = m_owner;
	ndRenderPrimitive::ndDescriptor descriptor(render);

	m_owner->m_cachedDebugLinePass = this;
	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugLineArray;
	m_renderLinesPrimitive = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));
}

void ndRenderPassDebugLines::SetDebugDisplayOptions()
{
	m_showCenterOfMass = m_manager->m_showCenterOfMass;
	m_showCollisionMeshMode = m_manager->m_showCollisionMeshMode;
}

void ndRenderPassDebugLines::ResetScene()
{
	m_meshCache.RemoveAll();
}

ndRenderPassDebugLines::ndDebugMesh* ndRenderPassDebugLines::CreateRenderPrimitive(const ndShapeInstance& shapeInstance) const
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

void ndRenderPassDebugLines::RenderCollisionShape()
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

void ndRenderPassDebugLines::GenerateCenterOfMass()
{
	ndFloat32 scale = ndFloat32(0.25f);

	ndPhysicsWorld* const world = m_manager->GetWorld();
	const ndBodyListView& bodyList = world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		const ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		const ndMatrix matrix(body->GetMatrix());
		ndVector com(matrix.TransformVector(body->GetCentreOfMass()));
		
		for (ndInt32 i = 0; i < 3; ++i)
		{
			ndLine line;
			line.m_p0 = com;
			line.m_color0 = ndVector::m_wOne;
			line.m_color0[i] = ndFloat32(1.0f);
			line.m_color1 = line.m_color0;
			line.m_p1 = com + matrix[i].Scale(scale);
			m_debugLines.PushBack(line);
		}
	}
}

void ndRenderPassDebugLines::RenderDebugLines()
{
	const ndMatrix matrix(ndGetIdentityMatrix());
	m_renderLinesPrimitive->Render(m_owner, matrix, m_debugLineArray);
}

void ndRenderPassDebugLines::RenderScene()
{
	if (m_showCollisionMeshMode)
	{
		RenderCollisionShape();
	}

	m_debugLines.SetCount(0);
	if (m_showCenterOfMass)
	{
		GenerateCenterOfMass();
	}

	if (m_debugLines.GetCount())
	{
		RenderDebugLines();
	}
}
#endif

ndRenderPassDebugLines::ndRenderPassDebugLines(ndRender* const owner, ndWorld* const world)
	:ndRenderPass(owner)
	,m_world(world)
{
	ndRender* const render = m_owner;
	m_owner->m_cachedDebugLinePass = this;
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugLineArray;
	m_renderLinesPrimitive = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	memset(&m_options, 0, sizeof(ndDebugLineOptions));
}

ndRenderPassDebugLines::~ndRenderPassDebugLines()
{
	m_owner->m_cachedDebugLinePass = nullptr;
}

void ndRenderPassDebugLines::SetDebugDisplayOptions(const ndDebugLineOptions& options)
{
	m_options = options;
}

void ndRenderPassDebugLines::GenerateCenterOfMass()
{
	ndFloat32 scale = ndFloat32(0.25f);

	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		const ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		const ndMatrix matrix(body->GetMatrix());
		ndVector com(matrix.TransformVector(body->GetCentreOfMass()));
	
		for (ndInt32 i = 0; i < 3; ++i)
		{
			ndLine line;
			line.m_point = com;
			line.m_color = ndVector::m_wOne;
			line.m_color[i] = ndFloat32(1.0f);
			m_debugLines.PushBack(line);

			line.m_point += matrix[i].Scale(scale);
			m_debugLines.PushBack(line);
		}
	}
}

void ndRenderPassDebugLines::RenderDebugLines()
{
	const ndMatrix matrix(ndGetIdentityMatrix());
	m_renderLinesPrimitive->Render(m_owner, matrix, m_debugLineArray);
}

void ndRenderPassDebugLines::RenderScene()
{
	ndAssert(m_world);

	m_debugLines.SetCount(0);
	if (m_options.m_showCentreOfMass)
	{
		GenerateCenterOfMass();
	}
	
	if (m_debugLines.GetCount())
	{
		RenderDebugLines();
	}
}