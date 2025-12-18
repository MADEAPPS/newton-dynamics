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

const ndArray<ndRenderPassDebugLines::ndLine>& ndRenderPassDebugLines::GetVertex() const
{
	return m_debugLines;
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