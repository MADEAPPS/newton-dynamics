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

class ndRenderPassDebugLines::ndCallback : public ndConstraintDebugCallback
{
	public:
	ndCallback(ndRenderPassDebugLines* const owner)
		:ndConstraintDebugCallback()
		,m_owner(owner)
	{
	}

	//void DrawPoint(const ndVector& point, const ndVector& color, ndFloat32 thickness) override
	void DrawPoint(const ndVector&, const ndVector&, ndFloat32) override
	{

	}

	//void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness = ndFloat32(1.0f)) override
	void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32) override
	{
		ndLine line;

		line.m_point = p0;
		line.m_color = color;
		m_owner->m_debugLines.PushBack(line);

		line.m_point = p1;
		line.m_color = color;
		m_owner->m_debugLines.PushBack(line);
	}

	ndRenderPassDebugLines* m_owner;
};

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

	ndCallback debugCallback(this);
	debugCallback.SetScale(scale);

	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		const ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		ndMatrix matrix(body->GetMatrix());
		matrix.m_posit = matrix.TransformVector(body->GetCentreOfMass());
		debugCallback.DrawFrame(matrix);
	}
}

void ndRenderPassDebugLines::GenerateJointsDebug()
{
	ndFloat32 scale = ndFloat32(0.25f);

	ndCallback debugCallback(this);
	debugCallback.SetScale(scale);

	const ndJointList& jointList = m_world->GetJointList();
	for (ndJointList::ndNode* node = jointList.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndJointBilateralConstraint>& joint = node->GetInfo();
		joint->DebugJoint(debugCallback);
	}
}

void ndRenderPassDebugLines::GenerateModelsDebug()
{
	ndFloat32 scale = ndFloat32(0.25f);

	ndCallback debugCallback(this);
	debugCallback.SetScale(scale);

	const ndModelList& modelList = m_world->GetModelList();
	for (ndModelList::ndNode* node = modelList.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndModelNotify>& notify = node->GetInfo()->GetNotifyCallback();
		notify->Debug(debugCallback);
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
	if (m_options.m_showJointDebugInfo)
	{
		GenerateJointsDebug();
	}
	if (m_options.m_showModelsDebugInfo)
	{
		GenerateModelsDebug();
	}

	if (m_debugLines.GetCount())
	{
		RenderDebugLines();
	}
}