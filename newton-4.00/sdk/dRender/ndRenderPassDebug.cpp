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
#include "ndRenderPassDebug.h"
#include "ndRenderSceneNode.h"
#include "ndRenderPrimitive.h"

class ndRenderPassDebug::ndCallback : public ndConstraintDebugCallback
{
	public:
	ndCallback(ndRenderPassDebug* const owner)
		:ndConstraintDebugCallback()
		,m_owner(owner)
	{
	}

	//void DrawPoint(const ndVector& point, const ndVector& color, ndFloat32 thickness) override
	void DrawPoint(const ndVector&, const ndVector&, ndFloat32) override
	{
		ndAssert(0);
	}

	//void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness = ndFloat32(1.0f)) override
	void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32) override
	{
		ndPoint line;

		line.m_point = p0;
		line.m_color = color;
		m_owner->m_debugLines.PushBack(line);

		line.m_point = p1;
		line.m_color = color;
		m_owner->m_debugLines.PushBack(line);
	}

	void DrawBox(const ndVector& p0, const ndVector& p1, const ndVector& color)
	{
		ndVector box[12][2];
		box[0][0] = ndVector(p0.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
		box[0][1] = ndVector(p1.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));

		box[1][0] = ndVector(p0.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));
		box[1][1] = ndVector(p1.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));

		box[2][0] = ndVector(p0.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));
		box[2][1] = ndVector(p1.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

		box[3][0] = ndVector(p0.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));
		box[3][1] = ndVector(p1.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));

		box[4][0] = ndVector(p0.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
		box[4][1] = ndVector(p0.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));

		box[5][0] = ndVector(p1.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
		box[5][1] = ndVector(p1.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));

		box[6][0] = ndVector(p0.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));
		box[6][1] = ndVector(p0.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

		box[7][0] = ndVector(p1.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));
		box[7][1] = ndVector(p1.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

		box[8][0] = ndVector(p0.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
		box[8][1] = ndVector(p0.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));

		box[9][0] = ndVector(p1.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
		box[9][1] = ndVector(p1.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));

		box[10][0] = ndVector(p0.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));
		box[10][1] = ndVector(p0.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

		box[11][0] = ndVector(p1.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));
		box[11][1] = ndVector(p1.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

		for (ndInt32 i = 0; i < 12; ++i)
		{
			DrawLine(box[i][0], box[i][1], color, ndFloat32(1.0f));
		}
	}

	ndRenderPassDebug* m_owner;
};

ndRenderPassDebug::ndRenderPassDebug(ndRender* const owner, ndWorld* const world)
	:ndRenderPass(owner)
	,m_world(world)
{
	ndRender* const render = m_owner;
	m_owner->m_cachedDebugPass = this;
	ndRenderPrimitive::ndDescriptor descriptor(render);

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugPointArray;
	m_renderPointsPrimitive = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugLineArray;
	m_renderLinesPrimitive = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));
}

ndRenderPassDebug::~ndRenderPassDebug()
{
	m_owner->m_cachedDebugPass = nullptr;
}

const ndArray<ndRenderPassDebug::ndPoint>& ndRenderPassDebug::GetPoints() const
{
	return m_debugPoints;
}

const ndArray<ndRenderPassDebug::ndPoint>& ndRenderPassDebug::GetVertex() const
{
	return m_debugLines;
}

void ndRenderPassDebug::SetDebugDisplayOptions(const ndDebugLineOptions& options)
{
	m_options = options;
}

void ndRenderPassDebug::GenerateBodyAABB()
{
	ndCallback debugCallback(this);

	ndVector color(ndVector::m_wOne);
	color.m_z = ndFloat32(1.0f);
	debugCallback.SetScale(1.0f);
	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndVector p0;
		ndVector p1;
		const ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		body->GetAABB(p0, p1);
		debugCallback.DrawBox(p0, p1, color);
	}
}

void ndRenderPassDebug::GenerateBodyFrames()
{
	ndFloat32 scale = ndFloat32(0.25f);

	ndCallback debugCallback(this);
	debugCallback.SetScale(scale);

	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		const ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		ndMatrix matrix(body->GetMatrix());
		debugCallback.DrawFrame(matrix);
	}
}

void ndRenderPassDebug::GenerateCenterOfMass()
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

void ndRenderPassDebug::GenerateJointsDebug()
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

void ndRenderPassDebug::GenerateModelsDebug()
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

void ndRenderPassDebug::GenerateBroadphase()
{
	class ndDrawScene : public ndSceneTreeNotiFy, public ndCallback
	{
		public:
		ndDrawScene(ndRenderPassDebug* const owner)
			:ndSceneTreeNotiFy()
			,ndCallback(owner)
			,m_color(ndVector::m_wOne)
		{
			m_color.m_x = ndFloat32(0.75f);
			m_color.m_y = ndFloat32(0.75f);
			m_color.m_z = ndFloat32(0.75f);
		}

		virtual void OnDebugNode(const ndBvhNode* const node)
		{
			ndVector p0;
			ndVector p1;
			node->GetAabb(p0, p1);
			DrawBox(p0, p1, m_color);
		}
		
		ndVector m_color;
	};

	ndDrawScene drawBroaphase(this);
	m_world->DebugScene(&drawBroaphase);
}

void ndRenderPassDebug::GenerateContacts()
{
	ndVector color(ndVector::m_wOne);
	color.m_x = ndFloat32(1.0f);
	const ndContactArray& contactList = m_world->GetContactList();
	for (ndInt32 i = 0; i < contactList.GetCount(); ++i)
	{
		const ndContact* const contact = contactList[i];
		if (contact->IsActive())
		{
			const ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
			{
				const ndContactPoint& contactPoint = contactPointsNode->GetInfo();
				ndPoint point;
				point.m_point = contactPoint.m_point;
				point.m_color = color;
				m_debugPoints.PushBack(point);
			}
		}
	}
}

void ndRenderPassDebug::GenerateContactForce()
{
	ndVector forceColor(ndVector::m_wOne);
	ndVector contactColor(ndVector::m_wOne);

	ndFloat32 forceScale(ndFloat32(0.01f));
	forceColor.m_x = ndFloat32(1.0f);
	contactColor.m_x = ndFloat32(1.0f);
	contactColor.m_y = ndFloat32(1.0f);
	const ndContactArray& contactList = m_world->GetContactList();
	for (ndInt32 i = 0; i < contactList.GetCount(); ++i)
	{
		const ndContact* const contact = contactList[i];
		if (contact->IsActive())
		{
			const ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
			{
				const ndContactMaterial& contactPoint = contactPointsNode->GetInfo();
				ndPoint point;
				point.m_point = contactPoint.m_point;
				point.m_color = contactColor;
				m_debugPoints.PushBack(point);

				point.m_color = forceColor;
				m_debugLines.PushBack(point);
				point.m_point += contactPoint.m_normal.Scale (contactPoint.m_normal_Force.m_force * forceScale);
				m_debugLines.PushBack(point);
			}
		}
	}
}

void ndRenderPassDebug::RenderScene()
{
	ndAssert(m_world);

	m_debugLines.SetCount(0);
	m_debugPoints.SetCount(0);

	if (m_options.m_showContacts)
	{
		GenerateContacts();
	}
	if (m_options.m_showContactsForce)
	{
		GenerateContactForce();
	}
	if (m_options.m_showBroadPhase)
	{
		GenerateBroadphase();
	}
	if (m_options.m_showBodyAABB)
	{
		GenerateBodyAABB();
	}
	if (m_options.m_showBodyFrame)
	{
		GenerateBodyFrames();
	}
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

	if (m_debugPoints.GetCount())
	{
		const ndMatrix matrix(ndGetIdentityMatrix());
		m_renderPointsPrimitive->Render(m_owner, matrix, m_debugPointArray);
	}

	if (m_debugLines.GetCount())
	{
		const ndMatrix matrix(ndGetIdentityMatrix());
		m_renderLinesPrimitive->Render(m_owner, matrix, m_debugLineArray);
	}
}