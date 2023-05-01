/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndFileFormatStdafx.h"
#include "ndFileFormatSave.h"
#include "ndFileFormatRegistrar.h"

ndFileFormatSave::ndFileFormatSave()
	:ndFileFormat()
	,m_world(nullptr)
	,m_doc(nullptr)
	,m_bodies()
	,m_models()
	,m_joints()
	,m_bodiesIds()
	,m_jointsIds()
	,m_uniqueShapesIds()
{
}

ndFileFormatSave::~ndFileFormatSave()
{
}

ndInt32 ndFileFormatSave::FindBodyId(const ndBody* const body) const
{
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node0 = m_bodiesIds.Find(body->GetId());
	ndAssert(node0);
	return node0 ? node0->GetInfo() : 0;
}

ndInt32 ndFileFormatSave::FindJointId(const ndJointBilateralConstraint* const joint) const
{
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node0 = m_bodiesIds.Find(joint->GetBody0()->GetId());
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node1 = m_bodiesIds.Find(joint->GetBody1()->GetId());
	ndAssert(node0);

	ndInt32 body0NodeId = node0->GetInfo();
	ndInt32 body1NodeId = node1 ? node1->GetInfo() : 0;

	union Key
	{
		ndUnsigned64 m_hash;
		struct
		{
			ndInt32 m_body0;
			ndInt32 m_body1;
		};
	};

	Key key;
	key.m_body0 = body0NodeId;
	key.m_body1 = body1NodeId;
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node = m_jointsIds.Find(key.m_hash);

	return node ? node->GetInfo() : 0;
}

void ndFileFormatSave::CollectScene()
{
	m_joints.SetCount(0);
	m_bodies.SetCount(0);
	m_models.SetCount(0);

	bool saveSentinel = true;
	for (ndJointList::ndNode* node = m_world->GetJointList().GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = *node->GetInfo();
		m_joints.PushBack(joint);
		if (saveSentinel)
		{
			ndBody* const body = node->GetInfo()->GetBody1();
			if (body == m_world->GetSentinelBody())
			{
				saveSentinel = false;
				m_bodies.PushBack(body);
			}
		}
	}

	for (ndBodyListView::ndNode* node = m_world->GetBodyList().GetFirst(); node; node = node->GetNext())
	{
		ndBody* const body = *node->GetInfo();
		m_bodies.PushBack(body);
	}

	for (ndModelList::ndNode* node = m_world->GetModelList().GetFirst(); node; node = node->GetNext())
	{
		ndModel* const model = *node->GetInfo();
		m_models.PushBack(model);
	}
}

void ndFileFormatSave::SaveShapes(nd::TiXmlElement* const rootNode)
{
	// save bodies without compound shapes.
	if (m_bodies.GetCount())
	{
		nd::TiXmlElement* const shapeNode = new nd::TiXmlElement("ndShapes");
		rootNode->LinkEndChild(shapeNode);
		for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
		{
			ndBodyKinematic* const body = m_bodies[i]->GetAsBodyKinematic();
			ndShape* const shape = body->GetCollisionShape().GetShape();
			if (!shape->GetAsShapeCompound())
			{
				ndUnsigned64 hash = shape->GetHash();
				ndTree<ndInt32, ndUnsigned64>::ndNode* const node = m_uniqueShapesIds.Insert(hash);
				if (node)
				{
					ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(shape->ClassName());
					if (!handler)
					{
						ndTrace(("failed to save shape type: %s\n", shape->ClassName()));
						ndAssert(0);
					}

					ndInt32 id = handler->SaveShape(this, shapeNode, shape);
					node->GetInfo() = id;
				}
			}
		}

		// save bodies with compound shapes.
		for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
		{
			ndBodyKinematic* const body = m_bodies[i]->GetAsBodyKinematic();
			ndShape* const shape = body->GetCollisionShape().GetShape();
			if (shape->GetAsShapeCompound())
			{
				ndUnsigned64 hash = shape->GetHash();
				ndTree<ndInt32, ndUnsigned64>::ndNode* const node = m_uniqueShapesIds.Insert(hash);
				if (node)
				{
					ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(shape->ClassName());
					ndInt32 id = handler->SaveShape(this, shapeNode, shape);
					node->GetInfo() = id;
				}
			}
		}
	}
}

void ndFileFormatSave::SaveBodies(nd::TiXmlElement* const rootNode)
{
	if (m_bodies.GetCount())
	{
		nd::TiXmlElement* const bodiesNode = new nd::TiXmlElement("ndBodies");
		rootNode->LinkEndChild(bodiesNode);

		for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
		{
			ndBody* const body = m_bodies[i];
			ndFileFormatRegistrar* handler = ndFileFormatRegistrar::GetHandler(body->ClassName());
			if (!handler)
			{
				ndTrace(("failed to save body type: %s\n", body->ClassName()));
				handler = ndFileFormatRegistrar::GetHandler(body->SuperClassName());
			}
			ndAssert(handler);
			if (handler)
			{
				handler->SaveBody(this, bodiesNode, body);
			}
		}
	}
}

void ndFileFormatSave::SaveJoints(nd::TiXmlElement* const rootNode)
{
	if (m_joints.GetCount())
	{
		nd::TiXmlElement* const jointsNode = new nd::TiXmlElement("ndJoints");
		rootNode->LinkEndChild(jointsNode);

		for (ndInt32 i = 0; i < m_joints.GetCount(); ++i)
		{
			ndJointBilateralConstraint* const joint = m_joints[i];
			ndFileFormatRegistrar* handler = ndFileFormatRegistrar::GetHandler(joint->ClassName());
			if (!handler)
			{
				ndTrace(("failed to save joint type: %s\n", joint->ClassName()));
				handler = ndFileFormatRegistrar::GetHandler(joint->SuperClassName());
			}
			if (handler)
			{
				handler->SaveJoint(this, jointsNode, joint);
			}
		}
	}
}

void ndFileFormatSave::SaveModels(nd::TiXmlElement* const rootNode)
{
	if (m_models.GetCount())
	{
		nd::TiXmlElement* const modelsNode = new nd::TiXmlElement("ndModels");
		rootNode->LinkEndChild(modelsNode);

		for (ndInt32 i = 0; i < m_models.GetCount(); ++i)
		{
			ndModel* const model = m_models[i];
			ndFileFormatRegistrar* handler = ndFileFormatRegistrar::GetHandler(model->ClassName());
			if (!handler)
			{
				ndTrace(("failed to save model type: %s\n", model->ClassName()));
				handler = ndFileFormatRegistrar::GetHandler(model->SuperClassName());
			}
			if (handler)
			{
				handler->SaveModel(this, modelsNode, model);
			}
		}
	}
}

void ndFileFormatSave::SaveWorld(nd::TiXmlElement* const rootNode)
{
	ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(m_world->ClassName());
	ndAssert(handler);
	if (handler)
	{
		nd::TiXmlElement* const worldNode = new nd::TiXmlElement(D_WORD_CLASS);
		rootNode->LinkEndChild(worldNode);
		handler->SaveWorld(this, worldNode, m_world);
	}
}

void ndFileFormatSave::BeginSave(const ndWorld* const world, const char* const path)
{
	xmlResetClassId();

	m_world = (ndWorld*)world;
	SetPath(path);

	m_doc = new nd::TiXmlDocument("ndFile");
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	m_doc->LinkEndChild(decl);
	
	m_oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	m_bodies.SetCount(0);
	m_models.SetCount(0);
	m_joints.SetCount(0);

	m_jointsIds.RemoveAll();
	m_bodiesIds.RemoveAll();
	m_uniqueShapesIds.RemoveAll();
}

void ndFileFormatSave::EndSave()
{
	m_doc->SaveFile(m_path.GetStr());
	setlocale(LC_ALL, m_oldloc.GetStr());

	m_bodies.SetCount(0);
	m_models.SetCount(0);
	m_joints.SetCount(0);

	m_jointsIds.RemoveAll();
	m_bodiesIds.RemoveAll();
	m_uniqueShapesIds.RemoveAll();
	delete m_doc;
}

void ndFileFormatSave::SaveBodies(const ndWorld* const world, const char* const path)
{
	BeginSave(world, path);

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndFile");
	m_doc->LinkEndChild(rootNode);

	CollectScene();
	SaveShapes(rootNode);
	SaveBodies(rootNode);
	
	EndSave();
}

void ndFileFormatSave::SaveWorld(const ndWorld* const world, const char* const path)
{
	BeginSave(world, path);

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndFile");
	m_doc->LinkEndChild(rootNode);

	CollectScene();
	SaveWorld(rootNode);
	SaveShapes(rootNode);
	SaveBodies(rootNode);
	SaveJoints(rootNode);
	SaveModels(rootNode);

	EndSave();
}

void ndFileFormatSave::SaveModels(const ndWorld* const world, const char* const path)
{
	BeginSave(world, path);

	ndTree<ndBody*, ndBody*> bodyFilter;
	for (ndModelList::ndNode* node = world->GetModelList().GetFirst(); node; node = node->GetNext())
	{
		ndModel* const baseModel = node->GetInfo()->GetAsModel();
		m_models.PushBack(baseModel);
		if (baseModel && baseModel->GetAsModelArticulation())
		{
			ndModelArticulation* const model = baseModel->GetAsModelArticulation();

			ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
			if (model->GetRoot())
			{
				stack.PushBack(model->GetRoot());
				while (stack.GetCount())
				{
					ndInt32 index = stack.GetCount() - 1;
					ndModelArticulation::ndNode* const limbNode = stack[index];
					stack.SetCount(index);
					m_bodies.PushBack(*limbNode->m_body);
					bodyFilter.Insert(*limbNode->m_body, *limbNode->m_body);
					if (*limbNode->m_joint)
					{
						m_joints.PushBack(limbNode->m_joint->GetAsBilateral());
					}

					for (ndModelArticulation::ndNode* child = limbNode->GetFirstChild(); child; child = child->GetNext())
					{
						stack.PushBack(child);
					}
				}
			}

			for (ndSharedList<ndJointBilateralConstraint>::ndNode* loopNode = model->m_closeLoops.GetFirst(); loopNode; loopNode = loopNode->GetNext())
			{
				m_joints.PushBack(loopNode->GetInfo()->GetAsBilateral());
			}
		}
	}

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndFile");
	m_doc->LinkEndChild(rootNode);

	SaveWorld(rootNode);
	SaveShapes(rootNode);
	SaveBodies(rootNode);
	SaveJoints(rootNode);
	SaveModels(rootNode);

	EndSave();
}

