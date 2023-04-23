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
#include "ndFileFormatLoad.h"
#include "ndFileFormatRegistrar.h"

ndFileFormatLoad::ndFileFormatLoad()
	:ndFileFormat()
	,m_bodies()
	,m_joints()
{
}

ndFileFormatLoad::~ndFileFormatLoad()
{
}

const ndList<ndSharedPtr<ndBody>>& ndFileFormatLoad::GetBodyList() const
{
	return m_bodies;
}

void ndFileFormatLoad::LoadShapes(const nd::TiXmlElement* const rootNode, ndTree<ndShape*, ndInt32>& shapeMap)
{
	const nd::TiXmlNode* const shapes = rootNode->FirstChild("ndShapes");
	if (shapes)
	{
		for (const nd::TiXmlNode* node = shapes->FirstChild(); node; node = node->NextSibling())
		{
			const nd::TiXmlElement* const element = (nd::TiXmlElement*)node;
			const char* const className = element->Attribute("className");
			ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(className);
			ndAssert(handler);

			ndInt32 nodeId;
			element->Attribute("nodeId", &nodeId);
			ndShape* const shape = handler->LoadShape(element, shapeMap);
			shape->AddRef();
			shapeMap.Insert(shape, nodeId);
		}
	}
}

void ndFileFormatLoad::LoadBodies(const nd::TiXmlElement* const rootNode, const ndTree<ndShape*, ndInt32>& shapeMap, ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	const nd::TiXmlNode* const bodies = rootNode->FirstChild("ndBodies");
	if (bodies)
	{
		for (const nd::TiXmlNode* node = bodies->FirstChild(); node; node = node->NextSibling())
		{
			nd::TiXmlNode* bodyNode = (nd::TiXmlNode*)node;
			while (bodyNode && !ndFileFormatRegistrar::GetHandler(((nd::TiXmlElement*)bodyNode)->Attribute("className")))
			{
				bodyNode = (nd::TiXmlNode*)bodyNode->FirstChild(D_BODY_CLASS);
			}

			ndAssert(bodyNode);
			if (bodyNode)
			{
				const nd::TiXmlElement* const element = (nd::TiXmlElement*)bodyNode;
				ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(element->Attribute("className"));
				ndSharedPtr<ndBody> body(handler->LoadBody(element, shapeMap));

				const nd::TiXmlNode* alliasNode = node;
				do {
					ndInt32 nodeId;
					nd::TiXmlElement* const alliasElement = (nd::TiXmlElement*)alliasNode;
					alliasElement->Attribute("nodeId", &nodeId);
					bodyMap.Insert(body, nodeId);
					alliasNode = alliasNode->FirstChild();
				} while (!strcmp(alliasNode->Value(), D_BODY_CLASS));

				m_bodies.Append(body);
			}
		}
	}
}

void ndFileFormatLoad::LoadJoints(const nd::TiXmlElement* const rootNode, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap)
{
	const nd::TiXmlNode* const joints = rootNode->FirstChild("ndJoints");
	if (joints)
	{
		for (const nd::TiXmlNode* node = joints->FirstChild(); node; node = node->NextSibling())
		{
			nd::TiXmlNode* jointNode = (nd::TiXmlNode*)node;
			while (jointNode && !ndFileFormatRegistrar::GetHandler(((nd::TiXmlElement*)jointNode)->Attribute("className")))
			{
				jointNode = (nd::TiXmlNode*)jointNode->FirstChild(D_JOINT_CLASS);
			}

			ndAssert(jointNode);
			if (jointNode)
			{
				const nd::TiXmlElement* const element = (nd::TiXmlElement*)jointNode;
				ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(element->Attribute("className"));
				ndSharedPtr<ndJointBilateralConstraint> joint (handler->LoadJoint(element, bodyMap));

				const nd::TiXmlNode* alliasNode = node;
				do {
					ndInt32 nodeId;
					nd::TiXmlElement* const alliasElement = (nd::TiXmlElement*)alliasNode;
					alliasElement->Attribute("nodeId", &nodeId);
					jointMap.Insert(joint, nodeId);
					alliasNode = alliasNode->FirstChild();
				} while (!strcmp(alliasNode->Value(), D_JOINT_CLASS));

				m_joints.Append(joint);
			}
		}
	}
}

void ndFileFormatLoad::LoadModels(const nd::TiXmlElement* const rootNode, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap)
{
	const nd::TiXmlNode* const models = rootNode->FirstChild("ndModels");
	if (models)
	{
		for (const nd::TiXmlNode* node = models->FirstChild(); node; node = node->NextSibling())
		{
			nd::TiXmlNode* modelNode = (nd::TiXmlNode*)node;
			while (modelNode && !ndFileFormatRegistrar::GetHandler(((nd::TiXmlElement*)modelNode)->Attribute("className")))
			{
				modelNode = (nd::TiXmlNode*)modelNode->FirstChild("D_MODEL_CLASS");
			}
		
			ndAssert(modelNode);
			if (modelNode)
			{
				const nd::TiXmlElement* const element = (nd::TiXmlElement*)modelNode;
				ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(element->Attribute("className"));
				ndSharedPtr<ndModel> model(handler->LoadModel(element, bodyMap, jointMap));
				m_models.Append(model);
			}
		}
	}
}

void ndFileFormatLoad::Load(const char* const path)
{
	// save the path for use with generated assets.
	SetPath(path);

	m_oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	nd::TiXmlDocument doc(m_path.GetStr());
	doc.LoadFile();

	if (doc.Error())
	{
		setlocale(LC_ALL, m_oldloc.GetStr());
		return;
	}
	ndAssert(!doc.Error());
	
	if (!doc.FirstChild("ndFile"))
	{
		setlocale(LC_ALL, m_oldloc.GetStr());
		return;
	}
	const nd::TiXmlElement* const rootNode = doc.RootElement();

	ndTree<ndShape*, ndInt32> shapeMap;
	ndTree<ndSharedPtr<ndBody>, ndInt32> bodyMap;
	ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32> jointMap;

	m_bodies.RemoveAll();
	m_joints.RemoveAll();

	LoadShapes(rootNode, shapeMap);
	LoadBodies(rootNode, shapeMap, bodyMap);
	LoadJoints(rootNode, bodyMap, jointMap);
	LoadModels(rootNode, bodyMap, jointMap);

	ndTree<ndShape*, ndInt32>::Iterator it (shapeMap);
	for (it.Begin(); it; it++)
	{
		ndTree<ndShape*, ndInt32>::ndNode* node = it.GetNode();
		ndShape* const shape = node->GetInfo();
		shape->Release();
	}
	
	setlocale(LC_ALL, m_oldloc.GetStr());
}

void ndFileFormatLoad::AddToWorld(ndWorld* const world)
{
	for (ndList<ndSharedPtr<ndBody>>::ndNode* node = m_bodies.GetFirst(); node; node = node->GetNext())
	{
		world->AddBody(node->GetInfo());
	}

	for (ndList<ndSharedPtr<ndJointBilateralConstraint>>::ndNode* node = m_joints.GetFirst(); node; node = node->GetNext())
	{
		world->AddJoint(node->GetInfo());
	}

	for (ndList<ndSharedPtr<ndModel>>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
	{
		world->AddModel(node->GetInfo());
	}
}