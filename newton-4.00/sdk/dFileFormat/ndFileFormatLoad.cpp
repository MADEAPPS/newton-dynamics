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
{
}

ndFileFormatLoad::~ndFileFormatLoad()
{
}

void ndFileFormatLoad::LoadShapes(const nd::TiXmlElement* const rootNode, ndTree<ndShape*, ndInt32>& shapeMap)
{
	const nd::TiXmlNode* const shapes = rootNode->FirstChild("ndShapes");
	if (shapes)
	{
		for (const nd::TiXmlNode* node = shapes->FirstChild(); node; node = node->NextSibling())
		{
			//const char* const name = node->Value();
			//const char* const name1 = node->Value();
			const nd::TiXmlElement* const element = (nd::TiXmlElement*)node;
			const char* const className = element->Attribute("className");
			ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(className);
			ndAssert(handler);

			ndInt32 nodeId;
			element->Attribute("nodeId", &nodeId);
			ndShape* const shape = handler->LoadShape(element);
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
			nd::TiXmlElement* element = (nd::TiXmlElement*)node;
			ndFileFormatRegistrar* handler = ndFileFormatRegistrar::GetHandler(element->Attribute("className"));
			if (!handler)
			{
				nd::TiXmlNode* childNode = (nd::TiXmlNode*)node;
				do {
					const char* const className = childNode->Value();
					if (strcmp(className, "ndBodyClass"))
					{
						break;
					}
					element = (nd::TiXmlElement*)childNode;
					ndTrace(("warning class %s not found\n", element->Attribute("className")));
					handler = ndFileFormatRegistrar::GetHandler(element->Attribute("className"));
					childNode = childNode->FirstChild();
				} while (!handler);
			}
			ndAssert(handler);

			ndSharedPtr<ndBody> body (handler->LoadBody(element, shapeMap));

			const nd::TiXmlNode* alliasNode = node;
			do {
				ndInt32 nodeId;
				nd::TiXmlElement* const alliasElement = (nd::TiXmlElement*)alliasNode;
				alliasElement->Attribute("nodeId", &nodeId);
				bodyMap.Insert(body, nodeId);
				alliasNode = alliasNode->FirstChild();
			} while (!strcmp(alliasNode->Value(), "ndBodyClass"));
			
			//m_world->AddBody(body);
		}
	}
}

void ndFileFormatLoad::Load(const ndWorld* const world, const char* const path)
{
	// save the path for use with generated assets.
	xmlResetClassId();

	m_oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	SetPath(path);

	//m_world = (ndWorld*)world;
	nd::TiXmlDocument doc(m_path.GetStr());
	//m_doc = new nd::TiXmlDocument(m_path.GetStr());
	 
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

	LoadShapes(rootNode, shapeMap);
	LoadBodies(rootNode, shapeMap, bodyMap);

	ndTree<ndShape*, ndInt32>::Iterator it (shapeMap);
	for (it.Begin(); it; it++)
	{
		ndTree<ndShape*, ndInt32>::ndNode* node = it.GetNode();
		ndShape* const shape = node->GetInfo();
		shape->Release();
	}
	
	setlocale(LC_ALL, m_oldloc.GetStr());
}