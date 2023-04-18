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
#include "ndFileFormatModelBase.h"

ndFileFormatModelBase::ndFileFormatModelBase()
	:ndFileFormatModel(ndModelBase::StaticClassName())
{
}

ndFileFormatModelBase::ndFileFormatModelBase(const char* const className)
	:ndFileFormatModel(className)
{
}

void ndFileFormatModelBase::SaveModel(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndModel* const model)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndModelClass", ndModelBase::StaticClassName());
	ndFileFormatModel::SaveModel(scene, classNode, model);
	
	ndModelBase* const modelBase = (ndModelBase*)model;

	if (modelBase->m_bodies.GetCount())
	{
		nd::TiXmlElement* const bodiesNode = new nd::TiXmlElement("bodies");
		classNode->LinkEndChild(bodiesNode);
		bodiesNode->SetAttribute("count", modelBase->m_bodies.GetCount());
		for (ndList<ndSharedPtr<ndBody>>::ndNode* node = modelBase->m_bodies.GetFirst(); node; node = node->GetNext())
		{
			ndInt32 nodeId = scene->FindBodyId(*node->GetInfo());
			xmlSaveParam(bodiesNode, "body", nodeId);
		}

		nd::TiXmlElement* const jointsNode = new nd::TiXmlElement("joints");
		classNode->LinkEndChild(jointsNode);
		jointsNode->SetAttribute("count", modelBase->m_joints.GetCount());
		for (ndList< ndSharedPtr<ndJointBilateralConstraint>>::ndNode* node = modelBase->m_joints.GetFirst(); node; node = node->GetNext())
		{
			ndInt32 nodeId = scene->FindJointId(*node->GetInfo());
			xmlSaveParam(jointsNode, "joint", nodeId);
		}
	}
}

ndModel* ndFileFormatModelBase::LoadModel(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap)
{
	ndModelBase* const model = new ndModelBase();
	LoadModel(node, bodyMap, jointMap, model);
	return model;
}

void ndFileFormatModelBase::LoadModel(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap, ndModel* const model)
{
	//ndFileFormatModel::LoadModel((nd::TiXmlElement*)node->FirstChild("ndJointClass"), bodyMap, jointMap, model);

	ndModelBase* const modelBase = (ndModelBase*)model;
	const nd::TiXmlNode* const bodiesNode = node->FirstChild("bodies");
	ndAssert(bodiesNode);
	for (const nd::TiXmlNode* childNode = bodiesNode->FirstChild("body"); childNode; childNode = childNode->NextSibling())
	{
		ndInt32 bodyId;
		((nd::TiXmlElement*)childNode)->Attribute("int32", &bodyId);
		ndTree<ndSharedPtr<ndBody>, ndInt32>::ndNode* const bodyNode = bodyMap.Find(bodyId);
		modelBase->m_bodies.Append(bodyNode->GetInfo());
	}

	const nd::TiXmlNode* const jointsNode = node->FirstChild("joints");
	ndAssert(jointsNode);
	for (const nd::TiXmlNode* childNode = jointsNode->FirstChild("joint"); childNode; childNode = childNode->NextSibling())
	{
		ndInt32 jointId;
		((nd::TiXmlElement*)childNode)->Attribute("int32", &jointId);
		ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>::ndNode* const bodyNode = jointMap.Find(jointId);
		modelBase->m_joints.Append(bodyNode->GetInfo());
	}
}
