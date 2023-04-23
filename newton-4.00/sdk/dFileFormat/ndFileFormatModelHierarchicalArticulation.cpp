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
#include "ndFileFormatModelHierarchicalArticulation.h"

ndFileFormatModelHierarchicalArticulation::ndFileFormatModelHierarchicalArticulation()
	:ndFileFormatModelBase(ndModelHierarchicalArticulation::StaticClassName())
{
}

ndFileFormatModelHierarchicalArticulation::ndFileFormatModelHierarchicalArticulation(const char* const className)
	:ndFileFormatModelBase(className)
{
}

void ndFileFormatModelHierarchicalArticulation::SaveModel(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndModel* const model)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_MODEL_CLASS, ndModelHierarchicalArticulation::StaticClassName());
	ndFileFormatModelBase::SaveModel(scene, classNode, model);

	ndModelHierarchicalArticulation* const articulatedModel = (ndModelHierarchicalArticulation*)model;

	if (articulatedModel->GetRoot())
	{
		xmlSaveParam(classNode, "rootBody", scene->FindBodyId(articulatedModel->GetRoot()->m_body));
		nd::TiXmlElement* const limbsNode = new nd::TiXmlElement("limbs");
		classNode->LinkEndChild(limbsNode);

		ndFixSizeArray<ndModelHierarchicalArticulation::ndNode*, 256> stack;
		for (ndModelHierarchicalArticulation::ndNode* child = articulatedModel->GetRoot()->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
		ndInt32 count = 0;
		while (stack.GetCount())
		{
			count++;
			ndInt32 index = stack.GetCount() - 1;
			ndModelHierarchicalArticulation::ndNode* const node = stack[index];
			stack.SetCount(index);
		
			nd::TiXmlElement* const limbNode = new nd::TiXmlElement("limb");
			limbsNode->LinkEndChild(limbNode);
			ndInt32 childId = scene->FindBodyId(node->m_body);
			ndInt32 parentId = scene->FindBodyId(node->GetParent()->m_body);
			ndInt32 jointId = scene->FindJointId(node->m_joint);
			ndAssert(childId != 0);
			ndAssert(parentId != 0);
			ndAssert(jointId != 0);
			limbNode->SetAttribute("childBody", childId);
			limbNode->SetAttribute("parentBody", parentId);
			limbNode->SetAttribute("joint", jointId);
		
			for (ndModelHierarchicalArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
		limbsNode->SetAttribute("count", count);
	}
}


ndModel* ndFileFormatModelHierarchicalArticulation::LoadModel(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap)
{
	ndModelHierarchicalArticulation* const model = new ndModelHierarchicalArticulation();
	LoadModel(node, bodyMap, jointMap, model);
	return model;
}

void ndFileFormatModelHierarchicalArticulation::LoadModel(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap, ndModel* const model)
{
	ndFileFormatModelBase::LoadModel((nd::TiXmlElement*)node->FirstChild(D_MODEL_CLASS), bodyMap, jointMap, model);

	ndModelHierarchicalArticulation* const modelBase = (ndModelHierarchicalArticulation*)model;

	ndInt32 rootBodyId = xmlGetInt(node, "rootBody");
	ndTree<ndSharedPtr<ndBody>, ndInt32>::ndNode* const rootBodyNode = bodyMap.Find(rootBodyId);

	ndTree<ndModelHierarchicalArticulation::ndNode*, ndInt32> filter;
	filter.Insert (modelBase->AddRootBody(rootBodyNode->GetInfo()), rootBodyId);

	const nd::TiXmlNode* const limbsNode = node->FirstChild("limbs");
	ndAssert(limbsNode);
	for (const nd::TiXmlNode* childNode = limbsNode->FirstChild("limb"); childNode; childNode = childNode->NextSibling())
	{
		ndInt32 childId;
		ndInt32 jointId;
		ndInt32 parentId;

		((nd::TiXmlElement*)childNode)->Attribute("joint", &jointId);
		((nd::TiXmlElement*)childNode)->Attribute("childBody", &childId);
		((nd::TiXmlElement*)childNode)->Attribute("parentBody", &parentId);

		ndTree<ndSharedPtr<ndBody>, ndInt32>::ndNode* const childBodyNode = bodyMap.Find(childId);
		ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>::ndNode* const jointNode = jointMap.Find(jointId);

		ndAssert(filter.Find(parentId));
		ndModelHierarchicalArticulation::ndNode* const parent = filter.Find(parentId)->GetInfo();
		ndModelHierarchicalArticulation::ndNode* const child = modelBase->AddLimb(parent, childBodyNode->GetInfo(), jointNode->GetInfo());
		filter.Insert(child, childId);
	}
}
