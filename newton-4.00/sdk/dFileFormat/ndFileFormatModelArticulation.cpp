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
#include "ndFileFormatModelArticulation.h"

ndFileFormatModelArticulation::ndFileFormatModelArticulation()
	:ndFileFormatModel(ndModelArticulation::StaticClassName())
{
}

ndFileFormatModelArticulation::ndFileFormatModelArticulation(const char* const className)
	:ndFileFormatModel(className)
{
}

void ndFileFormatModelArticulation::SaveModel(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndModel* const model)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_MODEL_CLASS, ndModelArticulation::StaticClassName());
	ndFileFormatModel::SaveModel(scene, classNode, model);

	ndModelArticulation* const articulatedModel = (ndModelArticulation*)model;

	if (articulatedModel->GetRoot())
	{
		xmlSaveParam(classNode, "rootBody", scene->FindBodyId(articulatedModel->GetRoot()->m_body->GetAsBodyKinematic()));
		nd::TiXmlElement* const limbsNode = new nd::TiXmlElement("limbs");
		classNode->LinkEndChild(limbsNode);

		ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
		for (ndModelArticulation::ndNode* child = articulatedModel->GetRoot()->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
		ndInt32 count = 0;
		while (stack.GetCount())
		{
			count++;
			ndInt32 index = stack.GetCount() - 1;
			ndModelArticulation::ndNode* const node = stack[index];
			stack.SetCount(index);
		
			nd::TiXmlElement* const limbNode = new nd::TiXmlElement("limb");
			limbsNode->LinkEndChild(limbNode);
			ndInt32 childId = scene->FindBodyId(node->m_body->GetAsBodyKinematic());
			ndInt32 parentId = scene->FindBodyId(node->GetParent()->m_body->GetAsBodyKinematic());
			ndInt32 jointId = scene->FindJointId(node->m_joint->GetAsBilateral());
			ndAssert(childId != 0);
			ndAssert(parentId != 0);
			ndAssert(jointId != 0);
			limbNode->SetAttribute("childBody", childId);
			limbNode->SetAttribute("parentBody", parentId);
			limbNode->SetAttribute("joint", jointId);
		
			for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
		limbsNode->SetAttribute("count", count);

		if (articulatedModel->m_closeLoops.GetCount())
		{
			nd::TiXmlElement* const loopsNode = new nd::TiXmlElement("closeLoops");
			classNode->LinkEndChild(loopsNode);

			loopsNode->SetAttribute("count", articulatedModel->m_closeLoops.GetCount());

			for (ndSharedList<ndJointBilateralConstraint>::ndNode* node = articulatedModel->m_closeLoops.GetFirst(); node; node = node->GetNext())
			{
				ndInt32 jointId = scene->FindJointId(node->GetInfo()->GetAsBilateral());
				nd::TiXmlElement* const loopNode = new nd::TiXmlElement("loopJoint");
				loopsNode->LinkEndChild(loopNode);
				loopNode->SetAttribute("joint", jointId);
			}
		}
	}
}

ndModel* ndFileFormatModelArticulation::LoadModel(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap)
{
	ndModelArticulation* const model = new ndModelArticulation();
	LoadModel(node, bodyMap, jointMap, model);
	return model;
}

void ndFileFormatModelArticulation::LoadModel(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap, ndModel* const model)
{
	ndFileFormatModel::LoadModel((nd::TiXmlElement*)node->FirstChild(D_MODEL_CLASS), bodyMap, jointMap, model);

	ndModelArticulation* const modelBase = (ndModelArticulation*)model;

	ndInt32 rootBodyId = xmlGetInt(node, "rootBody");
	ndTree<ndSharedPtr<ndBody>, ndInt32>::ndNode* const rootBodyNode = bodyMap.Find(rootBodyId);

	ndTree<ndModelArticulation::ndNode*, ndInt32> filter;
	filter.Insert(modelBase->AddRootBody(rootBodyNode->GetInfo()), rootBodyId);

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
		ndModelArticulation::ndNode* const parent = filter.Find(parentId)->GetInfo();
		ndModelArticulation::ndNode* const child = modelBase->AddLimb(parent, childBodyNode->GetInfo(), jointNode->GetInfo());
		filter.Insert(child, childId);
	}

	const nd::TiXmlNode* const closeLoopsNode = node->FirstChild("closeLoops");
	ndAssert(closeLoopsNode);
	for (const nd::TiXmlNode* childNode = closeLoopsNode->FirstChild("loopJoint"); childNode; childNode = childNode->NextSibling())
	{
		ndInt32 jointId;
		((nd::TiXmlElement*)childNode)->Attribute("joint", &jointId);
		ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>::ndNode* const jointNode = jointMap.Find(jointId);
		modelBase->AddCloseLoop(jointNode->GetInfo());
	}
}
