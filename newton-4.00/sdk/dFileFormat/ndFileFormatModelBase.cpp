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
#include "ndFileFormat.h"
#include "ndFileFormatModelBase.h"

ndFileFormatModelBase::ndFileFormatModelBase()
	:ndFileFormatModel(ndModelBase::StaticClassName())
{
}

ndFileFormatModelBase::ndFileFormatModelBase(const char* const className)
	:ndFileFormatModel(className)
{
}

void ndFileFormatModelBase::SaveModel(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndModel* const model)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndModelBase", ndModelBase::StaticClassName());
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
			ndAssert(0);
		}
	}
}
