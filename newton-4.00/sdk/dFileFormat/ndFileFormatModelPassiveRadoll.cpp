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
#include "ndFileFormatModelPassiveRadoll.h"

ndFileFormatModelPassiveRadoll::ndFileFormatModelPassiveRadoll()
	:ndFileFormatModelBase(ndModelPassiveRagdoll::StaticClassName())
{
}

ndFileFormatModelPassiveRadoll::ndFileFormatModelPassiveRadoll(const char* const className)
	:ndFileFormatModelBase(className)
{
}

void ndFileFormatModelPassiveRadoll::SaveModel(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndModel* const model)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndModelPassiveRagdoll", ndModelPassiveRagdoll::StaticClassName());
	ndFileFormatModelBase::SaveModel(scene, classNode, model);

	ndModelPassiveRagdoll* const ragDoll = (ndModelPassiveRagdoll*)model;

	if (ragDoll->GetRoot())
	{
		ndFixSizeArray<ndModelPassiveRagdoll::ndRagdollNode*, 256> stack;
		stack.PushBack(ragDoll->GetRoot());
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndModelPassiveRagdoll::ndRagdollNode* const node = stack[index];
			stack.SetCount(index);

			nd::TiXmlElement* const limbNode = new nd::TiXmlElement("limb");
			classNode->LinkEndChild(limbNode);
			ndInt32 childId = scene->FindBodyId(node->m_body);
			ndInt32 parentId = node->GetParent() ? scene->FindBodyId(node->GetParent()->m_body) : 0;
			xmlSaveParam(limbNode, "childBody", childId);
			xmlSaveParam(limbNode, "parentBody", parentId);

			for (ndModelPassiveRagdoll::ndRagdollNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}
}
