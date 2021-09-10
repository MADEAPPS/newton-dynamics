/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndCharacterLimbNode.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndCharacterLimbNode)

ndCharacterLimbNode::ndCharacterLimbNode(ndCharacterLimbNode* const parent)
	:dNodeHierarchy<ndCharacterLimbNode>()
{
	if (parent)
	{
		Attach(parent);
	}
}

ndCharacterLimbNode::ndCharacterLimbNode(const ndCharacterLoadDescriptor& desc)
	:dNodeHierarchy<ndCharacterLimbNode>()
{
	if (desc.m_parentModelNode)
	{
		Attach((ndCharacterLimbNode*)desc.m_parentModelNode);
	}

	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	dInt32 hashId;
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) xmlNode;
	element->Attribute("hashId", &hashId);
	desc.m_limbMap->Insert(this, hashId);

	ndCharacterLoadDescriptor childDesc(desc);
	childDesc.m_parentModelNode = this;
	for (const nd::TiXmlNode* node = xmlNode->FirstChild(); node; node = node->NextSibling())
	{
		const char* const partName = node->Value();
		//dTrace(("%s\n", partName));
		if (strstr(partName, "ndCharacter"))
		{
			childDesc.m_rootNode = node;
			D_CLASS_REFLECTION_LOAD_NODE(ndCharacterLimbNode, partName, childDesc);
		}
	}
}

ndCharacterLimbNode::~ndCharacterLimbNode()
{
}

dNodeBaseHierarchy* ndCharacterLimbNode::CreateClone() const
{
	dAssert(0);
	return nullptr;
}

void ndCharacterLimbNode::Save(const ndCharacterSaveDescriptor& desc) const
{
	ndCharacterSaveDescriptor childDesc(desc);

	childDesc.m_limbMap->Insert(childDesc.m_limbMap->GetCount(), (ndCharacterLimbNode*)this);
	for (ndCharacterLimbNode* child = GetChild(); child; child = child->GetSibling())
	{
		child->Save(childDesc);
	}
}

void ndCharacterLimbNode::Debug(ndConstraintDebugCallback& context) const
{
	for (ndCharacterLimbNode* child = GetChild(); child; child = child->GetSibling())
	{
		child->Debug(context);
	}
}