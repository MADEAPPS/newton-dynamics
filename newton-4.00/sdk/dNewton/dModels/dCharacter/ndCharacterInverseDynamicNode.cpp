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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndCharacter.h"
#include "ndBodyDynamic.h"
#include "ndJointBallAndSocket.h"
#include "ndCharacterInverseDynamicNode.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndCharacterInverseDynamicNode)

ndCharacterInverseDynamicNode::ndCharacterInverseDynamicNode(const ndMatrix& matrixInGlobalSpace, ndBodyDynamic* const body, ndCharacterNode* const parent)
	:ndCharacterNode(parent)
	,m_body(body)
	,m_joint(new ndJointBallAndSocket(matrixInGlobalSpace, body, parent->GetBody()))
{
	m_localPose = m_body->GetMatrix() * parent->GetBody()->GetMatrix().Inverse();
}

ndCharacterInverseDynamicNode::ndCharacterInverseDynamicNode(const ndCharacterLoadDescriptor& desc)
	:ndCharacterNode(desc)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	const char* const name = xmlGetString(xmlNode, "name");
	SetName(name);
	m_localPose = xmlGetMatrix(xmlNode, "localPose");
	dInt32 bodyHash = xmlGetInt(xmlNode, "bodyHash");
	dInt32 jointHash = xmlGetInt(xmlNode, "jointHash");

	const ndBody* const body = desc.m_bodyMap->Find(bodyHash)->GetInfo();
	const ndJointBilateralConstraint* const joint = desc.m_jointMap->Find(jointHash)->GetInfo();
	m_body = (ndBodyDynamic*)body;
	m_joint = (ndJointBilateralConstraint*)joint;
}

ndCharacterInverseDynamicNode::~ndCharacterInverseDynamicNode()
{
	delete m_joint;
	delete m_body;
}

void ndCharacterInverseDynamicNode::Save(const ndCharacterSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_limbMap->GetCount());
	ndCharacterNode::Save(ndCharacterSaveDescriptor(desc, childNode));

	ndTree<dInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(m_joint);
	if (!jointNode)
	{
		jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), m_joint);
	}

	ndTree<dInt32, const ndBodyKinematic*>::ndNode* bodyNode = desc.m_bodyMap->Find(m_body);
	if (!bodyNode)
	{
		bodyNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), m_body);
	}
	dAssert(jointNode);
	dAssert(bodyNode);

	xmlSaveParam(childNode, "name", GetName().GetStr());
	xmlSaveParam(childNode, "localPose", m_localPose);
	xmlSaveParam(childNode, "bodyHash", dInt32(bodyNode->GetInfo()));
	xmlSaveParam(childNode, "jointHash", dInt32(jointNode->GetInfo()));
}
