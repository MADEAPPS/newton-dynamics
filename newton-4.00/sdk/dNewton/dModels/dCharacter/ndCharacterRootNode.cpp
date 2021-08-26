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
#include "ndCharacter.h"
#include "ndBodyDynamic.h"
#include "ndCharacterRootNode.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndCharacterRootNode)

ndCharacterRootNode::ndCharacterRootNode(ndCharacter* const owner, ndBodyDynamic* const body)
	:ndCharacterLimbNode(nullptr)
	,m_coronalFrame(dGetIdentityMatrix())
	,m_invCoronalFrame(dGetIdentityMatrix())
	,m_gravityDir(dFloat32 (0.0f), dFloat32(-1.0f), dFloat32(0.0f), dFloat32(0.0f))
	,m_owner(owner)
	,m_body(body)
{
	SetCoronalFrame(m_body->GetMatrix());
}

ndCharacterRootNode::ndCharacterRootNode(const dLoadSaveBase::dLoadDescriptor& desc)
	//:ndCharacterLimbNode(dLoadSaveBase::dLoadDescriptor(desc))
	:ndCharacterLimbNode(desc)
	,m_coronalFrame(dGetIdentityMatrix())
	,m_invCoronalFrame(dGetIdentityMatrix())
	,m_gravityDir(dFloat32(0.0f), dFloat32(-1.0f), dFloat32(0.0f), dFloat32(0.0f))
	,m_owner(nullptr)
	,m_body(nullptr)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_coronalFrame = xmlGetMatrix(xmlNode, "coronalFrame");
	m_gravityDir = xmlGetVector3(xmlNode, "gravityDir");

	dInt32 bodyHash = xmlGetInt(xmlNode, "bodyHash");
	m_body = (ndBodyDynamic*)desc.m_bodyMap->Find(bodyHash)->GetInfo();

	SetCoronalFrame(m_body->GetMatrix());
}

ndCharacterRootNode::~ndCharacterRootNode()
{
}

void ndCharacterRootNode::SetCoronalFrame(const dMatrix& frameInGlobalSpace)
{
	dMatrix matrix(m_body->GetMatrix());
	m_coronalFrame = frameInGlobalSpace * matrix.Inverse();
	m_invCoronalFrame = m_coronalFrame.Inverse();
}

void ndCharacterRootNode::UpdateGlobalPose(ndWorld* const, dFloat32)
{
	// for now just; 
}

void ndCharacterRootNode::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	ndCharacterLimbNode::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "coronalFrame", m_coronalFrame);
	xmlSaveParam(childNode, "gravityDir", m_gravityDir);
	//xmlSaveParam(childNode, "modelHash", desc.m_nodeNodeHash);
	xmlSaveParam(childNode, "bodyHash", dInt32(desc.m_bodyMap->Find(GetBody())->GetInfo()));
}
