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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndModel.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndModel)

ndModel::ndModel(const ndLoadSaveBase::ndLoadDescriptor&)
	:ndContainersFreeListAlloc<ndModel>()
	,m_referencedBodies()
	,m_node(nullptr)
	,m_world(nullptr)
	,m_markedForRemoved(0)
{
}

void ndModel::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
}

void ndModel::AddToWorld(ndWorld* const world)
{
	m_world = world;
	for (ndReferencedObjects<ndBodyKinematic>::ndNode* node = m_referencedBodies.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBodyKinematic>& body = node->GetInfo();
		world->AddBody(body);
	}

	for (ndReferencedObjects<ndJointBilateralConstraint>::ndNode* node = m_referencedJoints.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndJointBilateralConstraint>& joint = node->GetInfo();
		world->AddJoint(joint);
	}
}

void ndModel::RemoveFromToWorld()
{
	if (m_world)
	{
		ndWorld* const world = m_world;
		m_world = nullptr;
		for (ndReferencedObjects<ndJointBilateralConstraint>::ndNode* node = m_referencedJoints.GetFirst(); node; node = node->GetNext())
		{
			ndSharedPtr<ndJointBilateralConstraint>& joint = node->GetInfo();
			world->RemoveJoint(*joint);
		}

		for (ndReferencedObjects<ndBodyKinematic>::ndNode* node = m_referencedBodies.GetFirst(); node; node = node->GetNext())
		{
			ndSharedPtr<ndBodyKinematic>& body = node->GetInfo();
			world->RemoveBody(*body);
		}
	}
}

void ndModel::AddBody(ndSharedPtr<ndBodyKinematic>& body)
{
	if (!FindBodyReference(*body))
	{
		m_referencedBodies.Append(body);
		ndBodyKinematic::ndModelList& modelList = body->GetModelList();
		modelList.Append(this);
	}
}

void ndModel::AddJoint(ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	if (!FindJointReference(*joint))
	{
		m_referencedJoints.Append(joint);
	}
}