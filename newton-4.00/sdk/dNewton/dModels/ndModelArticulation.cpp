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
#include "ndModelArticulation.h"

ndModelArticulation::ndNode::ndNode(const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint, ndNode* const parent)
	:ndNodeHierarchy<ndNode>()
	,m_body(body)
	,m_joint(joint)
{
	if (parent)
	{
		Attach(parent);
	}
}

ndModelArticulation::ndNode::~ndNode()
{
}

ndModelArticulation::ndModelArticulation()
	:ndModel()
	,m_rootNode(nullptr)
	,m_invDynamicsSolver()
	,m_closeLoops()
{
}

ndModelArticulation::~ndModelArticulation()
{
	if (m_rootNode)
	{
		delete m_rootNode;
	}
}

ndModelArticulation* ndModelArticulation::GetAsModelArticulation()
{
	return this;
}

ndModelArticulation::ndNode* ndModelArticulation::GetRoot() const
{
	return m_rootNode;
}

ndModelArticulation::ndNode* ndModelArticulation::AddRootBody(const ndSharedPtr<ndBody>& rootBody)
{
	ndAssert(!m_rootNode);
	ndSharedPtr <ndJointBilateralConstraint> dommyJoint;
	m_rootNode = new ndNode(rootBody, dommyJoint, nullptr);
	return m_rootNode;
}

ndModelArticulation::ndNode* ndModelArticulation::AddLimb(ndNode* const parent, const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	ndAssert(m_rootNode);
	ndAssert(joint->GetBody0() == body->GetAsBodyKinematic());
	ndAssert(joint->GetBody1() == parent->m_body->GetAsBodyKinematic());
	return new ndNode(body, joint, parent);
}

void ndModelArticulation::OnAddToWorld()
{
	ndAssert(m_world);
	ndFixSizeArray<ndNode*, 256> stack;
	if (m_rootNode)
	{
		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			m_world->AddBody(node->m_body);
			if (node->m_joint)
			{
				m_world->AddJoint(node->m_joint);
			}

			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}

	for (ndSharedList<ndJointBilateralConstraint>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		m_world->AddJoint(node->GetInfo());
	}
}

void ndModelArticulation::OnRemoveFromToWorld()
{
	ndAssert(m_world);
	ndFixSizeArray<ndNode*, 256> stack;
	if (m_rootNode)
	{
		for (ndSharedList<ndJointBilateralConstraint>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
		{
			if (node->GetInfo()->m_worldNode)
			{
				m_world->RemoveJoint(node->GetInfo());
			}
		}

		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			if (node->m_joint)
			{
				if (node->m_joint->m_worldNode)
				{
					m_world->RemoveJoint(node->m_joint);
				}
			}
			if (node->m_body->GetAsBodyKinematic()->m_sceneNode)
			{
				m_world->RemoveBody(node->m_body);
			}

			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}
}

void ndModelArticulation::AddCloseLoop(const ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	#ifdef _DEBUG

	auto Check = [&](const ndBodyKinematic* const body)
	{
		if (body->GetInvMass() == ndFloat32(0.0f))
		{
			return false;
		}
		ndFixSizeArray<ndNode*, 256> stack;
		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			if (node->m_body->GetAsBodyKinematic() == body)
			{
				return true;
			}

			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}

		return false;
	};

	ndAssert(Check(joint->GetBody0()) || Check(joint->GetBody1()));
	#endif

	for (ndSharedList<ndJointBilateralConstraint>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		if (*node->GetInfo() == *joint)
		{
			return;
		}
	}

	m_closeLoops.Append(joint);
}