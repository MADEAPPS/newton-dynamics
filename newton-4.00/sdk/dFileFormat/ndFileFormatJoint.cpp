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
#include "ndFileFormatJoint.h"

ndFileFormatJoint::ndFileFormatJoint()
	:ndFileFormatRegistrar(ndJointBilateralConstraint::StaticClassName())
{
}

ndFileFormatJoint::ndFileFormatJoint(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatJoint::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointBilateralConstraint::StaticClassName());

	ndBodyKinematic* const body0 = joint->GetBody0();
	ndBodyKinematic* const body1 = joint->GetBody1();

	ndTree<ndInt32, ndUnsigned64>::ndNode* const node0 = scene->m_bodiesIds.Find(body0->GetId());
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node1 = scene->m_bodiesIds.Find(body1->GetId());
	ndAssert(node0);

	ndInt32 body0NodeId = node0->GetInfo();
	ndInt32 body1NodeId = node1 ? node1->GetInfo() : 0;

	union Key
	{
		ndUnsigned64 m_hash;
		struct
		{
			ndInt32 m_body0;
			ndInt32 m_body1;
		};
	};

	Key key;
	key.m_body0 = body0NodeId;
	key.m_body1 = body1NodeId;
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node = scene->m_jointsIds.Insert(key.m_hash);
	ndAssert(node);
	if (node)
	{
		ndInt32 id;
		classNode->Attribute("nodeId", &id);
		node->GetInfo() = id;
	}

	xmlSaveParam(classNode, "body0", body0NodeId);
	xmlSaveParam(classNode, "body1", body1NodeId);

	xmlSaveParam(classNode, "localMatrix0", joint->GetLocalMatrix0());
	xmlSaveParam(classNode, "localMatrix1", joint->GetLocalMatrix1());
	xmlSaveParam(classNode, "solverMode", joint->GetSolverModel());
}

ndJointBilateralConstraint* ndFileFormatJoint::LoadJoint(const nd::TiXmlElement* const, const ndTree<ndSharedPtr<ndBody>, ndInt32>&)
{
	ndAssert(0);
	return nullptr;
}

void ndFileFormatJoint::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndInt32 idBody0 = xmlGetInt(node, "body0");
	ndInt32 idBody1 = xmlGetInt(node, "body1");

	ndTree<ndSharedPtr<ndBody>, ndInt32>::ndNode* const node0 = bodyMap.Find(idBody0);
	ndTree<ndSharedPtr<ndBody>, ndInt32>::ndNode* const node1 = bodyMap.Find(idBody1);

	ndAssert(node0);
	ndAssert(node1);
	ndBodyKinematic* const body0 = node0->GetInfo()->GetAsBodyKinematic();
	ndBodyKinematic* const body1 = node1->GetInfo()->GetAsBodyKinematic();

	ndMatrix matrix0(xmlGetMatrix(node, "localMatrix0"));
	ndMatrix matrix1(xmlGetMatrix(node, "localMatrix1"));
	ndInt32 solverModel = xmlGetInt(node, "solverMode");

	joint->m_body0 = body0;
	joint->m_body1 = body1;
	joint->m_localMatrix0 = matrix0;
	joint->m_localMatrix1 = matrix1;
	joint->m_solverModel = ndJointBilateralSolverModel(solverModel);
}