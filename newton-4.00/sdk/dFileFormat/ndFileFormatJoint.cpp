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
#include "ndFileFormatJoint.h"

ndFileFormatJoint::ndFileFormatJoint()
	:ndFileFormatRegistrar(ndJointBilateralConstraint::StaticClassName())
{
}

ndFileFormatJoint::ndFileFormatJoint(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatJoint::SaveJoint(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndJointClass", ndJointBilateralConstraint::StaticClassName());

	ndBodyKinematic* const body0 = joint->GetBody0();
	ndBodyKinematic* const body1 = joint->GetBody1();

	ndTree<ndInt32, ndUnsigned64>::ndNode* const node0 = scene->m_bodiesIds.Find(body0->GetId());
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node1 = scene->m_bodiesIds.Find(body1->GetId());
	ndAssert(node0);

	ndInt32 body0NodeId = node0->GetInfo();
	ndInt32 body1NodeId = node1 ? node1->GetInfo() : -1;

	union Key
	{
		ndUnsigned64 m_key;
		struct
		{
			ndInt32 m_low;
			ndInt32 m_high;
		};
	};

	Key key;
	key.m_low = body1NodeId;
	key.m_high = body0NodeId;
	ndTree<ndInt32, ndUnsigned64>::ndNode* const node = scene->m_jointsIds.Insert(key.m_key);
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
