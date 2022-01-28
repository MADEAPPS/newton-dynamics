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
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndBodyNotify.h"

ndUnsigned32 ndBody::m_uniqueIdCount = 0;

ndBody::ndBody()
	:ndContainersFreeListAlloc<ndBody>()
	,m_matrix(dGetIdentityMatrix())
	,m_veloc(ndVector::m_zero)
	,m_omega(ndVector::m_zero)
	,m_localCentreOfMass(ndVector::m_wOne)
	,m_globalCentreOfMass(ndVector::m_wOne)
	,m_minAabb(ndVector::m_wOne)
	,m_maxAabb(ndVector::m_wOne)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_uniqueId(m_uniqueIdCount)
	,m_flags(0)
	,m_isStatic(0)
	,m_autoSleep(1)
	,m_islandSleep(0)
	,m_equilibrium(0)
	,m_equilibrium0(0)
	,m_isJointFence0(0)
	,m_isJointFence1(0)
	,m_bodyIsConstrained(0)
{
	m_uniqueIdCount++;
	m_transformIsDirty = 1;
}

ndBody::ndBody(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndContainersFreeListAlloc<ndBody>()
	,m_matrix(dGetIdentityMatrix())
	,m_veloc(ndVector::m_zero)
	,m_omega(ndVector::m_zero)
	,m_localCentreOfMass(ndVector::m_wOne)
	,m_globalCentreOfMass(ndVector::m_wOne)
	,m_minAabb(ndVector::m_wOne)
	,m_maxAabb(ndVector::m_wOne)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_uniqueId(m_uniqueIdCount)
	,m_flags(0)
	,m_isStatic(0)
	,m_autoSleep(1)
	,m_islandSleep(0)
	,m_equilibrium(0)
	,m_equilibrium0(0)
	,m_isJointFence0(0)
	,m_isJointFence1(0)
	,m_bodyIsConstrained(0)
{
	m_uniqueIdCount++;
	m_transformIsDirty = 1;

	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	ndMatrix matrix(xmlGetMatrix(xmlNode, "matrix"));
	m_veloc = xmlGetVector3(xmlNode, "veloc");
	m_omega = xmlGetVector3(xmlNode, "omega");
	m_localCentreOfMass = xmlGetVector3(xmlNode, "centreOfMass");
	m_autoSleep = xmlGetInt(xmlNode, "autoSleep") ? 1 : 0;
	
	SetMatrix(matrix);
	const nd::TiXmlNode* const notifyNode = xmlNode->FirstChild("bodyNotifyClass");
	if (notifyNode)
	{
		const nd::TiXmlNode* const node = notifyNode->FirstChild();
		if (node)
		{
			const char* const className = node->Value();

			ndLoadSaveBase::ndLoadDescriptor notifyDesc(desc);
			notifyDesc.m_rootNode = node;
			m_notifyCallback = D_CLASS_REFLECTION_LOAD_NODE(ndBodyNotify, className, notifyDesc);
			m_notifyCallback->m_body = this;
		}
	}
}

ndBody::~ndBody()
{
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
}

void ndBody::SetCentreOfMass(const ndVector& com)
{
	m_localCentreOfMass.m_x = com.m_x;
	m_localCentreOfMass.m_y = com.m_y;
	m_localCentreOfMass.m_z = com.m_z;
	m_localCentreOfMass.m_w = ndFloat32(1.0f);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}

void ndBody::SetNotifyCallback(ndBodyNotify* const notify)
{
	//dAssert(notify->m_body == nullptr);
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
	m_notifyCallback = notify;
	if (m_notifyCallback)
	{
		m_notifyCallback->m_body = this;
	}
}

void ndBody::SetOmega(const ndVector& omega)
{
	m_omega = omega;
	m_equilibrium = 0;
}

void ndBody::SetVelocity(const ndVector& veloc)
{
	m_veloc = veloc;
	m_equilibrium = 0;
}

void ndBody::SetMatrix(const ndMatrix& matrix)
{
	m_equilibrium = 0;
	m_transformIsDirty = 1;
	m_matrix = matrix;
	dAssert(m_matrix.TestOrthogonal(ndFloat32(1.0e-4f)));

	m_rotation = ndQuaternion(m_matrix);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}

D_COLLISION_API const nd::TiXmlNode* ndBody::FindNode(const nd::TiXmlNode* const rootNode, const char* const name)
{
	return rootNode->FirstChild(name);
}

void ndBody::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);

	if (m_notifyCallback)
	{
		nd::TiXmlElement* const notifyNode = new nd::TiXmlElement("bodyNotifyClass");
		childNode->LinkEndChild(notifyNode);
		m_notifyCallback->Save(ndLoadSaveBase::ndSaveDescriptor(desc, notifyNode));
	}

	xmlSaveParam(childNode, "matrix", m_matrix);
	xmlSaveParam(childNode, "veloc", m_veloc);
	xmlSaveParam(childNode, "omega", m_omega);
	xmlSaveParam(childNode, "centreOfMass", m_localCentreOfMass);
	xmlSaveParam(childNode, "autoSleep", m_autoSleep);
}
