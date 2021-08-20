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
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndBodyNotify.h"

dUnsigned32 ndBody::m_uniqueIdCount = 0;

ndBody::ndBody()
	:m_matrix(dGetIdentityMatrix())
	,m_veloc(dVector::m_zero)
	,m_omega(dVector::m_zero)
	,m_localCentreOfMass(dVector::m_wOne)
	,m_globalCentreOfMass(dVector::m_wOne)
	,m_minAabb(dVector::m_wOne)
	,m_maxAabb(dVector::m_wOne)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_flags(0)
	,m_uniqueId(m_uniqueIdCount)
{
	m_autoSleep = 1;
	m_transformIsDirty = 1;
	//m_collideWithLinkedBodies = 1;
	m_uniqueIdCount++;
}

ndBody::ndBody(const dClassLoaderBase::dDesc& desc)
	:m_matrix(dGetIdentityMatrix())
	,m_veloc(dVector::m_zero)
	,m_omega(dVector::m_zero)
	,m_localCentreOfMass(dVector::m_wOne)
	,m_globalCentreOfMass(dVector::m_wOne)
	,m_minAabb(dVector::m_wOne)
	,m_maxAabb(dVector::m_wOne)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_flags(0)
	,m_uniqueId(m_uniqueIdCount)
{
	m_uniqueIdCount++;
	m_transformIsDirty = 1;

	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	dMatrix matrix(xmlGetMatrix(xmlNode, "matrix"));
	m_veloc = xmlGetVector3(xmlNode, "veloc");
	m_omega = xmlGetVector3(xmlNode, "omega");
	m_localCentreOfMass = xmlGetVector3(xmlNode, "centreOfMass");
	m_autoSleep = xmlGetInt(xmlNode, "autoSleep") ? 1 : 0;
	
	SetMatrix(matrix);
	const nd::TiXmlNode* const notifyNode = xmlNode->FirstChild("ndBodyNotify");
	if (notifyNode)
	{
		m_notifyCallback = new ndBodyNotify(notifyNode);
		m_notifyCallback->m_body = this;
	}
}

ndBody::~ndBody()
{
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
}

void ndBody::SetCentreOfMass(const dVector& com)
{
	m_localCentreOfMass.m_x = com.m_x;
	m_localCentreOfMass.m_y = com.m_y;
	m_localCentreOfMass.m_z = com.m_z;
	m_localCentreOfMass.m_w = dFloat32(1.0f);
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

void ndBody::SetOmega(const dVector& omega)
{
	m_equilibrium = 0;
	m_omega = omega;
}

void ndBody::SetVelocity(const dVector& veloc)
{
	m_equilibrium = 0;
	m_veloc = veloc;
}

dMatrix ndBody::GetMatrix() const
{
	return m_matrix;
}

dQuaternion ndBody::GetRotation() const
{
	return m_rotation;
}

void ndBody::SetMatrix(const dMatrix& matrix)
{
	m_equilibrium = 0;
	m_transformIsDirty = 1;
	m_matrix = matrix;
	dAssert(m_matrix.TestOrthogonal(dFloat32(1.0e-4f)));

	m_rotation = dQuaternion(m_matrix);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}

D_COLLISION_API const nd::TiXmlNode* ndBody::FindNode(const nd::TiXmlNode* const rootNode, const char* const name)
{
	return rootNode->FirstChild(name);
}

void ndBody::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32, dInt32 nodeHash) const
{
	nd::TiXmlElement* const paramNode = new nd::TiXmlElement(ClassName());
	rootNode->LinkEndChild(paramNode);
	paramNode->SetAttribute("hashId", nodeHash);

	if (m_notifyCallback)
	{
		m_notifyCallback->Save(paramNode, assetPath);
	}

	xmlSaveParam(paramNode, "matrix", m_matrix);
	xmlSaveParam(paramNode, "veloc", m_veloc);
	xmlSaveParam(paramNode, "omega", m_omega);
	xmlSaveParam(paramNode, "centreOfMass", m_localCentreOfMass);
	xmlSaveParam(paramNode, "autoSleep", m_autoSleep ? 1 : 0);
}
