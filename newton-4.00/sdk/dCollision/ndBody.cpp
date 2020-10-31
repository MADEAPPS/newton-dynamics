/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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


dUnsigned32 ndBody::m_uniqueIDCount = 0;

ndBody::ndBody()
	:m_matrix(dGetIdentityMatrix())
	,m_veloc(dVector::m_zero)
	,m_omega(dVector::m_zero)
	,m_localCentreOfMass(dVector::m_wOne)
	,m_globalCentreOfMass(dVector::m_wOne)
	,m_minAABB(dVector::m_wOne)
	,m_maxAABB(dVector::m_wOne)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_flags(0)
	,m_uniqueID(m_uniqueIDCount)
{
	m_autoSleep = 1;
	m_transformIsDirty = 1;
	m_collideWithLinkedBodies = 1;
	m_uniqueIDCount++;
}

ndBody::ndBody(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache)
	:m_matrix(dGetIdentityMatrix())
	,m_veloc(dVector::m_zero)
	,m_omega(dVector::m_zero)
	,m_localCentreOfMass(dVector::m_wOne)
	,m_globalCentreOfMass(dVector::m_wOne)
	,m_minAABB(dVector::m_wOne)
	,m_maxAABB(dVector::m_wOne)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_flags(0)
	,m_uniqueID(m_uniqueIDCount)
{
	m_uniqueIDCount++;
	m_transformIsDirty = 1;

	SetMatrix(xmlGetMatrix(xmlNode, "matrix"));
	m_veloc = xmlGetVector3(xmlNode, "veloc");
	m_omega = xmlGetVector3(xmlNode, "omega");
	m_localCentreOfMass = xmlGetVector3(xmlNode, "centreOfMass");
	m_autoSleep = xmlGetInt(xmlNode, "autoSleep") ? 1 : 0;
	m_gyroTorqueOn = xmlGetInt(xmlNode, "useGyroTorque") ? 1 : 0;
	m_collideWithLinkedBodies = xmlGetInt(xmlNode, "collideWithLinkedBodies") ? 1 : 0;
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

void ndBody::SetOmega(const dVector& veloc)
{
	m_equilibrium = 0;
	m_omega = veloc;
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

nd::TiXmlElement* ndBody::CreateRootElement(nd::TiXmlElement* const rootNode, const char* const name, dInt32 nodeid) const
{
	nd::TiXmlElement* const paramNode = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(paramNode);

	paramNode->SetAttribute("nodeId", nodeid);
	return paramNode;
}

void ndBody::Save(nd::TiXmlElement* const rootNode, dInt32 nodeid, const dTree<dUnsigned32, const ndShape*>& shapesCache) const
{
	nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBody", nodeid);
	xmlSaveParam(paramNode, "matrix", m_matrix);
	xmlSaveParam(paramNode, "veloc", m_veloc);
	xmlSaveParam(paramNode, "omega", m_omega);
	xmlSaveParam(paramNode, "centreOfMass", m_localCentreOfMass);
	xmlSaveParam(paramNode, "autoSleep", m_autoSleep ? 1 : 0);
	xmlSaveParam(paramNode, "useGyroTorque", m_gyroTorqueOn ? 1 : 0);
	xmlSaveParam(paramNode, "collideWithLinkedBodies", m_collideWithLinkedBodies ? 1 : 0);
}
