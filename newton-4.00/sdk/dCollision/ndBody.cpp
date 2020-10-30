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
#include "tinyxml.h"
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

void ndBody::SaveLow(void* const xmlNode, dInt32 nodeid) const
{
	nd::TiXmlElement* const paramNode = (nd::TiXmlElement*)xmlNode;
	paramNode->SetAttribute("nodeId", nodeid);

	#define ADD_PARAM(root, label, data) \
	{	\
		nd::TiXmlElement* const node = new nd::TiXmlElement("param"); \
		root->LinkEndChild(node); \
		node->SetAttribute(label, data); \
	}

	char buffer[1024];

	sprintf(buffer, "%f %f %f", m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z);
	ADD_PARAM(paramNode, "position", buffer);

	dVector euler0;
	dVector euler1;
	m_matrix.CalcPitchYawRoll(euler0, euler1);
	sprintf(buffer, "%f %f %f", euler0.m_x * dRadToDegree, euler0.m_y * dRadToDegree, euler0.m_z * dRadToDegree);
	ADD_PARAM(paramNode, "rotation", buffer);
	
	sprintf(buffer, "%f %f %f", m_veloc.m_x, m_veloc.m_y, m_veloc.m_z);
	ADD_PARAM(paramNode, "velocity", buffer);
	
	sprintf(buffer, "%f %f %f", m_omega.m_x, m_omega.m_y, m_omega.m_z);
	ADD_PARAM(paramNode, "omega", buffer);

	sprintf(buffer, "%f %f %f", m_localCentreOfMass.m_x, m_localCentreOfMass.m_y, m_localCentreOfMass.m_z);
	ADD_PARAM(paramNode, "centreOfMass", buffer);
}
