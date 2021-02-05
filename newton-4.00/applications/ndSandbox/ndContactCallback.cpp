/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndContactCallback.h"

ndContactCallback::ndContactCallback()
	:ndContactNotify()
	,m_materialMap()
{
	m_materialMap.Insert(ndMaterial(), ndMaterailKey(0, 0));
}

void ndContactCallback::OnBodyAdded(ndBodyKinematic* const body) const
{
}

void ndContactCallback::OnBodyRemoved(ndBodyKinematic* const body) const
{
}

ndMaterial& ndContactCallback::RegisterMaterial(dUnsigned32 id0, dUnsigned32 id1)
{
	ndMaterailKey key(id0, id1);
	dTree<ndMaterial, ndMaterailKey>::dTreeNode* node = m_materialMap.Find(key);
	if (!node)
	{
		node = m_materialMap.Insert(ndMaterial(), key);
	}
	return node->GetInfo();
}

ndMaterial ndContactCallback::GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
{
	ndMaterailKey key(instance0.GetMaterial().m_userId, instance1.GetMaterial().m_userId);
	dTree<ndMaterial, ndMaterailKey>::dTreeNode* const node = m_materialMap.Find(key);
	return node ? node->GetInfo() : ndMaterial();
}

bool ndContactCallback::OnAaabbOverlap(const ndContact* const contactJoint, dFloat32 timestep)
{
	return true;
}

void ndContactCallback::PlaySoundTest(const ndContact* const contactJoint)
{
	const ndBodyKinematic* const body0 = contactJoint->GetBody0();
	const ndBodyKinematic* const body1 = contactJoint->GetBody0();
	const ndContactPointList& contactPoints = contactJoint->GetContactPoints();

	dFloat32 maxSpeed = dFloat32 (0.0f);
	const ndContactMaterial* contact = nullptr;
	for (ndContactPointList::dListNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	{
		const ndContactMaterial& contactPoint = contactNode->GetInfo();
		const dVector pointVeloc0(body0->GetVelocityAtPoint(contactPoint.m_point));
		const dVector pointVeloc1(body1->GetVelocityAtPoint(contactPoint.m_point));

		dFloat32 speed = dAbs(contactPoint.m_normal.DotProduct(pointVeloc1 - pointVeloc0).GetScalar());
		if (speed > maxSpeed)
		{
			maxSpeed = speed;
			contact = &contactPoint;
		}
	}

	if (maxSpeed > 10.0f)
	{
		// playsound here;

	}
}

void ndContactCallback::OnContactCallback(dInt32 threadIndex, const ndContact* const contactJoint, dFloat32 timestep)
{
	const ndMaterial& material = contactJoint->GetMaterial();
	if (material.m_userFlags & playSound)
	{
		PlaySoundTest(contactJoint);
	}
}
