/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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

void ndContactCallback::OnBodyAdded(ndBodyKinematic* const) const
{
}

void ndContactCallback::OnBodyRemoved(ndBodyKinematic* const) const
{
}

ndMaterial& ndContactCallback::RegisterMaterial(dUnsigned32 id0, dUnsigned32 id1)
{
	ndMaterailKey key(id0, id1);
	dTree<ndMaterial, ndMaterailKey>::dNode* node = m_materialMap.Find(key);
	if (!node)
	{
		node = m_materialMap.Insert(ndMaterial(), key);
	}
	return node->GetInfo();
}

//ndMaterial ndContactCallback::GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
ndMaterial ndContactCallback::GetMaterial(const ndContact* const, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
{
	ndMaterailKey key(instance0.GetMaterial().m_userId, instance1.GetMaterial().m_userId);
	dTree<ndMaterial, ndMaterailKey>::dNode* const node = m_materialMap.Find(key);
	return node ? node->GetInfo() : ndMaterial();
}

bool ndContactCallback::OnAabbOverlap(const ndContact* const contactJoint, dFloat32)
{
	const ndBodyKinematic* const body0 = contactJoint->GetBody0();
	const ndBodyKinematic* const body1 = contactJoint->GetBody1();

	const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
	const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
	if ((instanceShape0.GetUserDataID() == m_dedris) && (instanceShape1.GetUserDataID() == m_dedris))
	{
		return false;
	}

	return true;
}

void ndContactCallback::PlaySoundTest(const ndContact* const contactJoint)
{
	const ndBodyKinematic* const body0 = contactJoint->GetBody0();
	const ndBodyKinematic* const body1 = contactJoint->GetBody1();
	const ndContactPointList& contactPoints = contactJoint->GetContactPoints();

	dFloat32 maxNornalSpeed = dFloat32 (0.0f);
	dFloat32 maxTangentSpeed = dFloat32(0.0f);
	const ndContactMaterial* normalContact = nullptr;
	const ndContactMaterial* tangentContact = nullptr;
	for (ndContactPointList::dNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	{
		const ndContactMaterial& contactPoint = contactNode->GetInfo();
		const dVector pointVeloc0(body0->GetVelocityAtPoint(contactPoint.m_point));
		const dVector pointVeloc1(body1->GetVelocityAtPoint(contactPoint.m_point));
		const dVector veloc(pointVeloc1 - pointVeloc0);

		const dFloat32 verticalSpeed = contactPoint.m_normal.DotProduct(veloc).GetScalar();
		const dFloat32 nornalSpeed = dAbs(verticalSpeed);
		if (nornalSpeed > maxNornalSpeed)
		{
			maxNornalSpeed = nornalSpeed;
			normalContact = &contactPoint;
		}

		dVector tangVeloc(veloc - contactPoint.m_normal.Scale(verticalSpeed));
		const dFloat32 tangentSpeed = tangVeloc.DotProduct(tangVeloc).GetScalar();
		if (tangentSpeed > maxTangentSpeed)
		{
			maxTangentSpeed = tangentSpeed;
			tangentContact = &contactPoint;
		}
	}

	const ndShapeInstance& instance0 = body0->GetCollisionShape();
	const ndShapeInstance& instance1 = body1->GetCollisionShape();
	const dFloat32 speedThreshold = dMax(instance0.GetMaterial().m_userParam[0].m_floatData, instance1.GetMaterial().m_userParam[0].m_floatData);
	if (maxNornalSpeed > speedThreshold)
	{
		// play impact sound here;

	}

	maxTangentSpeed = dSqrt(maxTangentSpeed);
	if (maxTangentSpeed > speedThreshold)
	{
		// play scratching sound here;
	}
}

//void ndContactCallback::OnContactCallback(dInt32 threadIndex, const ndContact* const contactJoint, dFloat32 timestep)
void ndContactCallback::OnContactCallback(dInt32, const ndContact* const contactJoint, dFloat32)
{
	const ndMaterial& material = contactJoint->GetMaterial();
	if (material.m_userFlags & m_playSound)
	{
		PlaySoundTest(contactJoint);
	}
}
