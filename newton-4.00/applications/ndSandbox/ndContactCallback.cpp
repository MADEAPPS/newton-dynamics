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

ndMaterial& ndContactCallback::RegisterMaterial(ndUnsigned32 id0, ndUnsigned32 id1)
{
	ndMaterailKey key(id0, id1);
	ndTree<ndMaterial, ndMaterailKey>::ndNode* node = m_materialMap.Find(key);
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
	ndTree<ndMaterial, ndMaterailKey>::ndNode* const node = m_materialMap.Find(key);
	return node ? node->GetInfo() : ndMaterial();
}

bool ndContactCallback::OnAabbOverlap(const ndContact* const contactJoint, ndFloat32)
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

	ndFloat32 maxNornalSpeed = ndFloat32 (0.0f);
	ndFloat32 maxTangentSpeed = ndFloat32(0.0f);
	const ndContactMaterial* normalContact = nullptr;
	const ndContactMaterial* tangentContact = nullptr;
	for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	{
		const ndContactMaterial& contactPoint = contactNode->GetInfo();
		const ndVector pointVeloc0(body0->GetVelocityAtPoint(contactPoint.m_point));
		const ndVector pointVeloc1(body1->GetVelocityAtPoint(contactPoint.m_point));
		const ndVector veloc(pointVeloc1 - pointVeloc0);

		const ndFloat32 verticalSpeed = contactPoint.m_normal.DotProduct(veloc).GetScalar();
		const ndFloat32 nornalSpeed = dAbs(verticalSpeed);
		if (nornalSpeed > maxNornalSpeed)
		{
			maxNornalSpeed = nornalSpeed;
			normalContact = &contactPoint;
		}

		ndVector tangVeloc(veloc - contactPoint.m_normal.Scale(verticalSpeed));
		const ndFloat32 tangentSpeed = tangVeloc.DotProduct(tangVeloc).GetScalar();
		if (tangentSpeed > maxTangentSpeed)
		{
			maxTangentSpeed = tangentSpeed;
			tangentContact = &contactPoint;
		}
	}

	const ndShapeInstance& instance0 = body0->GetCollisionShape();
	const ndShapeInstance& instance1 = body1->GetCollisionShape();
	const ndFloat32 speedThreshold = dMax(instance0.GetMaterial().m_userParam[0].m_floatData, instance1.GetMaterial().m_userParam[0].m_floatData);
	if (maxNornalSpeed > speedThreshold)
	{
		// play impact sound here;

	}

	maxTangentSpeed = ndSqrt(maxTangentSpeed);
	if (maxTangentSpeed > speedThreshold)
	{
		// play scratching sound here;
	}
}

//void ndContactCallback::OnContactCallback(ndInt32 threadIndex, const ndContact* const contactJoint, ndFloat32 timestep)
void ndContactCallback::OnContactCallback(ndInt32, const ndContact* const contactJoint, ndFloat32)
{
	const ndMaterial& material = contactJoint->GetMaterial();
	if (material.m_userFlags & m_playSound)
	{
		PlaySoundTest(contactJoint);
	}

#if 0
	ndContactSolver solver;
	ndFixSizeArray<ndContactPoint, 16> contactOut;
	ndBodyKinematic* bodyA = contactJoint->GetBody0();
	ndBodyKinematic* bodyB = contactJoint->GetBody1();
	const ndShapeInstance& shapeA = bodyA->GetCollisionShape();
	const ndShapeInstance& shapeB = bodyB->GetCollisionShape();
	solver.CalculateContacts(
		&shapeA, bodyA->GetMatrix(), bodyA->GetVelocity(),
		&shapeB, bodyB->GetMatrix(), bodyB->GetVelocity(),
		contactOut);
#endif
}
