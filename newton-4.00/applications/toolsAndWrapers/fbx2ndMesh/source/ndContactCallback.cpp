/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndModelStdafx.h"
#include "ndContactCallback.h"

ndMaterialGraph::~ndMaterialGraph()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndApplicationMaterial* const material = it.GetNode()->GetInfo();
		delete material;
	}
}

ndApplicationMaterial& ndContactCallback::RegisterMaterial(const ndApplicationMaterial& material, ndUnsigned32 id0, ndUnsigned32 id1)
{
	ndMaterialHash key(id0, id1);
	ndMaterialGraph::ndNode* node = m_materialGraph.Find(key);
	if (!node)
	{
		ndApplicationMaterial* const materialCopy = material.Clone();
		node = m_materialGraph.Insert(materialCopy, key);
	}
	return *node->GetInfo();
}

bool ndContactCallback::OnAabbOverlap(const ndContact* const contactJoint, ndFloat32 timestep) const
{
	const ndApplicationMaterial* const material = (ndApplicationMaterial*)contactJoint->GetMaterial();
	ndAssert(material);

	const ndBodyKinematic* const body0 = contactJoint->GetBody0();
	const ndBodyKinematic* const body1 = contactJoint->GetBody1();
	const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
	const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
	return material->OnAabbOverlap(contactJoint, timestep, instanceShape0, instanceShape1);
}

bool ndContactCallback::OnCompoundSubShapeOverlap(const ndContact* const contactJoint, ndFloat32 timestep, const ndShapeInstance* const instance0, const ndShapeInstance* const instance1) const
{
	const ndApplicationMaterial* const material = (ndApplicationMaterial*)contactJoint->GetMaterial();
	ndAssert(material);
	return material->OnAabbOverlap(contactJoint, timestep, *instance0, *instance1);
}

void ndContactCallback::OnContactCallback(const ndContact* const contactJoint, ndFloat32 timestep) const
{
	const ndApplicationMaterial* const material = (ndApplicationMaterial*)contactJoint->GetMaterial();
	ndAssert(material);
	material->OnContactCallback(contactJoint, timestep);
}
