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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderTexture.h"
#include "ndRenderSceneNodeInstance.h"
#include "ndRenderSceneNodeInstanceImplement.h"

ndRenderSceneNodeInstance::ndRenderSceneNodeInstance(const ndMatrix& matrix, const ndRenderPrimitiveMesh::ndDescriptor& descriptor)
	:ndRenderSceneNode(matrix)
	,m_savedShape(nullptr)
	,m_descriptor(descriptor)
	,m_implement()
	,m_isInitialized(false)
{
	if (m_descriptor.m_collision)
	{
		m_savedShape = ndSharedPtr<ndShapeInstance>(new ndShapeInstance(*m_descriptor.m_collision));
		m_descriptor.m_collision = *m_savedShape;
	}
	m_descriptor.m_meshBuildMode = ndRenderPrimitiveMesh::m_instancePrimitve;
	m_implement = ndSharedPtr<ndRenderSceneNodeInstanceImplement>(new ndRenderSceneNodeInstanceImplement(this));
}

ndRenderSceneNodeInstance* ndRenderSceneNodeInstance::GetAsInstance()
{
	return this;
}

const ndRenderSceneNodeInstance* ndRenderSceneNodeInstance::GetAsInstance() const
{
	return this;
}

void ndRenderSceneNodeInstance::Finalize()
{
	m_isInitialized = true;
	m_descriptor.m_numberOfInstances = ndInt32 (m_children.GetCount());
	m_implement->Finalize();
}

void ndRenderSceneNodeInstance::Render(const ndRender* const owner, ndFloat32 timeStep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const
{
	// instance node do not recurse, the just render all the child intances.
	ndAssert(m_isInitialized);
	if (m_isInitialized)
	{
		m_implement->Render(owner, timeStep, m_matrix * parentMatrix, renderMode);
	}
}
