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
#include "ndRenderSceneNode.h"
#include "ndRenderSceneNodeInstance.h"
#include "ndRenderSceneNodeInstanceImplement.h"

ndRenderSceneNodeInstance::ndRenderSceneNodeInstance(const ndMatrix& matrix, const ndRenderPrimitive::ndDescriptor& descriptor)
	:ndRenderSceneNode(matrix)
	,m_descriptor(descriptor)
	,m_implement()
	,m_isInitialized(false)
{
	m_descriptor.m_meshBuildMode = ndRenderPrimitive::m_instancePrimitve;
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

void ndRenderSceneNodeInstance::Render(const ndRender* const owner, const ndMatrix& modelViewMatrix, ndRenderPassMode renderMode) const
{
	// instance node do not recurse, the just render all the child intances.
	ndAssert(m_isInitialized);
	if (m_isInitialized)
	{
		//m_implement->Render(owner, m_matrix * modelViewMatrix, renderMode);
		m_implement->Render(owner, modelViewMatrix, renderMode);
	}
}
