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
#include "ndRenderPrimitive.h"
#include "ndRenderSceneNodeInstance.h"
#include "ndRenderPrimitiveImplement.h"
#include "ndRenderSceneNodeInstanceImplement.h"

ndRenderSceneNodeInstanceImplement::ndRenderSceneNodeInstanceImplement(ndRenderSceneNodeInstance* const owner)
	:ndContainersFreeListAlloc<ndRenderSceneNodeInstanceImplement>()
	,m_owner(owner)
{
}

void ndRenderSceneNodeInstanceImplement::Finalize()
{
	ndRenderSceneNodeInstance* const owner = (ndRenderSceneNodeInstance*)m_owner;

	const ndRenderPrimitive::ndDescriptor& descriptor = owner->m_descriptor;
	ndAssert(descriptor.m_numberOfInstances > 0);
	ndAssert(descriptor.m_meshBuildMode == ndRenderPrimitive::m_instancePrimitve);
	
	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
	owner->SetPrimitive(mesh);
}

void ndRenderSceneNodeInstanceImplement::Render(const ndRender* const owner, const ndMatrix& modelViewMatrix, ndRenderPassMode renderMode) const
{
	if ((renderMode == m_m_generateInstanceShadowMaps) || (renderMode == m_directionalDiffusseShadow))
	{
		ndRenderPrimitive* const mesh = *m_owner->m_primitive;
		ndRenderPassMode overrideRenderMode = (renderMode == m_directionalDiffusseShadow) ? m_directionalDiffusseInstanceShadow : renderMode;
		mesh->Render(owner, modelViewMatrix, overrideRenderMode);
	}
}
