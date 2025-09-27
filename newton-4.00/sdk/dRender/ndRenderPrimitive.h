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
#ifndef __ND_RENDER_PRIMITIVE_H__
#define __ND_RENDER_PRIMITIVE_H__

#include "ndRenderStdafx.h"

class ndRenderPassShadowsImplement;

enum ndRenderPassMode
{
	m_generateShadowMaps,
	m_transparencyBackface,
	m_transparencyFrontface,
	m_debugDisplaySolidMesh,
	m_debugDisplaySetZbuffer,
	m_debugDisplayWireFrameMesh,
	m_directionalDiffusseShadow,
	m_directionalDiffusseNoShadow,
	m_m_generateInstanceShadowMaps,
	m_directionalDiffusseInstanceShadow,
};

class ndRenderPrimitive : public ndContainersFreeListAlloc<ndRenderPrimitive>
{
	public:
	ndRenderPrimitive();
	ndRenderPrimitive(const ndRenderPrimitive& src);
	virtual ~ndRenderPrimitive();

	virtual bool IsSKinnedMesh() const = 0;
	virtual ndRenderPrimitive* Clone() = 0;
	virtual void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderPassMode) const = 0;
};

#endif