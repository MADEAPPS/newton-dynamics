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
#include "ndRenderTexture.h"
#include "ndRenderPrimitive.h"

ndRenderPrimitiveMaterial::ndRenderPrimitiveMaterial()
	:m_diffuse(ndFloat32(1.0f))
	,m_specular(ndFloat32(1.0f))
	,m_reflection(ndFloat32(0.5f))
	,m_specularPower(ndFloat32(250.0f))
	,m_opacity(ndFloat32(1.0f))
	,m_texture(nullptr)
	,m_castShadows(true)
{
}

ndRenderPrimitiveMaterial::ndRenderPrimitiveMaterial(const ndRenderPrimitiveMaterial& src)
	:m_diffuse(src.m_diffuse)
	,m_specular(src.m_specular)
	,m_reflection(src.m_reflection)
	,m_specularPower(src.m_specularPower)
	,m_opacity(src.m_opacity)
	,m_texture(src.m_texture)
	,m_castShadows(src.m_castShadows)
{
}


ndRenderPrimitive::ndRenderPrimitive()
	:ndContainersFreeListAlloc<ndRenderPrimitive>()
{
}

ndRenderPrimitive::ndRenderPrimitive(const ndRenderPrimitive&)
	:ndContainersFreeListAlloc<ndRenderPrimitive>()
{
}

ndRenderPrimitive::~ndRenderPrimitive()
{
}


