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
#ifndef __ND_RENDER_PRIMITIVE_MESH_H__
#define __ND_RENDER_PRIMITIVE_MESH_H__

#include "ndRenderStdafx.h"
#include "ndRenderPrimitive.h"

class ndRender;
class ndRenderTexture;
class ndRenderPrimitiveMeshImplement;

class ndRenderPrimitiveMeshMaterial
{
	public:
	ndRenderPrimitiveMeshMaterial();

	ndVector m_diffuse;
	ndVector m_specular;
	ndVector m_reflection;
	ndFloat32 m_specularPower;
	ndFloat32 m_opacity;
	ndSharedPtr<ndRenderTexture> m_texture;
	bool m_castShadows;
};

class ndRenderPrimitiveMeshSegment
{
	public:
	ndRenderPrimitiveMeshSegment();

	ndRenderPrimitiveMeshMaterial m_material;
	ndInt32 m_indexCount;
	ndInt32 m_segmentStart;
	bool m_hasTranparency;
};

class ndRenderPrimitiveMesh : public ndRenderPrimitive
{
	public:
	enum ndUvMapingMode
	{
		m_box,
		m_spherical,
		m_cylindrical
	};

	enum ndDebugMode
	{
		m_none,
		m_solid,
		m_wireFrame,
		m_hidenLines,
	};

	class ndDescriptor
	{
		public:
		ndDescriptor(ndRender* const render)
			:m_render(render)
			,m_collision(nullptr)
			,m_material()
			,m_mapping(m_box)
			,m_uvMatrix(ndGetIdentityMatrix())
			,m_debugMode(m_none)
			,m_stretchMaping(true)
		{
		}

		ndRender* m_render;
		const ndShapeInstance* m_collision;
		ndRenderPrimitiveMeshMaterial m_material;
		ndUvMapingMode m_mapping;
		ndMatrix m_uvMatrix;
		ndDebugMode m_debugMode;
		bool m_stretchMaping;
	};

	ndRenderPrimitiveMesh();
	virtual ~ndRenderPrimitiveMesh();

	virtual void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderPassMode) const override;

	static ndSharedPtr<ndRenderPrimitive> CreateMeshPrimitive(const ndDescriptor& descriptor);

	ndList<ndRenderPrimitiveMeshSegment> m_segments;
	ndSharedPtr<ndRenderPrimitiveMeshImplement> m_implement;
};

#endif