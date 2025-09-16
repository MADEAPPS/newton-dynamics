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
	enum ndMeshBuildMode
	{
		m_simplePrimitve,
		m_instancePrimitve,
		m_debugFlatShaded,
		m_debugWireFrame,
		m_debugHiddenLines,
	};

	enum ndUvMapingMode
	{
		m_box,
		m_capsule,
		m_spherical,
		m_cylindrical
	};

	class ndDescriptor
	{
		public:
		ndDescriptor(ndRender* const render)
			:m_render(render)
			,m_meshNode____(nullptr)
			,m_collision(nullptr)
			,m_materials()
			,m_mapping(m_box)
			,m_uvMatrix(ndGetIdentityMatrix())
			,m_meshBuildMode(m_simplePrimitve)
			,m_numberOfInstances(0)
			,m_stretchMaping(true)
		{
		}

		ndDescriptor(const ndDescriptor& src)
			:m_render(src.m_render)
			,m_meshNode____(src.m_meshNode____)
			,m_collision(src.m_collision)
			,m_materials()
			,m_mapping(src.m_mapping)
			,m_uvMatrix(src.m_uvMatrix)
			,m_meshBuildMode(src.m_meshBuildMode)
			,m_numberOfInstances(src.m_numberOfInstances)
			,m_stretchMaping(src.m_stretchMaping)
		{
			for (ndList<ndRenderPrimitiveMeshMaterial>::ndNode* node = src.m_materials.GetFirst(); node; node = node->GetNext())
			{
				m_materials.Append(node->GetInfo());
			}
		}

		ndRenderPrimitiveMeshMaterial& AddMaterial(const ndSharedPtr<ndRenderTexture>& texture)
		{
			ndList<ndRenderPrimitiveMeshMaterial>::ndNode* const node = m_materials.Append();
			node->GetInfo().m_texture = texture;
			return node->GetInfo();
		}

		ndRender* m_render;
		ndSharedPtr<ndMeshEffect> m_meshNode____;
		const ndShapeInstance* m_collision;
		ndList<ndRenderPrimitiveMeshMaterial> m_materials;
		ndUvMapingMode m_mapping;
		ndMatrix m_uvMatrix;
		ndMeshBuildMode m_meshBuildMode;
		ndInt32 m_numberOfInstances;
		bool m_stretchMaping;
	};

	ndRenderPrimitiveMesh();
	ndRenderPrimitiveMesh(const ndRenderPrimitiveMesh& src);
	virtual ~ndRenderPrimitiveMesh();

	virtual ndRenderPrimitive* Clone() override;
	virtual void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderPassMode) const override;

	static ndSharedPtr<ndRenderPrimitive> CreateMeshPrimitive(const ndDescriptor& descriptor);

	ndList<ndRenderPrimitiveMeshSegment> m_segments;
	ndSharedPtr<ndRenderPrimitiveMeshImplement> m_implement;
};

#endif