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

class ndRender;
class ndRenderTexture;
class ndRenderSceneNode;
class ndRenderPrimitiveImplement;
class ndRenderPassShadowsImplement;

enum ndRenderPassMode
{
	m_generateShadowMaps,
	m_transparencyBackface,
	m_transparencyFrontface,
	m_debugLineArray,
	m_debugPointArray,
	m_debugDisplaySolidMesh,
	m_debugDisplaySetZbuffer,
	m_debugDisplayWireFrameMesh,
	m_directionalDiffusseShadow,
	m_directionalDiffusseNoShadow,
	m_m_generateInstanceShadowMaps,
	m_directionalDiffusseInstanceShadow,
};

class ndRenderPrimitiveMaterial
{
	public:
	ndRenderPrimitiveMaterial();
	ndRenderPrimitiveMaterial(const ndRenderPrimitiveMaterial& src);

	ndVector m_diffuse;
	ndVector m_specular;
	ndVector m_reflection;
	ndReal m_specularPower; 
	ndReal m_opacity;
	ndSharedPtr<ndRenderTexture> m_texture;
	bool m_castShadows;
};

class ndRenderPrimitiveSegment
{
	public:
	ndRenderPrimitiveSegment();
	ndRenderPrimitiveSegment(const ndRenderPrimitiveSegment& src);

	ndRenderPrimitiveMaterial m_material;
	ndInt32 m_indexCount;
	ndInt32 m_segmentStart;
	bool m_hasTranparency;
};

class ndRenderPrimitiveSimpleMesh : public ndClassAlloc
{
	public:
	enum MeshType
	{
		m_lines,
		m_points,
		m_triangles,
	};
	ndArray<ndVector> m_vertex;
	ndArray<ndVector> m_color;
	MeshType m_type;
};

class ndRenderPrimitive : public ndContainersFreeListAlloc<ndRenderPrimitive>
{
	public:

	enum ndMeshBuildMode
	{
		m_simplePrimitve,
		m_instancePrimitve,
		m_debugFlatShaded,
		m_debugWireFrame,
		m_debugHiddenLines,
		m_debugLineArray,
		m_debugPointArray,
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
		ndDescriptor(ndRender* const render);
		ndDescriptor(const ndDescriptor& src);
		ndRenderPrimitiveMaterial& AddMaterial(const ndSharedPtr<ndRenderTexture>& texture);
	
		ndRender* m_render;
		ndSharedPtr<ndMeshEffect> m_meshNode;
		ndSharedPtr<ndRenderPrimitiveSimpleMesh> m_simpleMesh;
		ndSharedPtr<ndShapeInstance> m_collision;
		ndSharedPtr<ndRenderSceneNode> m_skeleton;
		ndList<ndRenderPrimitiveMaterial> m_materials;
		ndUvMapingMode m_mapping;
		ndMatrix m_uvMatrix;
		ndMeshBuildMode m_meshBuildMode;
		ndInt32 m_numberOfInstances;
		bool m_stretchMaping;
	};

	ndRenderPrimitive();
	ndRenderPrimitive(const ndRenderPrimitive& src);
	ndRenderPrimitive(const ndDescriptor& descriptor);
	ndRenderPrimitive(const ndRenderPrimitive& src, const ndRenderSceneNode* const skeleton);
	virtual ~ndRenderPrimitive();

	bool IsSKinnedMesh() const;
	void UpdateSkinPaletteMatrix();
	void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderPassMode) const;

	ndList<ndRenderPrimitiveSegment> m_segments;
	ndSharedPtr<ndRenderPrimitiveImplement> m_implement;
};

#endif