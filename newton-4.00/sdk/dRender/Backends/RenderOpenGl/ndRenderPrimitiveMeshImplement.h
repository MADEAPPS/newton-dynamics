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
#ifndef __ND_RENDER_PRIMITIVE_MESH_IMPLEMENT_H__
#define __ND_RENDER_PRIMITIVE_MESH_IMPLEMENT_H__

#include "ndRenderStdafx.h"
#include "ndRenderContext.h"
#include "ndRenderPrimitiveMesh.h"

class glPositionNormalUV;
class ndRenderPrimitiveMeshSegment;

class ndRenderPrimitiveMeshImplement : public ndContainersFreeListAlloc<ndRenderPrimitiveMeshImplement>
{
	public:
	ndRenderPrimitiveMeshImplement(ndRenderPrimitiveMesh* const owner, const ndRenderPrimitiveMesh::ndDescriptor& descriptor);

	//ndRenderPrimitiveMeshImplement(
	//	ndRenderPrimitiveMesh* const owner, 
	//	const ndRender* const render, 
	//	const ndShapeInstance* const collision,
	//	ndRenderPrimitiveMesh::ndMeshBuildMode mode);

	~ndRenderPrimitiveMeshImplement();

	void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderMode) const;
	
	private:
	void ResetOptimization____();
	void OptimizeForRender(
		const glPositionNormalUV* const points, ndInt32 pointCount,
		const ndInt32* const indices, ndInt32 indexCount);

	void BuildRenderMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildSolidDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildRenderInstanceMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildWireframeDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildSetZBufferDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);

	void RenderShadowMap(const ndRender* const render, const ndMatrix& lightMatrix) const;
	void RenderSolidColor(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDebugSetZbuffer(const ndRender* const render, const ndMatrix& modelMatrix) const;
	void RenderDebugShapeSolid(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderShadowSolidColor(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDebugShapeWireFrame(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderTransparency(const ndRender* const render, const ndMatrix& modelViewMatrix, bool backface) const;

	ndRenderPrimitiveMesh* m_owner;
	const ndRenderContext* m_context;

	GLint m_indexCount;
	GLint m_vertexCount;

	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;

	struct ShaderBlockBlock
	{
		virtual ~ShaderBlockBlock() {}

		virtual void GetShaderParameters(ndRenderPrimitiveMeshImplement* const self) = 0;
		virtual void Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const = 0;
	};

	struct SetZbufferCleanBlock: public ShaderBlockBlock
	{
		virtual void GetShaderParameters(ndRenderPrimitiveMeshImplement* const self) override;
		virtual void Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

		GLint viewModelProjectionMatrix;
	};

	struct DebugSolidColorBlock
	{
		GLint m_diffuseColor;
		GLint m_directionalLightAmbient;
		GLint m_directionalLightIntesity;
		GLint m_directionalLightDirection;
		GLint m_projectMatrixLocation;
		GLint m_viewModelMatrixLocation;
	};

	struct SolidColorBlock: public DebugSolidColorBlock
	{
		GLint m_texture;
		GLint m_environmentMap;
		GLint m_specularColor;
		GLint m_specularAlpha;
		GLint m_reflectionColor;
	};

	struct TransparentColorBlock : public SolidColorBlock
	{
		GLint m_opacity;
	};

	struct SolidShadowColorBlock : public SolidColorBlock
	{
		GLint m_worldMatrix;
		GLint m_shadowSlices;
		GLint m_depthMapTexture;
		GLint m_directionLightViewProjectionMatrixShadow;
	};

	struct InstancedSolidShadowColorBlock : public SolidShadowColorBlock
	{
		GLint m_matrixPalette;
	};

	SolidColorBlock m_solidColorBlock;
	SetZbufferCleanBlock m_setZbufferBlock;
	DebugSolidColorBlock m_debugSolidColorBlock;
	SolidShadowColorBlock m_solidShadowColorBlock;
	TransparentColorBlock m_transparencyColorBlock;
	InstancedSolidShadowColorBlock m_instancedShadowColorBlock;
};

#endif