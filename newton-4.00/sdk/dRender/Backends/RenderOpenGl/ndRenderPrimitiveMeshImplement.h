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
	enum ndDebugModeCreate
	{
		m_solid,
		m_wireFrame,
		m_hidenLines,
	};
	ndRenderPrimitiveMeshImplement(
		ndRenderPrimitiveMesh* const owner, 
		const ndRender* const render, 
		const ndShapeInstance* const collision,
		ndDebugModeCreate mode);

	ndRenderPrimitiveMeshImplement(
		ndRenderPrimitiveMesh* const owner,
		const ndRender* const render, const ndShapeInstance* const collision, 
		const ndRenderPrimitiveMeshMaterial& material, ndRenderPrimitiveMesh::ndUvMapingMode mapping,
		const ndMatrix& uvMatrix, bool stretchMaping);

	~ndRenderPrimitiveMeshImplement();

	void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderMode) const;
	
	private:
	void ResetOptimization();
	void OptimizeForRender(
		const glPositionNormalUV* const points, ndInt32 pointCount,
		const ndInt32* const indices, ndInt32 indexCount);

	void BuildSolidDebugMesh(const ndShapeInstance* const collision);
	void BuildWireframeDebugMesh(const ndShapeInstance* const collision);
	void BuildSetZBufferDebugMesh(const ndShapeInstance* const collision);

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

	struct DebugSolidColorBlock
	{
		GLint m_diffuseColor;
		GLint m_directionalLightAmbient;
		GLint m_directionalLightIntesity;
		GLint m_directionalLightDirection;
		GLint m_projectMatrixLocation;
		GLint m_viewModelMatrixLocation;
	};

	struct SetZbufferCleanBlock
	{
		GLint viewModelProjectionMatrix;
	};

	struct SolidColorBlock: public DebugSolidColorBlock
	{
		GLint m_texture;
		GLint m_environmentMap;
		GLint m_specularColor;
		GLint m_specularAlpha;
		GLint m_reflectionColor;
	};
	struct SolidShadowColorBlock : public SolidColorBlock
	{
		GLint m_worldMatrix;
		GLint m_shadowSlices;
		GLint m_depthMapTexture;
		GLint m_directionLightViewProjectionMatrixShadow;
	};

	struct TransparentColorBlock : public SolidColorBlock
	{
		GLint m_opacity;
	};

	SolidColorBlock m_solidColorBlock;
	SetZbufferCleanBlock m_setZbufferBlock;
	DebugSolidColorBlock m_debugSolidColorBlock;
	SolidShadowColorBlock m_solidShadowColorBlock;
	TransparentColorBlock m_transparencyColorBlock;
};

#endif