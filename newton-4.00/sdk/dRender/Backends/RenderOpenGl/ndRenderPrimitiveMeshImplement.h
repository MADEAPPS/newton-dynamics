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
#include "ndRenderShaderCache.h"
#include "ndRenderPrimitiveMesh.h"

class glPositionNormalUV;
class ndRenderPrimitiveMeshSegment;

class ndRenderPrimitiveMeshImplement : public ndContainersFreeListAlloc<ndRenderPrimitiveMeshImplement>
{
	public:
	ndRenderPrimitiveMeshImplement(ndRenderPrimitiveMesh* const owner, const ndRenderPrimitiveMesh::ndDescriptor& descriptor);

	~ndRenderPrimitiveMeshImplement();

	void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderMode) const;
	
	private:
	void OptimizeForRender(
		const glPositionNormalUV* const points, ndInt32 pointCount,
		const ndInt32* const indices, ndInt32 indexCount);

	void BuildRenderMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildDebugFlatShadedMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildRenderInstanceMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildWireframeDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildSetZBufferDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);

	void RenderGenerateShadowMaps(const ndRender* const render, const ndMatrix& lightMatrix) const;
	void RenderDirectionslDifuseColorNoShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDebugSetZbuffer(const ndRender* const render, const ndMatrix& modelMatrix) const;
	void RenderDebugShapeSolid(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDirectionslDifuseColorShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDebugShapeWireFrame(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderTransparency(const ndRender* const render, const ndMatrix& modelViewMatrix, bool backface) const;

	ndRenderPrimitiveMesh* m_owner;
	const ndRenderContext* m_context;

	GLint m_indexCount;
	GLint m_vertexCount;

	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;

	struct SolidColorBlock: public ndRenderShaderDebugFlatShadedDiffusedBlock
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
	 
	//struct InstancedSolidShadowColorBlock : public SolidShadowColorBlock
	//{
	//	GLint m_matrixPalette;
	//};
	
	TransparentColorBlock m_transparencyColorBlock;
	//InstancedSolidShadowColorBlock m_instancedShadowColorBlock;
	
	ndRenderShaderSetZbufferCleanBlock m_setZbufferBlock;
	ndRenderShaderDebugWireframeDiffuseBlock m_debugWireframeColorBlock;
	ndRenderShaderDebugFlatShadedDiffusedBlock m_debugFlatShadedColorBlock;
	ndRenderShaderOpaqueDiffusedColorBlock m_opaqueDifusedColorNoShadowBlock;
	ndRenderShaderOpaqueDiffusedShadowColorBlock m_opaqueDifusedColorShadowBlock;

	friend class ndRenderShaderSetZbufferCleanBlock;
	friend class ndRenderShaderGenerateShadowMapBlock;
	friend class ndRenderShaderOpaqueDiffusedColorBlock;
	friend class ndRenderShaderDebugWireframeDiffuseBlock;
	friend class ndRenderShaderDebugFlatShadedDiffusedBlock;
	friend class ndRenderShaderOpaqueDiffusedShadowColorBlock;
};

#endif