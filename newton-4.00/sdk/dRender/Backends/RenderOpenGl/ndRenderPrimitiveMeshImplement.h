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
#include "ndRenderShader.h"
#include "ndRenderContext.h"
#include "ndRenderOpenGlUtil.h"
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
	void BuildRenderMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildDebugFlatShadedMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildRenderInstanceMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildWireframeDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);
	void BuildSetZBufferDebugMesh(const ndRenderPrimitiveMesh::ndDescriptor& descriptor);

	void RenderGenerateShadowMaps(const ndRender* const render, const ndMatrix& lightMatrix) const;
	void RenderDirectionalDiffuseColorNoShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDebugSetZbuffer(const ndRender* const render, const ndMatrix& modelMatrix) const;
	void RenderDebugShapeSolid(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDebugShapeWireFrame(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderTransparency(const ndRender* const render, const ndMatrix& modelViewMatrix, bool backface) const;
	void RenderDirectionalDiffuseColorShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDirectionalDiffuseColorInstanceShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;

	ndRenderPrimitiveMesh* m_owner;
	const ndRenderContext* m_context;
	ndArray<glMatrix> m_instanceRenderMatrixPallete;

	GLint m_indexCount;
	GLint m_vertexCount;

	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	GLuint m_instanceRenderMatrixPalleteBuffer;
	
	ndRenderShaderSetZbufferCleanBlock m_setZbufferBlock;
	ndRenderShaderDebugWireframeDiffuseBlock m_debugWireframeColorBlock;
	ndRenderShaderDebugFlatShadedDiffusedBlock m_debugFlatShadedColorBlock;
	ndRenderShaderOpaqueDiffusedColorBlock m_opaqueDifusedColorNoShadowBlock;
	ndRenderShaderOpaqueDiffusedShadowColorBlock m_opaqueDifusedColorShadowBlock;
	ndRenderShaderTransparentDiffusedShadowColorBlock m_transparencyDiffusedBlock;
	ndRenderShaderInstancedOpaqueDiffusedShadowBlock m_opaqueDifusedColorNoShadowInstanceBlock;

	friend class ndRenderSceneNodeInstanceImplement;
	friend class ndRenderShaderSetZbufferCleanBlock;
	friend class ndRenderShaderGenerateShadowMapBlock;
	friend class ndRenderShaderOpaqueDiffusedColorBlock;
	friend class ndRenderShaderDebugWireframeDiffuseBlock;
	friend class ndRenderShaderDebugFlatShadedDiffusedBlock;
	friend class ndRenderShaderOpaqueDiffusedShadowColorBlock;
	friend class ndRenderShaderInstancedOpaqueDiffusedShadowBlock;
	friend class ndRenderShaderTransparentDiffusedShadowColorBlock;
};

#endif