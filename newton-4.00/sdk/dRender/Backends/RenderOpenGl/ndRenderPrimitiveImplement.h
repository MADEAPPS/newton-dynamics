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
#include "ndRenderPrimitive.h"
#include "ndRenderOpenGlUtil.h"
#include "ndRenderShaderCache.h"

class glPositionNormalUV;
class ndRenderPrimitiveSegment;

class ndRenderPrimitiveImplement : public ndContainersFreeListAlloc<ndRenderPrimitiveImplement>
{
	public:
	ndRenderPrimitiveImplement(const ndRenderPrimitiveImplement& src);
	ndRenderPrimitiveImplement(const ndRenderPrimitiveImplement& src, const ndRenderSceneNode* const srcSkeleton);
	ndRenderPrimitiveImplement(ndRenderPrimitive* const owner, const ndRenderPrimitive::ndDescriptor& descriptor);
	~ndRenderPrimitiveImplement();

	ndRenderPrimitiveImplement* Clone(ndRenderPrimitive* const owner, const ndRenderSceneNode* const skeletonOwner) const;

	void BuildFromMesh(const ndRenderPrimitive::ndDescriptor& descriptor);
	void BuildFromCollisionShape(const ndRenderPrimitive::ndDescriptor& descriptor);

	bool IsSKinnedMesh() const;
	void UpdateSkinPaletteMatrix();
	void Render(const ndRender* const render, const ndMatrix& modelViewMatrix, ndRenderPassMode renderMode) const;
	
	private:
	void InitShaderBlocks();
	void BuildRenderMeshFromCollisionShape(const ndRenderPrimitive::ndDescriptor& descriptor);
	void BuildRenderSimpleMeshFromMeshEffect(const ndRenderPrimitive::ndDescriptor& descriptor);
	void BuildRenderSkinnedMeshFromMeshEffect(const ndRenderPrimitive::ndDescriptor& descriptor);
	void BuildDebugFlatShadedMesh(const ndRenderPrimitive::ndDescriptor& descriptor);
	void BuildRenderInstanceMesh(const ndRenderPrimitive::ndDescriptor& descriptor);
	void BuildWireframeDebugMesh(const ndRenderPrimitive::ndDescriptor& descriptor);
	void BuildSetZBufferDebugMesh(const ndRenderPrimitive::ndDescriptor& descriptor);

	void RenderDebugSetZbuffer(const ndRender* const render, const ndMatrix& modelMatrix) const;
	void RenderGenerateShadowMaps(const ndRender* const render, const ndMatrix& lightMatrix) const;
	void RenderDebugShapeSolid(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDebugShapeWireFrame(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderGenerateInstancedShadowMaps(const ndRender* const render, const ndMatrix& lightMatrix) const;
	void RenderTransparency(const ndRender* const render, const ndMatrix& modelViewMatrix, bool backface) const;
	void RenderDirectionalDiffuseColorShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDirectionalDiffuseColorNoShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderDirectionalDiffuseColorInstanceShadow(const ndRender* const render, const ndMatrix& modelViewMatrix) const;

	ndRenderPrimitive* m_owner;
	const ndRenderContext* m_context;
	ndRenderSceneNode* m_skinSceneNode;
	ndArray<ndRenderSceneNode*> m_skeleton;
	ndArray<glMatrix> m_genericMatricArray;
	ndArray<ndMatrix> m_bindingSkinMatrixArray;

	GLint m_indexCount;
	GLint m_vertexCount;
	GLint m_vertexSize;

	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	GLuint m_matrixPaletteBuffer;
	
	ndRenderShaderOpaqueDiffusedColorBlock m_opaqueDifusedColorNoShadowBlock;
	ndRenderShaderOpaqueDiffusedShadowColorBlock m_opaqueDiffusedColorShadowBlock;
	ndRenderShaderTransparentDiffusedShadowColorBlock m_transparencyDiffusedBlock;
	ndRenderShaderOpaqueDiffusedShadowSkinColorBlock m_opaqueDiffusedColorShadowSkinBlock;
	ndRenderShaderInstancedOpaqueDiffusedShadowBlock m_opaqueDifusedColorNoShadowInstanceBlock;

	ndRenderShaderGenerateShadowMapBlock m_generateShadowMapsBlock;
	ndRenderShaderGenerateSkinShadowMapBlock m_generateSkinShadowMapsBlock;
	ndRenderShaderGenerateInstanceShadowMapBlock m_generateIntanceShadowMapsBlock;

	ndRenderShaderSetZbufferCleanBlock m_setZbufferBlock;
	ndRenderShaderDebugWireframeDiffuseBlock m_debugWireframeColorBlock;
	ndRenderShaderDebugFlatShadedDiffusedBlock m_debugFlatShadedColorBlock;

	friend class ndRenderSceneNodeInstanceImplement;
	friend class ndRenderShaderSetZbufferCleanBlock;
	friend class ndRenderShaderGenerateShadowMapBlock;
	friend class ndRenderShaderOpaqueDiffusedColorBlock;
	friend class ndRenderShaderDebugWireframeDiffuseBlock;
	friend class ndRenderShaderDebugFlatShadedDiffusedBlock;
	friend class ndRenderShaderOpaqueDiffusedShadowColorBlock;
	friend class ndRenderShaderGenerateInstanceShadowMapBlock;
	friend class ndRenderShaderOpaqueDiffusedShadowSkinColorBlock;
	friend class ndRenderShaderInstancedOpaqueDiffusedShadowBlock;
	friend class ndRenderShaderTransparentDiffusedShadowColorBlock;
};

#endif